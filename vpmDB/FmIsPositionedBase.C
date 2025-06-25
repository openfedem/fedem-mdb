// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmIsPositionedBase.H"
#include "vpmDB/FmAssemblyBase.H"
#include "vpmDB/FmSticker.H"
#include "vpmDB/FmLoad.H"


Fmd_DB_SOURCE_INIT(FcIS_POSITIONED_BASE, FmIsPositionedBase, FmIsPlottedBase);


FmIsPositionedBase::FmIsPositionedBase()
{
  Fmd_CONSTRUCTOR_INIT(FmIsPositionedBase);

  FFA_FIELD_DEFAULT_INIT(myCS, "COORDINATE_SYSTEM");
  FFA_FIELD_DEFAULT_INIT(myLocation, "LOCATION3D_DATA");

  FFA_REFERENCE_FIELD_INIT(myPosRefField, myPosRef, "LOCATION3D_POS_VIEW_REF");
  FFA_REFERENCE_FIELD_INIT(myRotRefField, myRotRef, "LOCATION3D_ROT_VIEW_REF");

  // To minimize the model file size (most positions have global reference)
  myPosRef.setPrintIfZero(false);
  myRotRef.setPrintIfZero(false);

  IAmSettingLocation = false;
}


FmIsPositionedBase::~FmIsPositionedBase()
{
  // Removal of the loads connected
  FmLoad* load = NULL;
  while (this->hasReferringObjs(load,"myAttachedLinks"))
    load->erase();

  this->removeAllStickers();
}


void FmIsPositionedBase::updateChildrenDisplayTopology()
{
  std::vector<FmLoad*> loads;
  this->getReferringObjs(loads);
  for (FmLoad* load : loads)
    load->updateTopologyInViewer();

  FmIsPlottedBase::updateChildrenDisplayTopology();
}


void FmIsPositionedBase::setLocalCS(const FaMat34& localMat)
{
  myCS.setValue(localMat);
  this->updateLocation();
}


FaMat34 FmIsPositionedBase::getGlobalCS() const
{
  FmAssemblyBase* parent = this->getPositionedAssembly();
  if (parent)
    return parent->toGlobal(this->getLocalCS());
  else
    return this->getLocalCS();
}


void FmIsPositionedBase::setGlobalCS(const FaMat34& globalMat, bool)
{
  FmAssemblyBase* parent = this->getPositionedAssembly();
  if (parent)
    this->setLocalCS(parent->toLocal(globalMat));
  else
    this->setLocalCS(globalMat);
  this->updateLocation();
}


void FmIsPositionedBase::setTranslation(const FaVec3& tr)
{
  myCS.getValue()[3] = tr;
  this->updateLocation();
}


void FmIsPositionedBase::setOrientation(const FaMat33& orient)
{
  FaMat34& itsCS = myCS.getValue();
  itsCS[0] = orient[0];
  itsCS[1] = orient[1];
  itsCS[2] = orient[2];
  this->updateLocation();
}


FaVec3 FmIsPositionedBase::getTranslation() const
{
  return myCS.getValue().translation();
}


FaMat33 FmIsPositionedBase::getOrientation() const
{
  return myCS.getValue().direction();
}


FmAssemblyBase* FmIsPositionedBase::getPositionedAssembly() const
{
  return dynamic_cast<FmAssemblyBase*>(this->getParentAssembly());
}


void FmIsPositionedBase::getStickers(std::vector<FmSticker*>& stickers) const
{
  this->getLocalStickers(stickers);
}


void FmIsPositionedBase::getLocalStickers(std::vector<FmSticker*>& stickers) const
{
  // Note that the stickers std::vector is not cleared on entry
  this->getReferringObjs(stickers,"myOwner");
}


bool FmIsPositionedBase::hasStickers() const
{
  std::vector<FmSticker*> stickers;
  // Calling virtual method to catch stickers on triads on links
  this->getStickers(stickers);
  return !stickers.empty();
}


bool FmIsPositionedBase::removeSticker(FmSticker* sticker)
{
  if (sticker->getStuckObject() == this)
    return sticker->erase();
  else
    return false;
}


bool FmIsPositionedBase::addSticker(FmSticker* sticker)
{
  sticker->disconnect();
  return sticker->connect(this);
}


bool FmIsPositionedBase::addStickers(const std::vector<FmSticker*>& stickers)
{
  bool ret = true;
  for (FmSticker* sticker : stickers)
    ret &= this->addSticker(sticker);

  return ret;
}


void FmIsPositionedBase::removeAllStickers()
{
  std::vector<FmSticker*> stickers;
  this->getLocalStickers(stickers);
  for (FmSticker* sticker : stickers)
    sticker->erase();
}


int FmIsPositionedBase::getNumberOfStickers() const
{
  std::vector<FmSticker*> stickers;
  this->getLocalStickers(stickers);
  return stickers.size();
}


bool FmIsPositionedBase::localParse(const char* keyWord,
                                    std::istream& activeStatement,
                                    FmIsPositionedBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmIsPositionedBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmIsPositionedBase::getClassTypeID()))
    return false;

  FmIsPositionedBase* copyObj = static_cast<FmIsPositionedBase*>(obj);

  if (depth == FmBase::SHALLOW || depth >= FmBase::DEEP_APPEND)
  {
    myPosRef.copy(copyObj->myPosRef);
    myRotRef.copy(copyObj->myRotRef);
  }

  if (depth >= FmBase::DEEP_APPEND)
  {
    std::vector<FmLoad*> loads;
    copyObj->getReferringObjs(loads,"itsPos1");
    for (FmLoad* load : loads)
      load->setFromRef(this);

    loads.clear();
    copyObj->getReferringObjs(loads,"itsPos2");
    for (FmLoad* load : loads)
      load->setToRef(this);

    std::vector<FmSticker*> stickers;
    copyObj->getLocalStickers(stickers);
    this->addStickers(stickers);
  }

  return true;
}


bool FmIsPositionedBase::setPosRef(FmIsPositionedBase* ref)
{
  if (myPosRef.getRef() == ref)
    return false;

  FaMat34 newRefCS, oldRefCS;
  if (ref)      newRefCS = ref->getGlobalCS();
  if (myPosRef) oldRefCS = myPosRef->getGlobalCS();

  myLocation.getValue().changePosRefCS(newRefCS,oldRefCS);

  myPosRef.setRef(ref);
  return true;
}


bool FmIsPositionedBase::setRotRef(FmIsPositionedBase* ref)
{
  if (myRotRef.getRef() == ref)
    return false;

  FaMat34 newRefCS, oldRefCS;
  if (ref)      newRefCS = ref->getGlobalCS();
  if (myRotRef) oldRefCS = myRotRef->getGlobalCS();

  myLocation.getValue().changeRotRefCS(newRefCS,oldRefCS);

  myRotRef.setRef(ref);
  return true;
}


void FmIsPositionedBase::setLocation(const FFa3DLocation& loc)
{
  myLocation = loc;

  FaMat34 posRefCS, rotRefCS;
  if (myPosRef) posRefCS = myPosRef->getGlobalCS();
  if (myRotRef) rotRefCS = myRotRef->getGlobalCS();

  IAmSettingLocation = true;
  this->setGlobalCS(myLocation.getValue().getMatrix(posRefCS,rotRefCS),true);
  IAmSettingLocation = false;

  if (this->getPosRef() == this || this->getRotRef() == this)
    this->updateLocation();
}


/*!
  Updates the FFa3DLocation value from the position matrix and the
  reference coordinate systems.
  If updateReferringObjs is true, the objects using this as a
  CS reference are also updated.
*/

void FmIsPositionedBase::updateLocation(bool updateReferringObjs)
{
  // Update all locations in objects referring to this
  // as a position or rotation reference

  if (updateReferringObjs) {
    std::vector<FmIsPositionedBase*> posRefs;
    this->getReferringObjs(posRefs,"myPosRef");
    this->getReferringObjs(posRefs,"myRotRef");
    for (FmIsPositionedBase* ref : posRefs)
      ref->updateLocation(false);
  }

  if (IAmSettingLocation) return;

  FaMat34 posRefCS, rotRefCS;
  if (myPosRef) posRefCS = myPosRef->getGlobalCS();
  if (myRotRef) rotRefCS = myRotRef->getGlobalCS();

  myLocation.getValue().set(myLocation.getValue().getPosType(), posRefCS,
                            myLocation.getValue().getRotType(), rotRefCS,
                            this->getGlobalCS());
}
