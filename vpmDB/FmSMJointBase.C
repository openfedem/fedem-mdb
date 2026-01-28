// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSMJointBase.H"
#include "vpmDB/FmAssemblyBase.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdBase.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcSM_JOINT_BASE, FmSMJointBase, FmJointBase);


FmSMJointBase::FmSMJointBase()
{
  Fmd_CONSTRUCTOR_INIT(FmSMJointBase);

  FFA_FIELD_INIT(IAmMovingMasterTriadAlong, true, "MOVE_MASTER_TRIAD_ALONG");
  FFA_FIELD_INIT(IAmMovingSlaveTriadAlong, true, "MOVE_SLAVE_TRIAD_ALONG");

  FFA_REFERENCE_FIELD_INIT(itsMasterTriadField, itsMasterTriad, "MASTER_TRIAD");
}


FmSMJointBase::~FmSMJointBase()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmSMJointBase::eraseOptions()
{
  FmTriad* triads[2] = { this->getItsMasterTriad(), this->getSlaveTriad() };

  this->removeItsMasterTriad();
  this->removeItsSlaveTriad();

  for (int i = 0; i < 2; i++)
    if (triads[i])
    {
      if (!triads[i]->hasReferences())
        triads[i]->erase();
      else
        triads[i]->updateTopologyInViewer();
    }

  return this->FmJointBase::eraseOptions();
}


bool FmSMJointBase::isMasterAttachedToLink(bool allowMultipleLinks) const
{
  FmTriad* triad = this->getItsMasterTriad();
  return triad ? triad->isAttached(false,allowMultipleLinks) : false;
}


FmLink* FmSMJointBase::getMasterLink() const
{
  FmTriad* triad = this->getItsMasterTriad();
  return triad ? triad->getOwnerLink() : NULL;
}


FmPart* FmSMJointBase::getMasterPart(bool noEarth) const
{
  FmTriad* triad = this->getItsMasterTriad();
  if (!triad) return NULL;

  FmPart* part = triad->getOwnerPart();
  if (!part) return NULL;

  return noEarth && part->isEarthLink() ? NULL : part;
}


void FmSMJointBase::getMasterTriads(std::vector<FmTriad*>& triadsToFill) const
{
  if (this->getItsMasterTriad())
    triadsToFill = { this->getItsMasterTriad() };
  else
    triadsToFill.clear();
}


bool FmSMJointBase::isMasterTriad(const FmTriad* triad) const
{
  if (triad)
    return this->getItsMasterTriad() == const_cast<FmTriad*>(triad);
  else
    return false;
}


FaMat34 FmSMJointBase::getGlobalCS() const
{
  // The local coordinate system is relative to its independent triad
  if (FmTriad* triad = this->getItsMasterTriad(); triad)
    return triad->getGlobalCS() * this->getLocalCS();

  // The local coordinate system is relative to the parent assembly (if any)
  FmAssemblyBase* parent = dynamic_cast<FmAssemblyBase*>(this->getParentAssembly());
  return parent ? parent->toGlobal(this->getLocalCS()) : this->getLocalCS();
}


/*!
  Sets the position of this joint to the provided matrix,
  taking into account the independent triad position.
  If the embedded triads are set to follow, they are also moved,
  if moveRelationsAlong = true.
  Anyway the dependent triad is moved to keep the joint angles constant.
*/

void FmSMJointBase::setGlobalCS(const FaMat34& globalMat,
				bool moveRelationsAlong)
{
  FmTriad* triad1 = this->getItsMasterTriad();
  FmTriad* triad2 = this->getSlaveTriad();
  FaVec3 jointAngles = this->getRotJointVariables();

  FaMat34 xf2ToJoint;
  if (triad2)
    xf2ToJoint = triad2->getGlobalCS().inverse() * this->getGlobalCS();

  if (!triad1)
    this->setLocalCS(globalMat);
  else if (moveRelationsAlong && this->isMasterMovedAlong())
  {
    triad1->setGlobalCS(globalMat * this->getLocalCS().inverse());
    triad1->updateDisplayTopology();
  }
  else
    this->setLocalCS(triad1->getGlobalCS().inverse() * globalMat);

  if (triad2 && moveRelationsAlong && this->isSlaveMovedAlong())
  {
    triad2->setGlobalCS(globalMat * xf2ToJoint.inverse());
    triad2->updateDisplayTopology();
  }
  else
    this->setRotJointVariables(jointAngles);
}


bool FmSMJointBase::isTranslatable() const
{
  if (this->isMasterMovedAlong())
    if (FmTriad* triad = this->getItsMasterTriad(); triad)
      if (!triad->isTranslatable(this))
        return false;

  if (this->isSlaveMovedAlong())
    if (FmTriad* triad = this->getSlaveTriad(); triad)
      if (!triad->isTranslatable(this))
        return false;

  return true;
}

char FmSMJointBase::isRotatable() const
{
  if (this->isMasterMovedAlong())
    if (FmTriad* triad = this->getItsMasterTriad(); triad)
      if (triad && !triad->isRotatable(this))
        return false;

  if (this->isSlaveMovedAlong())
    if (FmTriad* triad = this->getSlaveTriad(); triad)
      if (!triad->isRotatable(this))
        return false;

  return true;
}


FaVec3 FmSMJointBase::getTransJointVariables() const
{
  if (FmTriad* triad = this->getSlaveTriad(); triad)
    return this->getGlobalCS().inverse()*triad->getGlobalTranslation();
  else
    return this->getGlobalCS().inverse().translation();
}


FaVec3 FmSMJointBase::getRotJointVariables() const
{
  if (FmTriad* triad = this->getSlaveTriad(); triad)
    return this->getJointRotations(this->getGlobalCS(),triad->getGlobalCS());
  else
    return this->getJointRotations(this->getGlobalCS(),FaMat34());
}


void FmSMJointBase::setRotJointVariables(const FaVec3& rotations)
{
  this->setJointRotations(rotations,this->getGlobalCS());
}


bool FmSMJointBase::detach()
{
  bool isAttached1 = this->isSlaveAttachedToLink(true);
  bool isAttached2 = this->isMasterAttachedToLink(true);
  if (!isAttached1 && !isAttached2)
  {
    ListUI <<"Error : "<< this->getIdString() <<" is already detached.\n";
    return false;
  }

  if (isAttached1)
  {
    // detaching the dependent triad
    FmTriad* oldTriad = this->getSlaveTriad();
    FmTriad* newTriad = new FmTriad();
    newTriad->setParentAssembly(this->getParentAssembly());
    newTriad->connect();
    newTriad->setLocalCS(oldTriad->getGlobalCS());
    this->setAsSlaveTriad(newTriad);
    if (!oldTriad->hasReferences())
      oldTriad->erase();
#ifdef USE_INVENTOR
    else
      oldTriad->getFdPointer()->updateFdDetails();
#endif
  }

  if (isAttached2)
  {
    // detaching the independent triad
    FmTriad* oldTriad = this->getItsMasterTriad();
    FmTriad* newTriad = new FmTriad();
    newTriad->setParentAssembly(this->getParentAssembly());
    newTriad->connect();
    newTriad->setLocalCS(oldTriad->getGlobalCS());
    this->setAsMasterTriad(newTriad);
    if (!oldTriad->hasReferences())
      oldTriad->erase();
#ifdef USE_INVENTOR
    else
      oldTriad->getFdPointer()->updateFdDetails();
#endif
  }

  this->getItsMasterTriad()->draw();
  this->getSlaveTriad()->draw();
  return true;
}


bool FmSMJointBase::swapMasterAndSlave()
{
  FmTriad* triad1 = this->getItsMasterTriad();
  FmTriad* triad2 = this->getSlaveTriad();
  if (!triad1 || !triad2 || !triad1->isAttached() || !triad2->isAttached())
    ListUI <<"Error : "<< this->getIdString() <<" is not fully attached.\n";
  else if (triad1->isAttached(FmDB::getEarthLink()))
    ListUI <<"Error : "<< this->getIdString() <<" is attached to ground.\n";
  else if (triad1->isSlaveTriad(true))
    ListUI <<"Error : "<< triad1->getIdString() <<" is already dependent.\n";
  else
  {
    this->setAsSlaveTriad(triad1);
    this->setAsMasterTriad(triad2);
    triad1->onChanged();
    triad2->onChanged();
    ListUI <<"  => Swapping independent/dependent triads for "<< this->getIdString(true) <<".\n";
    return true;
  }

  ListUI <<"Error : Cannot swap triads for "<< this->getIdString(true);
  FFaMsg::list(".\n",true);
  return false;
}


bool FmSMJointBase::localParse(const char* keyWord, std::istream& activeStatement,
			       FmSMJointBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmSMJointBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmSMJointBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmSMJointBase* copyObj = static_cast<FmSMJointBase*>(obj);
  if (FmTriad* triad = copyObj->getItsMasterTriad(); triad)
  {
    if (depth == FmBase::DEEP_REPLACE)
      copyObj->removeItsMasterTriad();
    this->setAsMasterTriad(triad);
  }

  return true;
}


void FmSMJointBase::initAfterResolve()
{
  this->FmJointBase::initAfterResolve();

  this->setAsMasterTriad(itsMasterTriad);
}
