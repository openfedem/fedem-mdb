// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFlLib/FFlFEParts/FFlNode.H"

#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdLoad.H"
#endif

#include "vpmDB/FmPart.H"
#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmLoad.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcLOAD, FmLoad, FmIsControlledBase);

FmLoad* FmLoad::editedLoad = NULL;


FmLoad::FmLoad()
{
  Fmd_CONSTRUCTOR_INIT(FmLoad);

  FFA_FIELD_INIT(itsLoad, 0.0, "VALUE");
  FFA_FIELD_INIT(itsLoadType, FORCE, "LOAD_TYPE");
  FFA_FIELD_DEFAULT_INIT(itsPoint[0], "FROM_POINT");
  FFA_FIELD_DEFAULT_INIT(itsPoint[1], "TO_POINT");

  FFA_REFERENCE_FIELD_INIT(itsOwnerField, itsOwnerTriad, "OWNER_TRIAD");

  FFA_REFERENCE_FIELD_INIT(itsPosField[0], itsPos[0], "FROM_OBJECT");
  FFA_REFERENCE_FIELD_INIT(itsPosField[1], itsPos[1], "TO_OBJECT");

#ifdef USE_INVENTOR
  itsDisplayPt = new FdLoad(this);
#endif
}


FmLoad::~FmLoad()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmLoad::connect(FmBase* parent)
{
  bool status = this->mainConnect();

  if (parent && parent->isOfType(FmTriad::getClassTypeID()))
    this->setOwnerTriad(static_cast<FmTriad*>(parent));

  return status;
}


bool FmLoad::connect(FmTriad* tr,
                     FmIsPositionedBase* l1, const FaVec3& p1,
                     FmIsPositionedBase* l2, const FaVec3& p2)
{
  bool status = this->mainConnect();

  this->setOwnerTriad(tr);

  this->setFromRef(l1);
  this->setLocalFromPoint(p1);

  this->setToRef(l2);
  this->setLocalToPoint(p2);

  return status;
}


bool FmLoad::disconnect()
{
  bool status = this->mainDisconnect();
  this->setOwnerTriad(NULL);

  return status;
}


void FmLoad::setRefPoint(FmIsPositionedBase* p, int idx)
{
  if (p && p->isOfType(FmRefPlane::getClassTypeID()))
    itsPos[idx].setRef(FmDB::getEarthLink());
  else
    itsPos[idx].setRef(p);
}


void FmLoad::moveAttackPoint(const FaVec3& pos, bool global,
                             FmIsPositionedBase* attackObj)
{
  if (!attackObj)
    if (FmTriad* ownerTriad = this->getOwnerTriad(); ownerTriad)
      attackObj = ownerTriad->getOwnerLink();

  if (attackObj && global)
    // Convert point from global to local coordinates for the attacked object
    this->changeAttackPoint(attackObj->getLocalCS().inverse()*pos, attackObj);
  else
    this->changeAttackPoint(pos, attackObj);
}


void FmLoad::changeAttackPoint(const FaVec3& localPos, FmIsPositionedBase* obj)
{
  FmTriad* newTriad = NULL;
  FmTriad* oldTriad = this->getOwnerTriad();
  if (FmLink* link  = dynamic_cast<FmLink*>(obj); link)
  {
    FaVec3 loadPos = localPos;
    double tolerance = FmDB::getPositionTolerance();
    if (FmPart* part = dynamic_cast<FmPart*>(link);
        part && !part->useGenericProperties.getValue())
    {
      // Find position of the FE node that is closest to the given point
      if (FFlNode* node = part->getNodeAtPoint(localPos,tolerance); node)
        loadPos = node->getPos();
      else
        return;
    }

    newTriad = link->getTriadAtPoint(loadPos,tolerance);
    if (!newTriad)
    {
      newTriad = new FmTriad(link->getLocalCS() * loadPos);
      newTriad->setParentAssembly(link->getParentAssembly());
      newTriad->connect(link);
      newTriad->draw();
    }
  }
  else
    newTriad = dynamic_cast<FmTriad*>(obj);

  if (newTriad)
  {
    this->setOwnerTriad(newTriad);

    if (oldTriad && !oldTriad->hasReferences())
      oldTriad->erase();

    this->updateDisplayTopology();
  }
  else if (oldTriad)
  {
    // No link is owning the triad. Change triad CS
    oldTriad->setTranslation(localPos);
    oldTriad->updateDisplayCS();
    oldTriad->updateChildrenDisplayTopology();
  }
}


void FmLoad::setLocalPoint(const FaVec3& pt, int idx)
{
  if (itsPoint[idx].setValue(pt))
    this->updateDisplayTopology();
}


FaVec3 FmLoad::getGlbPoint(int idx) const
{
  if (!itsPos[idx].isNull())
    return itsPos[idx]->getLocalCS() * itsPoint[idx].getValue();
  else
    return itsPoint[idx].getValue();
}


void FmLoad::setGlbPoint(const FaVec3& pt, int idx)
{
  if (!itsPos[idx].isNull())
    itsPoint[idx].setValue(itsPos[idx]->getLocalCS().inverse() * pt);
  else
    itsPoint[idx].setValue(pt);

  this->updateDisplayTopology();
}


void FmLoad::changeAttackPt(FmIsPositionedBase* p, const FaVec3& pt)
{
  if (editedLoad)
    editedLoad->moveAttackPoint(pt,true,p);
}


void FmLoad::changeFromPt(FmIsPositionedBase* p, const FaVec3& pt)
{
  if (editedLoad)
  {
    editedLoad->setFromRef(p);
    editedLoad->setGlobalFromPoint(pt);
  }
}


void FmLoad::changeToPt(FmIsPositionedBase* p, const FaVec3& pt)
{
  if (editedLoad)
  {
    editedLoad->setToRef(p);
    editedLoad->setGlobalToPoint(pt);
  }
}


bool FmLoad::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmLoad::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmLoad::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmLoad* copyObj = static_cast<FmLoad*>(obj);
  if (FmTriad* owner = copyObj->getOwnerTriad(); owner)
  {
    this->disconnect();
    this->connect(owner);
  }

  this->setFromRef(copyObj->getFromRef());
  this->setToRef(copyObj->getToRef());

  return true;
}


bool FmLoad::detach()
{
  FmTriad* oldTr = this->getOwnerTriad();
  if (!oldTr)
  {
    ListUI <<"Error: The load is already detached.\n";
    return false;
  }

  FmTriad* newTr = new FmTriad();
  newTr->setParentAssembly(oldTr->getParentAssembly());
  newTr->setGlobalCS(oldTr->getGlobalCS());
  newTr->connect();
  this->setOwnerTriad(newTr);
  newTr->draw();
  if (!oldTr->hasReferences())
    oldTr->erase();

  return true;
}


const char* FmLoad::getUITypeName() const
{
  return itsLoadType.getValue() == FORCE ? "Force" : "Torque";
}


/***********************************************************************
 *
 * Input and output from stream.
 *
 ***********************************************************************/

std::ostream& FmLoad::writeFMF(std::ostream& os)
{
  os <<"LOAD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmLoad::readAndConnect(std::istream& is, std::ostream&)
{
  FmLoad* obj = new FmLoad();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      // Conversion of old keywords
      if (strcmp(keyWord,"INIT_LOAD") == 0)
        FmLoad::parentParse("VALUE", activeStatement, obj);
      else
        FmLoad::parentParse(keyWord, activeStatement, obj);
    }
  }

  // If the FROM_OBJECT or the TO_OBJECT was the earth link (ID = -1),
  // the references to it have to be resolved manually here,
  // because the earth link is not member of the main link ring.

  for (int idx = 0; idx < 2; idx++)
    if (obj->itsPos[idx].getRefID() == -1)
      if (obj->itsPos[idx].getRefTypeID() == FmPart::getClassTypeID() ||
          obj->itsPos[idx].getRefTypeID() == FmLink::getClassTypeID())
        obj->itsPos[idx].setRef(FmDB::getEarthLink());

  obj->connect();
  return true;
}


int FmLoad::printSolverEntry(FILE* fp)
{
  FmTriad* owner = this->getOwnerTriad();
  if (!owner || owner->isSuppressed())
    return 0; // triad is suppressed, ignore load

  fprintf(fp,"&LOAD\n");
  this->printID(fp);

  if (this->getLoadType() == FORCE)
    fprintf(fp,"  type = 'force'\n");
  else
    fprintf(fp,"  type = 'moment'\n");

  fprintf(fp,"  triadId = %d\n", owner->getBaseID());

  // Beta feature: Update external forces based on previous configuration
  if (FFaString(this->getUserDescription()).hasSubString("#PrevStep"))
    fprintf(fp,"  updateFlag = %d\n", 1);

  // Making the load direction reference points
  FmIsPositionedBase* fromRef = this->getFromRef();
  FmIsPositionedBase* toRef   = this->getToRef();

  FaVec3 fromPoint = this->getLocalFromPoint();
  FaVec3 toPoint   = this->getLocalToPoint();

  // the FROM point
  int refLinkNo = 0, refTriadNo = 0;
  if (!fromRef)
    ListUI <<" ==> WARNING: No from-object specified for "<< this->getIdString()
           <<"\n     Assuming its from-point is referring to global axes.\n";
  else if (fromRef == FmDB::getEarthLink())
    fromPoint = fromRef->getLocalCS()*fromPoint;
  else if (fromRef->isOfType(FmLink::getClassTypeID()))
    refLinkNo = fromRef->getBaseID();
  else if (fromRef->isOfType(FmTriad::getClassTypeID()))
  {
    if (FmLink* fromLink = ((FmTriad*)fromRef)->getOwnerLink(); fromLink)
      refLinkNo = fromLink->getBaseID();
    else
      refTriadNo = fromRef->getBaseID();
  }
  fprintf(fp,"  vec1 = %17.9e %17.9e %17.9e",
          fromPoint[0],fromPoint[1],fromPoint[2]);
  if (refLinkNo > 0)
    fprintf(fp,", supEl1Id = %d",refLinkNo);
  else if (refTriadNo > 0)
    fprintf(fp,", triad1Id = %d",refTriadNo);

  // the TO point
  refLinkNo = refTriadNo = 0;
  if (!toRef)
    ListUI <<" ==> WARNING: No to-object specified for "<< this->getIdString()
           <<"\n     Assuming its to-point is referring to global axes.\n";
  else if (toRef == FmDB::getEarthLink())
    toPoint = toRef->getLocalCS()*toPoint;
  else if (toRef->isOfType(FmLink::getClassTypeID()))
    refLinkNo = toRef->getBaseID();
  else if (toRef->isOfType(FmTriad::getClassTypeID()))
  {
    if (FmLink* toLink = ((FmTriad*)toRef)->getOwnerLink(); toLink)
      refLinkNo = toLink->getBaseID();
    else
      refTriadNo = toRef->getBaseID();
  }
  fprintf(fp,"\n  vec2 = %17.9e %17.9e %17.9e",
          toPoint[0],toPoint[1],toPoint[2]);
  if (refLinkNo > 0)
    fprintf(fp,", supEl2Id = %d",refLinkNo);
  else if (refTriadNo > 0)
    fprintf(fp,", triad2Id = %d",refTriadNo);

  if (FmEngine* engine = this->getEngine(); engine)
    fprintf(fp,"\n  f1 = 1.0, loadEngineId = %d\n", engine->getBaseID());
  else // constant load
    fprintf(fp,"\n  f0 = %17.9e\n", this->getInitLoad());

  // Variables to be saved:
  // 1 - Global force vector
  // 2 - Signed force amplitude
  // 3 - Energies
  this->writeSaveVar(fp,3);

  fprintf(fp,"/\n\n");
  return 0;
}
