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

  if (parent)
    if (parent->isOfType(FmTriad::getClassTypeID()))
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


void FmLoad::moveAttackPointGlobal(const FaVec3& pos, FmLink* link)
{
  FmTriad* ownerTriad = this->getOwnerTriad();
  FmLink*  attackLink = !link && ownerTriad ? ownerTriad->getOwnerLink() : link;

  // convert point from global to local coordinates for the chosen link
  if (attackLink)
    this->moveAttackPoint(attackLink->getLocalCS().inverse()*pos, attackLink);
  else
    this->moveAttackPoint(pos, attackLink);
}


void FmLoad::moveAttackPointLocal(const FaVec3& pos, FmLink* link)
{
  FmTriad* ownerTriad = this->getOwnerTriad();
  FmLink*  attackLink = !link && ownerTriad ? ownerTriad->getOwnerLink() : link;

  this->moveAttackPoint(pos, attackLink);
}


void FmLoad::moveAttackPoint(const FaVec3& localPos, FmLink* link)
{
  FmTriad* oldTriad = this->getOwnerTriad();
  if (link)
  {
    FaVec3 loadPos = localPos;
    double tolerance = FmDB::getPositionTolerance();
    FmPart* part = dynamic_cast<FmPart*>(link);
    if (part && !part->useGenericProperties.getValue())
    {
      // find position of the FE node that is closest to the given point
      FFlNode* tmpNode = part->getNodeAtPoint(localPos,tolerance);
      if (tmpNode)
	loadPos = tmpNode->getPos();
      else
	return;
    }

    FmTriad* newTriad = link->getTriadAtPoint(loadPos,tolerance);
    if (!newTriad)
    {
      newTriad = new FmTriad(link->getLocalCS() * loadPos);
      newTriad->setParentAssembly(link->getParentAssembly());
      newTriad->connect(link);
      newTriad->draw();
    }
    this->setOwnerTriad(newTriad);

    if (!oldTriad->hasReferences())
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


void FmLoad::changeAttackPt(FmLink* link, const FaVec3& pt)
{
  if (editedLoad)
    editedLoad->moveAttackPointGlobal(pt,link);
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
  FmTriad* owner = copyObj->getOwnerTriad();
  if (owner)
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
  if (itsLoadType.getValue() == FORCE)
    return "Force";
  else
    return "Torque";
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


static bool localParse(const char* keyWord, std::istream& is, FmLoad* obj)
{
  // Conversion of old keywords
  if (strcmp(keyWord,"INIT_LOAD") == 0)
    return FmLoad::parentParse("VALUE",is,obj);

  return FmLoad::parentParse(keyWord,is,obj);
}

bool FmLoad::readAndConnect(std::istream& is, std::ostream&)
{
  FmLoad* obj = new FmLoad();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      ::localParse(keyWord, activeStatement, obj);
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
  int fromRefLinkNo = 0;
  int toRefLinkNo   = 0;

  FmIsPositionedBase* fromRef = this->getFromRef();
  FmIsPositionedBase* toRef   = this->getToRef();

  FaVec3 fromPoint = this->getLocalFromPoint();
  FaVec3 toPoint   = this->getLocalToPoint();

  // the FROM point
  if (!fromRef)
    ListUI <<" ==> WARNING: No from-object is specified for "<< this->getIdString(true)
	   <<"\n     Assuming its from-point is referring to the global system.\n";
  else if (fromRef == FmDB::getEarthLink())
    fromPoint = fromRef->getLocalCS()*fromPoint;
  else if (fromRef->isOfType(FmLink::getClassTypeID()))
    fromRefLinkNo = fromRef->getBaseID();
  else if (fromRef->isOfType(FmTriad::getClassTypeID()))
  {
    FmLink* fromRefLink = ((FmTriad*)fromRef)->getOwnerLink();
    if (fromRefLink) fromRefLinkNo = fromRefLink->getBaseID();
  }

  fprintf(fp,"  vec1 = %17.9e %17.9e %17.9e, supEl1Id = %d\n",
	  fromPoint[0],fromPoint[1],fromPoint[2],fromRefLinkNo);

  // the TO point
  if (!toRef)
    ListUI <<" ==> WARNING: No to-object is specified for "<< this->getIdString(true)
	   <<"\n     Assuming its to-point is referring to the global system.\n";
  else if (toRef == FmDB::getEarthLink())
    toPoint = toRef->getLocalCS()*toPoint;
  else if (toRef->isOfType(FmLink::getClassTypeID()))
    toRefLinkNo = toRef->getBaseID();
  else if (toRef->isOfType(FmTriad::getClassTypeID()))
  {
    FmLink* toRefLink = ((FmTriad*)toRef)->getOwnerLink();
    if (toRefLink) toRefLinkNo = toRefLink->getBaseID();
  }

  fprintf(fp,"  vec2 = %17.9e %17.9e %17.9e, supEl2Id = %d\n",
	  toPoint[0],toPoint[1],toPoint[2],toRefLinkNo);

  if (this->getEngine())
    fprintf(fp,"  f1 = 1.0, loadEngineId = %d\n", this->getEngine()->getBaseID());
  else // constant load
    fprintf(fp,"  f0 = %17.9e\n", this->getInitLoad());

  // Variables to be saved:
  // 1 - Global force vector
  // 2 - Signed force amplitude
  // 3 - Energies
  this->writeSaveVar(fp,3);

  fprintf(fp,"/\n\n");
  return 0;
}
