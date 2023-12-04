// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdAxialSprDa.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcAXIAL_DAMPER, FmAxialDamper, FmDamperBase);


FmAxialDamper::FmAxialDamper()
{
  Fmd_CONSTRUCTOR_INIT(FmAxialDamper);

  FFA_REFERENCELIST_FIELD_INIT(itsTriadsField, itsTriads, "TRIAD_CONNECTIONS");
  itsTriads.setAutoSizing(false);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdAxialSprDa(this);
#endif
}


FmAxialDamper::~FmAxialDamper()
{
  this->disconnect();
}


void FmAxialDamper::getEntities(std::vector<FmSensorChoice>& choicesToFill, int)
{
  choicesToFill.clear();
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::LENGTH]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::VEL]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::FORCE]);
}


bool FmAxialDamper::connect(FmTriad* tr1, FmTriad* tr2)
{
  bool status = this->mainConnect();

  this->setOwnerTriads(tr1, tr2);
  return status;
}


bool FmAxialDamper::detach()
{
  if (!detachTriad)
    return false;
  else if (!detachTriad->isAttached())
  {
    FFaMsg::list("Detach: The damper triad is already detached.\n");
    return false;
  }

  // Check which triad the user wants to remove
  FmTriad* triad1 = this->getFirstTriad();
  FmTriad* triad2 = this->getSecondTriad();
  if (detachTriad == triad1)
    triad1 = NULL;
  else if (detachTriad == triad2)
    triad2 = NULL;
  else
    return false;

  // Make a new triad for the detach
  FmTriad* newTriad = new FmTriad();
  newTriad->setParentAssembly(this->getParentAssembly());
  newTriad->connect();
  newTriad->setGlobalCS(detachTriad->getGlobalCS());
  if (!triad1)
    this->setOwnerTriads(newTriad,triad2);
  else
    this->setOwnerTriads(triad1,newTriad);
  newTriad->draw();

  if (!detachTriad->hasReferences())
  {
    detachTriad->erase();
    detachTriad = NULL;
  }

  return true;
}


FmTriad* FmAxialDamper::getFirstTriad() const
{
  return (itsTriads.size() < 1 ? static_cast<FmTriad*>(NULL) : itsTriads[0]);
}


FmTriad* FmAxialDamper::getSecondTriad() const
{
  return (itsTriads.size() < 2 ? static_cast<FmTriad*>(NULL) : itsTriads[1]);
}


void FmAxialDamper::removeOwnerTriads()
{
  itsTriads.setPtrs({NULL, NULL});
}


void FmAxialDamper::setOwnerTriads(FmTriad* tr1, FmTriad* tr2)
{
  itsTriads.setPtrs({tr1, tr2});
}


std::ostream& FmAxialDamper::writeFMF(std::ostream& os)
{
  os <<"AXIAL_DAMPER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmAxialDamper::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmAxialDamper::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmAxialDamper* copyObj = static_cast<FmAxialDamper*>(obj);
  this->setOwnerTriads(copyObj->getFirstTriad(),copyObj->getSecondTriad());
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->removeOwnerTriads();

  return true;
}


bool FmAxialDamper::readAndConnect(std::istream& is, std::ostream&)
{
  FmAxialDamper* obj = new FmAxialDamper();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


void FmAxialDamper::initAfterResolve()
{
  this->FmDamperBase::initAfterResolve();

  this->setOwnerTriads(this->getFirstTriad(),this->getSecondTriad());
}


bool FmAxialDamper::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


int FmAxialDamper::checkAxialDampers()
{
  int errCount = 0;
  std::vector<FmAxialDamper*> allDampers;
  FmDB::getAllAxialDampers(allDampers);
  for (FmAxialDamper* damper : allDampers)
    if (!damper->getFirstTriad() || !damper->getSecondTriad())
    {
      errCount++;
      ListUI <<"ERROR: "<< damper->getIdString(true)
             <<" is not attached to any triads.\n";
    }

  return errCount;
}


int FmAxialDamper::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Axial damper\n");
  fprintf(fp,"&DAMPER\n");
  this->printID(fp);
  for (int i = 0; i < 2; i++)
    fprintf(fp,"  triad%dId = %d\n", i+1, itsTriads[i]->getBaseID());
  return this->FmDamperBase::printSolverEntry(fp);
}
