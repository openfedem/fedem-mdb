// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdAxialSprDa.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcAXIAL_SPRING, FmAxialSpring, FmSpringBase);


FmAxialSpring::FmAxialSpring()
{
  Fmd_CONSTRUCTOR_INIT(FmAxialSpring);

  FFA_REFERENCELIST_FIELD_INIT(itsTriadsField, itsTriads, "TRIAD_CONNECTIONS");
  itsTriads.setAutoSizing(false);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdAxialSprDa(this);
#endif
}


FmAxialSpring::~FmAxialSpring()
{
  this->disconnect();
}


void FmAxialSpring::getEntities(std::vector<FmSensorChoice>& choicesToFill, int)
{
  choicesToFill.clear();
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::LENGTH]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::DEFL]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::FORCE]);
}


double FmAxialSpring::getModelSpringLength() const
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  double length = 0.0;
  for (size_t i = 1; i < triads.size(); i++)
    if (triads[i] && triads[i-1])
      length += (triads[i-1]->getGlobalTranslation() - triads[i]->getGlobalTranslation()).length();

  return length;
}


bool FmAxialSpring::connect(FmTriad* tr1, FmTriad* tr2)
{
  bool status = this->mainConnect();

  this->setOwnerTriads(tr1, tr2);
  return status;
}


bool FmAxialSpring::detach()
{
  if (!detachTriad)
    return false;
  else if (!detachTriad->isAttached())
  {
    FFaMsg::list("Detach: The spring triad is already detached.\n");
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


FmTriad* FmAxialSpring::getFirstTriad() const
{
  return (itsTriads.size() < 1 ? static_cast<FmTriad*>(NULL) : itsTriads[0]);
}


FmTriad* FmAxialSpring::getSecondTriad() const
{
  return (itsTriads.size() < 2 ? static_cast<FmTriad*>(NULL) : itsTriads[1]);
}


void FmAxialSpring::getTriads(std::vector<FmTriad*>& toFill) const
{
  toFill.clear();
  toFill.reserve(2);
  toFill.push_back(itsTriads[0]);
  toFill.push_back(itsTriads[1]);

  // Beta feature: Pulley element with arbitrary number of triads (max 10)
  int extraTriads[8];
  int nTriads = FFaString(this->getUserDescription()).getIntsAfter("#addTriads",8,extraTriads);
  for (int j = 0; j < nTriads; j++)
    toFill.push_back(static_cast<FmTriad*>(FmDB::findObject(extraTriads[j])));
}


void FmAxialSpring::removeOwnerTriads()
{
  std::vector<FmTriad*> v(2,static_cast<FmTriad*>(NULL));
  itsTriads.setPtrs(v);
}


void FmAxialSpring::setOwnerTriads(FmTriad* tr1, FmTriad* tr2)
{
  std::vector<FmTriad*> v(2);
  v[0] = tr1; v[1] = tr2;
  itsTriads.setPtrs(v);
}


std::ostream& FmAxialSpring::writeFMF(std::ostream& os)
{
  os <<"AXIAL_SPRING\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmAxialSpring::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmAxialSpring::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmAxialSpring* copyObj = static_cast<FmAxialSpring*>(obj);
  this->setOwnerTriads(copyObj->getFirstTriad(),copyObj->getSecondTriad());
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->removeOwnerTriads();

  return true;
}


bool FmAxialSpring::readAndConnect(std::istream& is, std::ostream&)
{
  FmAxialSpring* obj = new FmAxialSpring();

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


void FmAxialSpring::initAfterResolve()
{
  this->FmSpringBase::initAfterResolve();

  this->setOwnerTriads(this->getFirstTriad(),this->getSecondTriad());
}


bool FmAxialSpring::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


int FmAxialSpring::checkAxialSprings()
{
  int errCount = 0;
  std::vector<FmAxialSpring*> allSprings;
  FmDB::getAllAxialSprings(allSprings);

  for (size_t i = 0; i < allSprings.size(); i++)
    if (!allSprings[i]->getFirstTriad() || !allSprings[i]->getSecondTriad())
    {
      errCount++;
      ListUI <<"ERROR: "<< allSprings[i]->getIdString()
	     <<" is not attached to any triads.\n";
    }

  return errCount;
}


int FmAxialSpring::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Axial spring\n");
  fprintf(fp,"&SPRING_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  springBaseId = %d\n", this->getBaseID());
  // Beta feature: Stiffness-proportional damping
  FFaString sDesc(this->getUserDescription());
  if (sDesc.hasSubString("#Rayleigh"))
    fprintf(fp,"  alpha2 = %f\n", sDesc.getDoubleAfter("#Rayleigh"));
  fprintf(fp,"  triadIDs =");
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  for (size_t i = 0; i < triads.size(); i++)
    fprintf(fp," %d", triads[i] ? triads[i]->getBaseID() : 0);
  fprintf(fp,"\n/\n\n");
  return 0;
}
