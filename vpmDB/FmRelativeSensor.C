// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRelativeSensor.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdSensor.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcRELATIVE_SENSOR, FmRelativeSensor, FmSensorBase);


FmRelativeSensor::FmRelativeSensor()
{
  Fmd_CONSTRUCTOR_INIT(FmRelativeSensor);

  FFA_REFERENCELIST_FIELD_INIT(itsMeasureField, itsMeasure, "MEASURED");
  itsMeasure.setAutoSizing(false);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdSensor(this);
#endif
}


FmRelativeSensor::~FmRelativeSensor()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

std::string FmRelativeSensor::getInfoString() const
{
  if (!this->isConnected())
    return this->FmSensorBase::getInfoString();

  return itsMeasure[1]->getInfoString() + " relative to "
    +    itsMeasure[0]->getInfoString();
}


bool FmRelativeSensor::isConnected() const
{
  if (itsMeasure.size() < 2) return false;

  if (itsMeasure[0].getPointer() && itsMeasure[1].getPointer())
    if (itsMeasure[0]->isOfType(FmTriad::getClassTypeID()))
      if (itsMeasure[1]->isOfType(FmTriad::getClassTypeID()))
	return true;

  return false;
}


void FmRelativeSensor::getSensorEntities(std::vector<FmSensorChoice>& choices, int)
{
  choices.clear();

  if (this->isConnected())
  {
    choices.reserve(3);
    choices.push_back(itsEntityTable[FmIsMeasuredBase::DISTANCE]);
    choices.push_back(itsEntityTable[FmIsMeasuredBase::VEL]);
    choices.push_back(itsEntityTable[FmIsMeasuredBase::ACCEL]);
  }
}


void FmRelativeSensor::getSensorDofs(std::vector<FmSensorChoice>& choices)
{
  choices.clear();

  if (this->isConnected())
  {
    choices.reserve(7);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_X]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_Y]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_Z]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_RX]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_RY]);
    choices.push_back(itsDofTable[FmIsMeasuredBase::REL_RZ]);
  }
}


bool FmRelativeSensor::connect(FmIsMeasuredBase* mb1, FmIsMeasuredBase* mb2)
{
  bool status = this->mainConnect();
  this->setMeasured(mb1,mb2);
  return status;
}


FmIsMeasuredBase* FmRelativeSensor::getMeasured(int ind) const
{
  if (ind < 1 || (size_t)ind > itsMeasure.size()) return NULL;

  return itsMeasure[ind-1].getPointer();
}


void FmRelativeSensor::getMeasured(std::vector<FmIsMeasuredBase*>& measured) const
{
  itsMeasure.getPtrs(measured);
}


void FmRelativeSensor::removeMeasured()
{
  itsMeasure.setPtrs({NULL,NULL});
}


void FmRelativeSensor::setMeasured(FmIsMeasuredBase* m1, FmIsMeasuredBase* m2)
{
  itsMeasure.setPtrs({m1,m2});
}


bool FmRelativeSensor::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmRelativeSensor::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmRelativeSensor* copyObj = static_cast<FmRelativeSensor*>(obj);

  std::vector<FmIsMeasuredBase*> cplMes;
  copyObj->getMeasured(cplMes);
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->removeMeasured();
  itsMeasure.setPtrs(cplMes);

  return true;
}


std::ostream& FmRelativeSensor::writeFMF(std::ostream& os)
{
  os <<"RELATIVE_SENSOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRelativeSensor::readAndConnect(std::istream& is, std::ostream&)
{
  FmRelativeSensor* obj = new FmRelativeSensor();

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


bool FmRelativeSensor::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


void FmRelativeSensor::initAfterResolve()
{
  this->FmSensorBase::initAfterResolve();

  this->setMeasured(itsMeasure[0].getPointer(),itsMeasure[1].getPointer());
}


int FmRelativeSensor::printSolverData(FILE* fp, FmEngine* engine, int iarg) const
{
  int err = 0;
  fprintf(fp,"  type = 'RELATIVE_TRIAD'\n");
  FmIsMeasuredBase* triad = NULL;
  for (int indx = 1; indx <= 2; indx++)
    if (!(triad = this->getMeasured(indx)))
      return indx;
    else if (triad->isOfType(FmTriad::getClassTypeID()))
      fprintf(fp,"  triad%dId  = %d\n", indx, triad->getBaseID());
    else
    {
      ++err;
      ListUI <<" --> Error: Invalid object type ("<< triad->getUITypeName()
             <<") for "<< this->getIdString(true)
             <<", only Triad is allowed.\n";
    }

  int dof = engine->getDof(iarg) - FmIsMeasuredBase::REL;
  int ent = engine->getEntity(iarg);
  // Beta feature: Sensor measuring rotation in terms of Rodrigues
  if (ent == FmIsMeasuredBase::DISTANCE && dof >= 4 && dof <= 6 &&
      this->getUserDescription().find("#Rodrig") != std::string::npos)
    dof += 3;
  fprintf(fp,"  dof       = %d\n", dof);

  switch (ent)
    {
    case FmIsMeasuredBase::DISTANCE:
      fprintf(fp,"  dofEntity = 'REL_POS'\n");
      break;
    case FmIsMeasuredBase::VEL:
      fprintf(fp,"  dofEntity = 'VEL'\n");
      break;
    case FmIsMeasuredBase::ACCEL:
      fprintf(fp,"  dofEntity = 'ACC'\n");
      break;
    default:
      ListUI <<" --> Error: Invalid entity "<< ent
             <<" for "<< this->getIdString(true) <<"\n";
      ++err;
    }

  fprintf(fp,"  dofSystem = 'GLOBAL'\n");
  return err;
}
