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
  switch (this->isConnected()) {
  case 2:
    return itsMeasure[1]->getInfoString() + " relative to "
      +    itsMeasure[0]->getInfoString();
  case 4:
    return "Angle between Triads " + itsMeasure[0]->getIdPath()
      + "-" + itsMeasure[2]->getIdPath()
      + " and Triads " + itsMeasure[1]->getIdPath()
      + "-" + itsMeasure[3]->getIdPath();
  default:
    return this->FmSensorBase::getInfoString();
  }
}


int FmRelativeSensor::isConnected() const
{
  int connected = 0;
  for (size_t i = 0; i < itsMeasure.size(); i++)
    if (itsMeasure[i].getPointer())
      if (itsMeasure[i]->isOfType(FmTriad::getClassTypeID()))
        connected++;

  return connected == 2 || connected == 4 ? connected : 0;
}


void FmRelativeSensor::getSensorEntities(FmSensorChoices& choices, int)
{
  switch (this->isConnected()) {
  case 2:
    choices = {
      itsEntityTable[FmIsMeasuredBase::DISTANCE],
      itsEntityTable[FmIsMeasuredBase::VEL],
      itsEntityTable[FmIsMeasuredBase::ACCEL]
    };
    break;
  case 4:
    choices = { itsEntityTable[FmIsMeasuredBase::ANGLE] };
    break;
  default:
    choices.clear();
  }
}


void FmRelativeSensor::getSensorDofs(FmSensorChoices& choices)
{
  switch (this->isConnected()) {
  case 2:
    choices = {
      itsDofTable[FmIsMeasuredBase::REL],
      itsDofTable[FmIsMeasuredBase::REL_X],
      itsDofTable[FmIsMeasuredBase::REL_Y],
      itsDofTable[FmIsMeasuredBase::REL_Z],
      itsDofTable[FmIsMeasuredBase::REL_RX],
      itsDofTable[FmIsMeasuredBase::REL_RY],
      itsDofTable[FmIsMeasuredBase::REL_RZ]
    };
    break;
  case 4:
    choices = {
      itsDofTable[FmIsMeasuredBase::UNSIGNED],
      itsDofTable[FmIsMeasuredBase::ANGLE_YZ],
      itsDofTable[FmIsMeasuredBase::ANGLE_ZX],
      itsDofTable[FmIsMeasuredBase::ANGLE_XY]
    };
    break;
  default:
    choices.clear();
  }
}


bool FmRelativeSensor::connect(FmIsMeasuredBase* mb1, FmIsMeasuredBase* mb2)
{
  bool status = this->mainConnect();
  itsMeasure.setPtrs({mb1,mb2});
  return status;
}


bool FmRelativeSensor::connect(FmIsMeasuredBase* mb1, FmIsMeasuredBase* mb2,
                               FmIsMeasuredBase* mb3, FmIsMeasuredBase* mb4)
{
  bool status = this->mainConnect();
  itsMeasure.setPtrs({mb1,mb2,mb3,mb4});
  return status;
}


FmIsMeasuredBase* FmRelativeSensor::getMeasured(int ind) const
{
  if (ind < 1 || ind > static_cast<int>(itsMeasure.size())) return NULL;

  return itsMeasure[ind-1].getPointer();
}


void FmRelativeSensor::getMeasured(std::vector<FmIsMeasuredBase*>& objs) const
{
  itsMeasure.getPtrs(objs);
}


void FmRelativeSensor::removeMeasured()
{
  size_t nRef = itsMeasure.size();
  itsMeasure.clear();
  for (size_t i = 0; i < nRef; i++)
    itsMeasure.push_back(NULL);
}


bool FmRelativeSensor::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmRelativeSensor::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmRelativeSensor* copyObj = static_cast<FmRelativeSensor*>(obj);

  std::vector<FmIsMeasuredBase*> objs;
  copyObj->getMeasured(objs);
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->removeMeasured();
  itsMeasure.setPtrs(objs);

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
      FmRelativeSensor::parentParse(keyWord, activeStatement, obj);
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

  std::vector<FmIsMeasuredBase*> objs;
  itsMeasure.getPtrs(objs,true);
  itsMeasure.setPtrs(objs);
}


int FmRelativeSensor::printSolverData(FILE* fp, FmEngine* engine, int arg) const
{
  int err = 0;
  fprintf(fp,"  type = 'RELATIVE_TRIAD'\n  triadId   =");
  for (size_t indx = 1; indx <= itsMeasure.size(); indx++)
    if (FmIsMeasuredBase* triad = this->getMeasured(indx); !triad)
      return indx;
    else if (triad->isOfType(FmTriad::getClassTypeID()))
      fprintf(fp," %d", triad->getBaseID());
    else
    {
      ++err;
      ListUI <<" --> Error: Invalid object type ("<< triad->getUITypeName()
             <<") for "<< this->getIdString(true)
             <<", only Triad is allowed.\n";
    }
  fprintf(fp,"\n");

  int entity  = engine->getEntity(arg);
  if (int dof = engine->getDof(arg) - FmIsMeasuredBase::REL; dof > 0)
  {
    // Beta feature: Sensor measuring rotation in terms of Rodrigues
    if (entity == FmIsMeasuredBase::DISTANCE && dof >= 4 && dof <= 6 &&
        this->getUserDescription().find("#Rodrig") != std::string::npos)
      dof += 3;
    else if (entity == FmIsMeasuredBase::ANGLE && dof >= 14)
      dof -= 4;
    fprintf(fp,"  dof       = %d\n", dof);
  }

  int lerr = err;
  if (itsMeasure.size() == 4)
    switch (entity) {
    case FmIsMeasuredBase::ANGLE:
      fprintf(fp,"  dofEntity = 'ANGLE'\n");
      break;
    default:
      ++err;
    }
  else
    switch (entity) {
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
      ++err;
    }

  if (err > lerr)
    ListUI <<" --> Error: Invalid entity "<< entity
           <<" for "<< this->getIdString(true) <<"\n";

  fprintf(fp,"  dofSystem = 'GLOBAL'\n");
  return err;
}
