// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcSENSOR_BASE, FmSensorBase, FmIsRenderedBase);


FmSensorBase::FmSensorBase()
{
  Fmd_CONSTRUCTOR_INIT(FmSensorBase);
}


FmSensorBase::~FmSensorBase()
{
  // Do not try to set the time sensor as sensor if we are deleting it
  FmSensorBase* timeSens = FmDB::getTimeSensor(false);
  if (timeSens == this) timeSens = NULL;

  std::vector<FmEngine*> engines;
  this->getEngines(engines);
  for (size_t i = 0; i < engines.size(); i++)
  {
    size_t nArgs = engines[i]->getNoArgs();
    FmSensorBase* sensor = engines[i]->getSensor();
    for (size_t j = 0; j < nArgs; sensor = engines[i]->getSensor(++j))
      if (sensor == this)
	engines[i]->setSensor(timeSens,j);
  }
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmSensorBase::getEngines(std::vector<FmEngine*>& toFill) const
{
  toFill.clear();
  this->getReferringObjs(toFill,"mySensor");
}


bool FmSensorBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmSensorBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_REPLACE)
    return true;

  FmSensorBase* copyObj = static_cast<FmSensorBase*>(obj);

  std::vector<FmEngine*> engines;
  copyObj->getEngines(engines);
  for (size_t i = 0; i < engines.size(); i++)
  {
    size_t nArg = engines[i]->getNoArgs();
    FmSensorBase* sensor = engines[i]->getSensor();
    for (size_t j = 0; j < nArg; sensor = engines[i]->getSensor(++j))
      if (sensor == copyObj)
        engines[i]->setSensor(this,j);
  }

  return true;
}


bool FmSensorBase::localParse(const char* keyWord, std::istream& activeStatement,
			      FmSensorBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}
