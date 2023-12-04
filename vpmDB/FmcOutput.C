// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmcOutput.H"
#include "vpmDB/FmCtrlLine.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdCtrlElement.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FccOUTPUT, FmcOutput, FmCtrlElementBase);

FmcOutput::FmcOutput()
{
  Fmd_CONSTRUCTOR_INIT(FmcOutput);

  itsPixmap = ctrlElemOut_xpm;
  itsInput = NULL;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmcOutput::getListViewPixmap() const
{
  FmEngine* engine = this->getEngine();
  if (engine && engine->myOutput.getValue())
    return sensor_xpm;

  return NULL;
}


FmEngine* FmcOutput::getEngine() const
{
  FmEngine* engine = NULL;
  FmSensorBase* sensor = NULL;
  if (this->hasReferringObjs(sensor,"itsMeasuredPt"))
    sensor->hasReferringObjs(engine,"mySensor");

  return engine;
}


void FmcOutput::initAfterResolve()
{
  this->FmCtrlElementBase::initAfterResolve();

  FmSensorBase* sensor = this->getSimpleSensor(true);

  std::vector<FmEngine*> engines;
  sensor->getEngines(engines);
  if (engines.empty())
  {
    ListUI <<"  -> "<< this->getIdString(true)
           <<" lacks associated Function object, creating one.\n";
    FmEngine* engine = new FmEngine();
    engine->setSensor(sensor);
    engine->connect();
  }
  else if (engines.size() > 1) // There should only be one
    ListUI <<"  -> "<< this->getIdString(true)
           <<" is connected to "<< engines.size() <<" Function objects.\n"
           <<" Only the first one will be used, please check your model.\n";
}


bool FmcOutput::interactiveErase()
{
  // Delete the associated sensor, and the engine using it as well
  FmSensorBase* sensor = NULL;
  if (this->hasReferringObjs(sensor,"itsMeasuredPt"))
  {
    std::vector<FmEngine*> engines;
    sensor->getEngines(engines);
    for (FmEngine* engine : engines)
    {
      // Decouple the sensor before erasing the engine,
      // to avoid erasing the sensor twice (Bugfix #371)
      if (engine->getSensor() == sensor)
        engine->setSensor(NULL);
      engine->erase();
    }
    sensor->erase();
  }

  return this->erase();
}


FmCtrlElementBase* FmcOutput::copy()
{
  FmcOutput* newObj = new FmcOutput();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


std::ostream& FmcOutput::writeFMF(std::ostream& os)
{
  os <<"CONTROL_OUTPUT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmcOutput::readAndConnect(std::istream& is, std::ostream&)
{
  FmcOutput* obj = new FmcOutput();

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


bool FmcOutput::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcOutput::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmcOutput::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmcOutput* copyObj = static_cast<FmcOutput*>(obj);
  FmCtrlLine* cpLine = copyObj->getLine();
  copyObj->setLine(1,NULL);
  this->setLine(1,cpLine);
  return true;
}


FmCtrlLine* FmcOutput::getLine(int portNo) const
{
  return portNo == 1 ? itsInput : NULL;
}


bool FmcOutput::setLine(int portNo, FmCtrlLine* line)
{
  if (portNo != 1)
    return false;

  itsInput = line;
  line->setEndElement(this);
  return true;
}


bool FmcOutput::releaseFromPort(FmCtrlLine* line)
{
  if (line != itsInput)
    return false;

  line->setEndElement(NULL);
  itsInput = NULL;
  return true;
}


int FmcOutput::atWhatPort(const FmCtrlLine* line) const
{
  return line == itsInput ? 1 : -1;
}
