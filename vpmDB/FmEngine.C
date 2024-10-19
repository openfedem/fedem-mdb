// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmEngine.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmIsControlledBase.H"
#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmcInput.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmTimeSensor.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FmfWaveSinus.H"
#include "vpmDB/FmfMultiArgBase.H"
#include "vpmDB/FmfExternalFunction.H"
#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/external.xpm"
#include "vpmDB/Icons/sensor.xpm"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

std::set<int> FmEngine::betaFeatureEngines;

/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcENGINE, FmEngine, FmIsPlottedBase);


FmEngine::FmEngine(bool useTimeSensor)
{
  Fmd_CONSTRUCTOR_INIT(FmEngine);

  FFA_REFERENCE_FIELD_INIT(myFunctionField, myFunction, "MATH_FUNC");
  FFA_REFERENCE_FIELD_INIT(myFunctionOwnerField, myFunctionOwner, "ENGINE_TO_USE_FUNCTION_FROM");
  myFunctionOwner.setPrintIfZero(false);

  FFA_REFERENCELIST_FIELD_INIT(mySensorField, mySensor, "SENSOR");

  FFA_FIELD_DEFAULT_INIT(myEntityNames, "ENTITY_NAME");
  FFA_FIELD_DEFAULT_INIT(myEntities, "ENTITY");
  FFA_FIELD_DEFAULT_INIT(myDofs, "DOF");

  FFA_FIELD_INIT(myOutput, false, "OUTPUT_SENSOR");
  FFA_FIELD_DEFAULT_INIT(myThreshold, "DTS_THRESHOLD");

  if (useTimeSensor)
    this->setSensor(FmDB::getTimeSensor());
}


FmEngine::~FmEngine()
{
  std::vector<FmIsControlledBase*> allControlled;
  this->getReferringObjs(allControlled,"myEngine");
  for (FmIsControlledBase* obj : allControlled)
    obj->setEngine(NULL);

  // Remove function if not used by others
  std::vector<FmEngine*> engines;
  if (!myFunction.isNull())
  {
    myFunction->getEngines(engines);
    if (engines.size() == 1 && engines.front() == this)
      myFunction->erase();
  }

  // Clean up if this is used to link a function from
  engines.clear();
  this->getReferringObjs(engines,"myFunctionOwner");

  // Set first object as new parent
  if (!engines.empty())
    engines.front()->setEngineToLinkFunctionFrom(NULL);

  // Set the others to point at it
  for (size_t i = 1; i < engines.size(); i++)
    engines[i]->setEngineToLinkFunctionFrom(engines.front());

  // Remove all non-listable sensors only used by this engine
  std::vector<FmSensorBase*> sensors;
  mySensor.getPtrs(sensors);
  for (FmSensorBase* sensor : sensors)
  {
    engines.clear();
    sensor->getEngines(engines);
    if (engines.size() == 1 && engines.front() == this)
      if (!sensor->isListable() && !sensor->isTime())
        sensor->erase();
  }

  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmEngine::getListViewPixmap() const
{
  if (this->isExternalFunc())
    return external_xpm;
  else if (myOutput.getValue())
    return sensor_xpm;

  return NULL;
}


std::string FmEngine::getInfoString() const
{
  if (this->isDriveFile())
    return myFunction->getInfoString();

  if (this->isControlOutEngine())
    return this->getSensor()->getMeasured()->getInfoString();

  return this->FmBase::getInfoString();
}


bool FmEngine::isActive() const
{
  if (myOutput.getValue())
    return true;

  if (betaFeatureEngines.find(this->getBaseID()) != betaFeatureEngines.end())
    return true;

  if (FmDB::getActiveAnalysis()->solverAddOpts.getValue().find("-allEngineVars") != std::string::npos)
    return true;

  std::vector<FmModelMemberBase*> users;
  if (!this->getUsers(users,true))
    return false;

#ifdef FM_DEBUG
  std::cout <<"\nHere are the users of "<< this->getIdString(true) <<":";
  for (FmModelMemberBase* obj : users) std::cout <<"\n\t"<< obj->getIdString(true);
  std::cout << std::endl;
#endif
  return true;
}


bool FmEngine::isControlOutEngine() const
{
  FmSensorBase* sensor = mySensor.getFirstPtr();
  if (sensor && mySensor.size() == 1)
    return sensor->isControlOutput();

  return false;
}


bool FmEngine::isControlInEngine() const
{
  std::vector<FmModelMemberBase*> objs;
  this->getReferringObjs(objs);

  for (FmModelMemberBase* obj : objs)
    if (!obj->isOfType(FmcInput::getClassTypeID()))
      return false;

  return !objs.empty();
}


bool FmEngine::isDriveFile() const
{
  if (this->isFunctionLinked()) return false;
  if (myFunction.isNull()) return false;

  return (myFunction->getFunctionUse() == FmMathFuncBase::DRIVE_FILE);
}


bool FmEngine::isExternalFunc() const
{
  if (myFunction.isNull()) return false;

  return myFunction->isOfType(FmfExternalFunction::getClassTypeID());
}


bool FmEngine::isListable() const
{
  if (this->isControlOutEngine()) return false;
  if (this->isControlInEngine()) return false;
  if (this->isDriveFile()) return false;

  return true;
}


void FmEngine::setEngineToLinkFunctionFrom(FmEngine* engine)
{
  myFunctionOwner.setRef(engine);

  if (engine == this)
    this->setFunction(NULL);
  else if (engine)
    this->setFunction(engine->getFunction());
}


FmBase* FmEngine::duplicate() const
{
  FmVesselMotion* vm = NULL;
  if (this->hasReferringObjs(vm,"motionEngine"))
    return NULL; // Not allowed to duplicate RAO motion engines directly

  FmEngine* newEng = static_cast<FmEngine*>(this->copy(FmBase::SHALLOW));
  if (!myFunction.isNull())
    newEng->setFunction(static_cast<FmMathFuncBase*>(myFunction->duplicate()));
  newEng->connect();

  return newEng;
}


bool FmEngine::disconnect()
{
  myFunction = NULL;
  mySensor.clear();
  return this->mainDisconnect();
}


/*!
  Only the currently active users are returned by this method.
  Any sensors measuring this engine are bypassed, and the engines using those
  sensors as argument are returned instead, unless \a recursive is \e true.
  In the latter case, this method is invoked recursively to find the user(s)
  of the engine using this engine instead.
*/

bool FmEngine::getUsers(std::vector<FmModelMemberBase*>& toFill,
                        bool recursive) const
{
  size_t n = toFill.size();
  std::vector<FmModelMemberBase*> engines;

  this->getReferringObjs(toFill);
  for (size_t i = n; i < toFill.size();)
    if (toFill[i]->isOfType(FmEngine::getClassTypeID()))
    {
      // Replace engine by the control input(s) using it, if any
      std::vector<FmcInput*> inputs;
      toFill[i]->getReferringObjs(inputs,"myEngine");
      for (FmcInput* obj : inputs) toFill.push_back(obj);
      // Remove other engines referring directly to this engine.
      // It is only sharing the math function with this one and
      // should not be regarded as an engine user (TT #2899).
      toFill.erase(toFill.begin()+i);
    }
    else if (toFill[i]->isOfType(FmIsControlledBase::getClassTypeID()))
    {
      // Replace by the active owner, if any
      FmModelMemberBase* owner = static_cast<FmIsControlledBase*>(toFill[i])->getActiveOwner();
      if (owner)
        toFill[i++] = owner;
      else
        toFill.erase(toFill.begin()+i);
    }
    else if (toFill[i]->isOfType(FmSensorBase::getClassTypeID()))
    {
      toFill[i]->getReferringObjs(engines,"mySensor");
      toFill.erase(toFill.begin()+i);
    }
    else
      i++;

  if (!recursive)
    toFill.insert(toFill.end(),engines.begin(),engines.end());
  else for (FmModelMemberBase* engine : engines)
  {
    FmEngine* e = static_cast<FmEngine*>(engine);
    if (!e->getUsers(toFill,true))
      // This engine has no direct users, but add the engine itself instead if
      // it is an output sensor or specified through a beta feature command
      if (e->myOutput.getValue() ||
          betaFeatureEngines.find(e->getBaseID()) != betaFeatureEngines.end())
        toFill.push_back(engine);
  }

  return toFill.size() > n;
}


FmSensorBase* FmEngine::getUniqueSensor() const
{
  FmSensorBase* sensor = NULL;
  size_t first = 0, nArg = this->getNoArgs();
  while (first < nArg)
    if ((sensor = this->getSensor(first)))
      break;
    else
      ++first;

  for (size_t i = first; i < nArg; i++)
  {
    FmSensorBase* s = this->getSensor(i);
    if (!s) continue;

    FmEngine* e = dynamic_cast<FmEngine*>(s->getMeasured());
    if (e)
    {
      FmSensorBase* us = e->getUniqueSensor();
      if (us != sensor)
      {
	if (i == first && us)
	  sensor = us;
	else
	  return NULL;
      }
    }
    else if (s != sensor)
      return NULL;
  }

  return sensor;
}


bool FmEngine::initGetValue() const
{
  bool retVal = true;
  FmfMultiArgBase* f = dynamic_cast<FmfMultiArgBase*>(myFunction.getPointer());
  if (f)
    retVal = f->initGetValueNoRecursion();
  else if (!myFunction.isNull())
    retVal = myFunction->initGetValue();

  size_t nArg = this->getNoArgs();
  if (nArg == 1 || !retVal) return retVal;

  for (size_t i = 0; i < nArg; i++)
  {
    FmSensorBase* s = this->getSensor(i);
    if (!s) continue;

    FmEngine* e = dynamic_cast<FmEngine*>(s->getMeasured());
    if (e && !e->initGetValue())
      return false;
  }

  return true;
}


bool FmEngine::getValue(double x, double& y) const
{
  int ierr = 0;
  size_t nArg = this->getNoArgs();
  if (nArg == 1)
  {
    y = myFunction.isNull() ? x : myFunction->getValue(x,ierr);
    return ierr == 0;
  }

  DoubleVec args(nArg,0.0);
  args.front() = x;
  for (size_t i = 0; i < nArg; i++)
  {
    FmSensorBase* s = this->getSensor(i);
    FmEngine* e = s ? dynamic_cast<FmEngine*>(s->getMeasured()) : NULL;
    if (e && !e->getValue(x,args[i]))
      return false;
  }

  y = myFunction->getValue(args,ierr);
  return ierr == 0;
}


size_t FmEngine::getNoArgs() const
{
  return myFunction.isNull() ? 1 : myFunction->getNoArgs();
}


FmSensorBase* FmEngine::getSensor(size_t i) const
{
  return mySensor.getPtr(i);
}


void FmEngine::setSensor(FmSensorBase* sensor, int argIdx)
{
  unsigned int i = 0;
  if (argIdx < 0) // skip argument out-of-range check
    i = -argIdx;
  else if ((i = argIdx))
    if (myFunction.isNull() || i >= myFunction->getNoArgs())
      return;

  if (sensor || i < mySensor.size())
    mySensor.setPtr(sensor,i);
  if (!sensor) return;

  // Set an updated dof and entity, that is valid
  if (this->getDof(i) == -1) {
    std::vector<FmSensorChoice> dofChoices;
    sensor->getSensorDofs(dofChoices);
    if (!dofChoices.empty())
      this->setDof(dofChoices.front().first,i);
  }

  if (this->getEntity(i) == -1) {
    std::vector<FmSensorChoice> entChoices;
    sensor->getSensorEntities(entChoices,this->getDof(i));
    if (!entChoices.empty())
      this->setEntity(entChoices.front().first,i);
  }
}


FmMathFuncBase* FmEngine::getFunction() const
{
  return myFunction.getPointer();
}


void FmEngine::setFunction(FmMathFuncBase* func)
{
  myFunction.setRef(func);

  size_t nArg = func ? func->getNoArgs() : 1;

  if (nArg == 0)
    mySensor.clear();
  else if (mySensor.size() > nArg) {
    std::vector<FmSensorBase*> sens;
    mySensor.getPtrs(sens,true);
    sens.resize(nArg);
    mySensor.setPtrs(sens);
  }

  if (myEntityNames.getValue().size() > nArg)
    myEntityNames.getValue().resize(nArg);

  if (myEntities.getValue().size() > nArg)
    myEntities.getValue().resize(nArg);

  if (myDofs.getValue().size() > nArg)
    myDofs.getValue().resize(nArg);
}


const std::string& FmEngine::getEntityName(size_t i) const
{
  if (i < myEntityNames.getValue().size())
    if (this->getSensor(i))
      if (this->getSensor(i)->hasEntityChoice())
	return myEntityNames.getValue()[i];

  static const std::string empty;
  return empty;
}


int FmEngine::getEntity(size_t i) const
{
  if (i < myEntities.getValue().size())
    if (this->getSensor(i))
      if (this->getSensor(i)->hasEntityChoice())
	return myEntities.getValue()[i];

  return -1;
}


int FmEngine::getDof(size_t i) const
{
  if (i < myDofs.getValue().size())
    if (this->getSensor(i))
      if (this->getSensor(i)->hasDofChoice())
	return myDofs.getValue()[i];

  return -1;
}


void FmEngine::setEntityName(const std::string& name, size_t i)
{
  if (i >= this->getNoArgs()) return;

  if (i >= myEntityNames.getValue().size())
  {
    if (name.empty()) return;
    myEntityNames.getValue().resize(i+1);
  }

  myEntityNames.getValue()[i] = name;
}


void FmEngine::setEntity(int ent, size_t i)
{
  if (i >= this->getNoArgs()) return;

  if (i >= myEntities.getValue().size())
  {
    if (ent < 0) return;
    myEntities.getValue().resize(i+1,0);
  }

  myEntities.getValue()[i] = ent;
}


void FmEngine::setDof(int dof, size_t i)
{
  if (i >= this->getNoArgs()) return;

  if (i >= myDofs.getValue().size())
  {
    if (dof < 0) return;
    myDofs.getValue().resize(i+1,0);
  }

  myDofs.getValue()[i] = dof;
}


std::ostream& FmEngine::writeFMF(std::ostream& os)
{
  os <<"ENGINE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmEngine::readAndConnect(std::istream& is, std::ostream&)
{
  FmEngine* obj = new FmEngine(false);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  // The time sensor is not (any longer) stored on the model file,
  // so we need to resolve the reference to it manually here
  for (size_t i = 0; i < obj->mySensor.size(); i++)
    if (obj->mySensor[i].getRefTypeID() == FmTimeSensor::getClassTypeID())
      obj->setSensor(FmDB::getTimeSensor(),-i);

  obj->connect();
  return true;
}


bool FmEngine::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmEngine::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmEngine::getClassTypeID()))
    return false;

  FmEngine* copyObj = static_cast<FmEngine*>(obj);

  if (depth == FmBase::SHALLOW || depth >= FmBase::DEEP_APPEND)
  {
    size_t nArg = copyObj->mySensor.size();
    for (size_t i = 0; i < nArg; i++)
      this->setSensor(copyObj->getSensor(i),-i);
  }

  if (depth >= FmBase::DEEP_APPEND)
  {
    std::vector<FmIsControlledBase*> allControlled;
    copyObj->getReferringObjs(allControlled,"myEngine");
    for (FmIsControlledBase* ctrl : allControlled)
      ctrl->setEngine(this);

    if (copyObj->getFunction())
      this->setEngineToLinkFunctionFrom(copyObj);
  }

  return true;
}


void FmEngine::initAfterResolve()
{
  this->FmIsPlottedBase::initAfterResolve();

  for (size_t i = 0; i < mySensor.size(); i++)
  {
    this->setSensor(mySensor[i],i);
    if (dynamic_cast<FmStrainRosette*>(mySensor[i]->getMeasured()))
      if (FmDB::getModelFileVer() < FFaVersionNumber(7,3,2,10) && i < myDofs.getValue().size())
      {
        // Increment the strain rosette sensor DOF for older model files
        // which only had the GAGE_[123] choices available
        int iDof = myDofs.getValue()[i];
        if (iDof < FmIsMeasuredBase::NUM_DOF-4)
          this->setDof(iDof+4,i);
      }
  }

  FFaString eDesc(this->getUserDescription());
  if (eDesc.hasSubString("#TimeStepEngine") && FmDB::getActiveAnalysis()->myTimeIncEngine.isNull())
    ListUI <<"\n---> WARNING: Ignoring #TimeStepEngine"
           <<" in the description field for "<< this->getIdString()
           <<".\n     Select a General Function in the \"Time increment\" field"
           <<" in the Solver Setup dialog box instead.\n";

  // Conversion from older model files: Wave spectrum beta feature
  int nWave, spectrum;
  if ((nWave = eDesc.getIntAfter("#PiersonMoskowitz")))
    spectrum = FmfWaveSpectrum::PiersonMoskowitz;
  else if ((nWave = eDesc.getIntAfter("#JONSWAP")))
    spectrum = FmfWaveSpectrum::JONSWAP;
  else
    return;

  // Check that the old function actually was a sinusoidal...
  FmfSinusoidal* oldFunc = dynamic_cast<FmfSinusoidal*>(this->getFunction());
  if (!oldFunc) return;

  // Create a wave spectrum function
  FmfWaveSpectrum* newFunc = new FmfWaveSpectrum();
  newFunc->connect();
  newFunc->clone(oldFunc,FmBase::SHALLOW);
  newFunc->spectrum.setValue((FmfWaveSpectrum::FmSpectrum)spectrum);
  newFunc->rndPhase.setValue(nWave < 0);
  newFunc->nComp.setValue(nWave < 0 ? -nWave : nWave);

  // Move preview curve if old function has one
  FmCurveSet* curve = oldFunc->getPreviewCurve();
  if (curve) curve->setFunctionRef(newFunc);

  // Transfer the function parameters
  newFunc->myHs.setValue(oldFunc->getAmplitude());
  newFunc->myTp.setValue(oldFunc->getPeriodDelay());
  double omega1 = oldFunc->getFrequency();
  double omega0 = omega1 + newFunc->nComp.getValue()*oldFunc->getAmplitudeDisplacement();
  newFunc->myTrange.setValue(FmRange(1.0/omega0,1.0/omega1));

  // Attach the new function and erase the old one
  this->setFunction(newFunc);
  oldFunc->erase();
}


/*!
  Conversion function called from FmDB after loading a mechanism model,
  to make sure it is updated regarding linked functions in engines.
*/

void FmEngine::updateFunctionLinkedFromStuff()
{
  std::vector<FmMathFuncBase*> functions;
  FmDB::getAllFunctions(functions);

  for (FmMathFuncBase* function : functions)
  {
    std::vector<FmEngine*> engines;
    function->getEngines(engines);
    if (engines.size() == 1)
      engines.front()->setEngineToLinkFunctionFrom(NULL);
    else if (engines.size() > 1)
    {
      FmEngine* parent = NULL;
      for (FmEngine* engine : engines)
        if (!engine->isFunctionLinked())
          if ((parent = engine)) break;

      if (!parent) {
        parent = engines.front();
        parent->setEngineToLinkFunctionFrom(NULL);
      }

      for (FmEngine* engine : engines)
        if (engine != parent)
          engine->setEngineToLinkFunctionFrom(parent);
    }
  }
}


/*!
  Conversion method called from FmDB after loading a mechanism model,
  to translate old joint entity information for sensors.
*/

void FmEngine::translateJointSensorEntity()
{
  for (size_t i = 0; i < mySensor.size(); i++)
    if (this->getEntity(i) == FmIsMeasuredBase::POS && mySensor[i]->getMeasured())
      if (mySensor[i]->getMeasured()->isOfType(FmJointBase::getClassTypeID()))
        this->setEntity(FmIsMeasuredBase::REL_POS,i);
}


/*!
  Construct a unique base ID for a sensor instance.
  The solver uses a unique sensor object for each quantity that is measured
  on a structural object. In the DB they all share the same sensor object,
  so we can not just use the base ID of that sensor.
*/

int FmEngine::getSensorId(size_t i) const
{
  if (!this->getSensor(i)) return 0;

  int sensorId  = this->getSensor(i)->getBaseID();
  int sensorDof = this->getDof(i) + 1;
  int sensorEnt = this->getEntity(i) + 1;

  int measureId = sensorEnt, d = 1;
  for (; sensorEnt > 0; sensorEnt /= 10, d *= 10);
  measureId += d*sensorDof;
  for (; sensorDof > 0; sensorDof /= 10, d *= 10);
  measureId += d*sensorId;

  return measureId;
}


int FmEngine::printSolverEntry(FILE* fp)
{
  if (!this->isActive())
    return 0; // Engine is not in use, ignore it

  fprintf(fp,"&ENGINE\n");
  this->printID(fp);
  if ((this->isExternalFunc() || myOutput.getValue()) && !this->getTag().empty())
    fprintf(fp,"  tag = '%1.128s'\n", this->getTag().c_str());

  if (!myFunction.isNull()) // A function is optional
    fprintf(fp,"  functionId = %d\n", myFunction->getBaseID());

  size_t nArg = this->getNoArgs();
  if (nArg > 0)
  {
    fprintf(fp,"  nArg = %u, argSensorId =", (unsigned int)nArg);
    for (size_t i = 0; i < nArg; i++)
      fprintf(fp," %d", this->getSensorId(i));
    fprintf(fp,"\n");
  }

  fprintf(fp,"/\n\n");
  return 0;
}
