// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmFileSys.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(FcMECHANISM, FmMechanism, FmIsPlottedBase);


FmMechanism::FmMechanism()
{
  Fmd_CONSTRUCTOR_INIT(FmMechanism);

  myInitialRSD = NULL;

  FFA_FIELD_DEFAULT_INIT(cadModelFileName,    "CAD_MODEL_FILE_NAME");
  FFA_FIELD_DEFAULT_INIT(cadConfigurationName,"CAD_CONFIGURATION_NAME");

  FFA_FIELD_DEFAULT_INIT(activeFunctionPlugin,"FUNCTION_PLUGIN");
  FFA_FIELD_DEFAULT_INIT(activeElementPlugin, "ELEMENT_PLUGIN");

  FFA_FIELD_INIT(maxConcurrentProcesses,  1, "MAX_CONCURRENT_PROCESSES");
  FFA_FIELD_DEFAULT_INIT(propertyRepository, "PROPERTY_REPOSITORY");
  FFA_FIELD_DEFAULT_INIT(modelLinkRepository,"MODEL_LINK_REPOSITORY");
  FFA_FIELD_DEFAULT_INIT(modelDatabaseUnits, "MODEL_DATABASE_UNITS");
  FFA_FIELD_DEFAULT_INIT(myResultStatusData, "RESULT_STATUS_DATA");
  FFA_FIELD_DEFAULT_INIT(myDisabledResults,  "DISABLED_RESULT_FILES");

  FFA_FIELD_INIT(positionTolerance, 1.0e-4, "POSITION_TOLERANCE");
  FFA_FIELD_INIT(gravity, FaVec3(0.0,0.0,-9.81), "GRAVITY");
  FFA_FIELD_DEFAULT_INIT(initVel, "GLOBAL_INITIAL_VELOCITY");

  FFA_FIELD_DEFAULT_INIT(FmDB::getEarthLink()->myCS, "EARTH_COORDINATE_SYSTEM");
}


FmMechanism::~FmMechanism()
{
  this->disconnect();

  delete myInitialRSD;
}


FmResultStatusData* FmMechanism::getResultStatusData(bool current)
{
  if (current)
    return &myResultStatusData.getValue();

  if (!myInitialRSD)
    myInitialRSD = new FmResultStatusData();

  return myInitialRSD;
}


std::string FmMechanism::getModelName(bool keepExt) const
{
  if (keepExt)
    return FFaFilePath::getFileName(myModelFileName);
  else
    return FFaFilePath::getBaseName(myModelFileName,true);
}


std::string FmMechanism::getAbsModelLRDBPath(bool createDir) const
{
  std::string retVar = modelLinkRepository.getValue();
  if (!retVar.empty())
    FFaFilePath::makeItAbsolute(retVar,myAbsModelFilePath);
  else
  {
    // An internal link repository is used
    retVar = myAbsModelRDBPath;
    if (createDir)
      if (!FmFileSys::verifyDirectory(retVar))
      {
	ListUI <<"===> Could not access directory "<< retVar <<"\n";
	return "";
      }

    FFaFilePath::appendToPath(retVar,"link_DB");
  }

  // Ensure that the directory really exist
  if (createDir)
    if (!FmFileSys::verifyDirectory(retVar))
    {
      ListUI <<"===> Could not access directory "<< retVar <<"\n";
      return "";
    }

  return retVar;
}


std::string FmMechanism::getPropertyLibPath(bool createDir) const
{
  std::string retVar = propertyRepository.getValue();
  if (retVar.empty())
  {
    // Create the default location of the property library
#if defined(win32) || defined(win64)
    const char* home = getenv("USERPROFILE");
#else
    const char* home = getenv("HOME");
#endif
    if (!home) return retVar;

    retVar = FFaFilePath::appendFileNameToPath(home,"Fedem_properties");
    if (createDir)
      const_cast<FmMechanism*>(this)->propertyRepository.setValue(retVar);
  }

  // Ensure that the directory really exist
  if (createDir)
    if (!FmFileSys::verifyDirectory(retVar))
    {
      ListUI <<"===> Could not access directory "<< retVar <<"\n";
      return "";
    }

  return retVar;
}


std::string FmMechanism::getAirFoilLibPath() const
{
  std::string retVar = this->getPropertyLibPath(false);
  return FFaFilePath::appendToPath(retVar,"AeroData");
}


std::string FmMechanism::getAbsBladeFolderPath() const
{
  return FFaFilePath::getBaseName(myModelFileName) + "_blade";
}


bool FmMechanism::setGravity(const FaVec3& vec)
{
  if (!gravity.setValue(vec))
    return false;

  if (FmDB::getMechanismObject(false) == this)
  {
    FmDB::drawGVector();
    FmDB::drawSea();
  }
  return true;
}


bool FmMechanism::isEnabled(const std::string& fileName) const
{
  for (const std::string& file : myDisabledResults.getValue())
    if (fileName.find(file) != std::string::npos)
      return false;

  return true;
}


bool FmMechanism::getDisabledResultFiles(Strings& disabledFiles,
                                         bool absPath) const
{
  disabledFiles.clear();
  disabledFiles.reserve(myDisabledResults.getValue().size());
  for (const std::string& file : myDisabledResults.getValue())
    if (absPath && FFaFilePath::isRelativePath(file))
      disabledFiles.push_back(FFaFilePath::appendFileNameToPath(myAbsModelRDBPath,file));
    else
      disabledFiles.push_back(file);

  return !disabledFiles.empty();
}


bool FmMechanism::hasDisabledFiles() const
{
  return !myDisabledResults.getValue().empty();
}


bool FmMechanism::disableResultFile(const std::string& fileName)
{
  return myDisabledResults.getValue().insert(fileName).second;
}


bool FmMechanism::enableResultFile(const std::string& fileName)
{
  return myDisabledResults.getValue().erase(fileName) > 0;
}


bool FmMechanism::enableAllResultFiles()
{
  if (myDisabledResults.getValue().empty())
    return false;

  myDisabledResults.getValue().clear();
  return true;
}


bool FmMechanism::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmMechanism::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmMechanism::getClassTypeID());
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmMechanism::writeFMF(std::ostream& os)
{
  os <<"MECHANISM\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmMechanism::readAndConnect(std::istream& is, std::ostream&)
{
  // If we are importing a regular model as a sub-assembly,
  // the mechanism object read from file should be ignored
  if (FmSubAssembly::old2newAssID.second > 0)
    return true;

  FmMechanism* obj = new FmMechanism();
  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  // A mechanism object should always exist before reading a new model
  FmMechanism* old = FmDB::getMechanismObject(false);
  if (!old) return obj->connect(); // Probably we are reading a template file

  if (old->getID() != obj->getID())
  {
    // This should normally not happen
    ListUI <<"===> Multiple Mechanism objects detected.\n"
           <<"     Please check your model file.\n";
    return obj->connect();
  }

  // Clone the read object into the existing one
  if (!old->activeFunctionPlugin.getValue().empty())
    obj->activeFunctionPlugin.setValue(old->activeFunctionPlugin.getValue());
  if (!old->activeElementPlugin.getValue().empty())
    obj->activeElementPlugin.setValue(old->activeElementPlugin.getValue());
  old->clone(obj,FmBase::DEEP_REPLACE);
  old->sendSignal(FmModelMemberBase::MODEL_MEMBER_CHANGED);
  return obj->erase();
}


int FmMechanism::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&MECHANISM\n");
  this->printID(fp);

  // Function for prescribed time step size
  FmEngine* timeEngine = FmDB::getActiveAnalysis()->getEngine();
  if (timeEngine)
    fprintf(fp,"  timeIncrEngine = %d\n",timeEngine->getBaseID());

  // Beta feature: Functions for stop time and modal damping
  int timeStopId = 0;
  int modalDmpId = 0;
  std::vector<FmEngine*> allEngines;
  FmDB::getAllEngines(allEngines);
  for (FmEngine* engine : allEngines)
    if (FFaString(engine->getUserDescription()).hasSubString("#ModalDamping"))
      modalDmpId = engine->getBaseID();
    else if (FFaString(engine->getUserDescription()).hasSubString("#EndTime"))
      timeStopId = engine->getBaseID();

  if (timeStopId) fprintf(fp,"  timeEndEngine = %d\n",timeStopId);
  if (modalDmpId) fprintf(fp,"  modalDmpFunction = %d\n",modalDmpId);

  // Calculation of the length-scale weight factor, i.e.,
  // largest distance in x, y or z direction for all triads in the model
  std::vector<FmTriad*> allTriads;
  FmDB::getAllTriads(allTriads);

  FaVec3 maxPos, minPos;
  if (!allTriads.empty())
    minPos = maxPos = allTriads.front()->getGlobalTranslation();

  for (FmTriad* triad : allTriads)
  {
    FaVec3 currentPos = triad->getGlobalTranslation();
    for (int j = 0; j < 3; j++)
    {
      if (currentPos[j] > maxPos[j]) maxPos[j] = currentPos[j];
      if (currentPos[j] < minPos[j]) minPos[j] = currentPos[j];
    }
  }

  FaVec3 distance = maxPos - minPos;
  double wl = distance[0];
  if (wl < distance[1]) wl = distance[1];
  if (wl < distance[2]) wl = distance[2];

  if (wl >= 1.0e-6)
    fprintf(fp,"  weightTranslation =%17.9e\n",1.0/wl);
  else
    fprintf(fp,"  weightTranslation =%17.9e\n",1.0);

  fprintf(fp,"  weightRotation    =%17.9e\n",1.0);
  fprintf(fp,"  weightGeneralized =%17.9e\n",1.0);

  // Variables to be saved:
  // 1 - Centre of gravity
  // 2 - Energies
  // 3 - Algorithm parameters
  this->writeSaveVar(fp,3);

  fprintf(fp,"/\n\n");
  return 0;
}


void FmMechanism::initAfterResolve()
{
  this->FmIsPlottedBase::initAfterResolve();

  myResultStatusData.getValue().setPath(myAbsModelRDBPath);

  if (FmDB::getMechanismObject(false) != this)
    return;

  // Update the earth link coordinate system
  FmLink* earth = FmDB::getEarthLink();
  earth->setLocalCS(earth->myCS.getValue());
  earth->updateDisplayCS();

  // Update from old model file
  if (maxConcurrentProcesses.wasOnFile())
    FmDB::getActiveAnalysis()->maxConcurrentProcesses.setValue(maxConcurrentProcesses.getValue());
}


void FmMechanism::syncPath(const std::string& name, bool updateRSD)
{
  std::string modelPath = FFaFilePath::getPath(name);
  std::string modelRDB  = FFaFilePath::getBaseName(name,true) + "_RDB";

  myModelFileName    = name;
  myAbsModelFilePath = modelPath;
  myAbsModelRDBPath  = FFaFilePath::appendToPath(modelPath,modelRDB);

  if (updateRSD)
  {
    this->getResultStatusData(true)->setPath(myAbsModelRDBPath);
    this->getResultStatusData(false)->copy(this->getResultStatusData());
  }
}
