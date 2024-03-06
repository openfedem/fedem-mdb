// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FedemDB.C
  \brief Exported global functions for accessing the Fedem mechanism model.
  \details This file contains a collection of global functions serving as
  the API for creating and accessing a Fedem mechanism model from python.
*/

#include "vpmDB/FmModelLoader.H"
#include "vpmDB/FmSolverInput.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmMaterialProperty.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmfMathExpr.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FmfLinVar.H"
#include "vpmDB/FmfConstant.H"
#include "vpmDB/FmfScale.H"
#include "vpmDB/FmfRamp.H"
#include "vpmDB/FmfLimRamp.H"
#include "vpmDB/FmfDeviceFunction.H"
#include "vpmDB/FmfExternalFunction.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmUserDefinedElement.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFileSys.H"
#include "vpmDB/FmCreate.H"
#include "vpmDB/Icons/FmIconPixmapsMain.H"

#include "FiUserElmPlugin/FiUserElmPlugin.H"
#include "FFaFunctionLib/FFaUserFuncPlugin.H"

#define FFL_INIT_ONLY
#include "FFlLib/FFlMemPool.H"
#include "FFlLib/FFlFEParts/FFlNode.H"
#include "FFlLib/FFlFEParts/FFlAllFEParts.H"
#include "FFlLib/FFlIOAdaptors/FFlAllIOAdaptors.H"

#include "FFaLib/FFaCmdLineArg/FFaCmdLineArg.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaOS/FFaFilePath.H"

#include <cstring>
#include <cstdlib>
#include <fstream>

#if defined(win32) || defined(win64)
#include <windows.h>
#define DLLexport(ret) extern "C" __declspec(dllexport) ret
#else
#define DLLexport(ret) extern "C" ret
#endif


static IntVec  typeMap; //!< Python-to-Fedem object type mapping
static Strings funcMap; //!< Channel-to-tag input function mapping


/*!
  \brief Safe mapping from object type index to class Type ID.
*/

inline int classType (int objType)
{
  return objType < 0 || objType >= (int)typeMap.size() ? 0 : typeMap[objType];
}


/*!
  \brief Initializes the channel-to-tag input function mapping.
*/

static bool initFuncMap ()
{
  FmfExternalFunction* fn = NULL;
  std::vector<FmEngine*> engines;
  FmDB::getAllEngines(engines);
  for (FmEngine* eng : engines)
    if ((fn = dynamic_cast<FmfExternalFunction*>(eng->getFunction())))
    {
      size_t channel = fn->channel.getValue();
      if (channel > funcMap.size() && !eng->getTag().empty())
        funcMap.resize(channel);
      if (channel <= funcMap.size())
        funcMap[channel-1] = eng->getTag();
    }

  return !funcMap.empty();
}


/*!
  \brief A sub-class of FFaMsg that writes the list-messages to the log-file.
*/

class FileMsg : public FFaMsg
{
  std::ofstream os; //!< File stream for log-messages

public:
  //! \brief The constructor opens the log-file \a logf in append mode.
  FileMsg(const std::string& logf)
  {
    os.open(logf.c_str(),std::ios_base::app);
    if (!os)
      std::cerr <<" *** Failed to open log-file "<< logf <<"\n"
                <<"     Output will be written to console instead."<< std::endl;
  }

  //! \brief The destructor closes the log-file.
  virtual ~FileMsg() { if (os) os.close(); }

protected:
  //! \brief Writes the \a msg to the log-file.
  virtual void listVt(const std::string& msg, bool)
  {
    if (os)
      os << msg << std::flush;
    else
      std::cout << msg << std::flush;
  }
  //! \brief Writes the \a msg to the log-file.
  virtual int dialogVt(const std::string& msg, const FFaDialogType, const char**)
  {
    if (os)
      os << msg << std::endl;
    else
      std::cout << msg << std::endl;
    return -1;
  }
};


/*!
  \brief Opens the log-file associated with the given model file.
*/

static void openAssociatedLogFile (const char* fmmFile)
{
  std::string logFile(fmmFile);
  size_t idot = logFile.find_last_of('.');
  if (idot < logFile.size())
    logFile.replace(idot,std::string::npos,".log");
  else
    logFile.append(".log");
  FFaMsg::setMessager(new FileMsg(logFile));
}


/*!
  \brief Erases all dynamic objects from memory.
*/

static void cleanUpMemory ()
{
  FmDB::eraseAll(true);
  FFl::releaseAllReaders();
  FFl::releaseAllElements();
  FFlMemPool::deleteAllLinkMemPools();
  FFaMsg::setMessager();
  funcMap.clear();
}

////////////////////////////////////////////////////////////////////////////////

DLLexport(void) FmInit (const char* plugin1, const char* plugin2)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmInit()"<< std::endl;
  if (plugin1) std::cout <<"\tPlugin1 = "<< plugin1 << std::endl;
  if (plugin2) std::cout <<"\tPlugin2 = "<< plugin2 << std::endl;
#endif

  const char* program = "FedemDB";
  FFaCmdLineArg::init(1,const_cast<char**>(&program));
  // Add command-line options that will be attempted evaluated
  FFaCmdLineArg::instance()->addOption("memPoll",false,"Stop execution for memory polling");
  FFaCmdLineArg::instance()->addOption("allow3DofAttach",true,"Allow triads to be attached to 3-DOF nodes");
  FFaCmdLineArg::instance()->addOption("allowDepAttach",false,"Allow triads to be attached to dependent RGD nodes");
  FFaCmdLineArg::instance()->addOption("convertToLinear",1,"Convert parabolic shell and beam elements to linears");
  FFaCmdLineArg::instance()->addOption("ID_increment",0,"User ID increment");
  FFaCmdLineArg::instance()->addOption("reUseUserID",false,"Fill holes in user ID range");
  // Initialize the model database data structure
  FmDB::init();

  // Initialize the object type mapping (see the class FmType in enums.py)
  typeMap = {
    FmSimulationModelBase::getClassTypeID(),
    FmTriad::getClassTypeID(),
    FmBeam::getClassTypeID(),
    FmPart::getClassTypeID(),
    FmBeamProperty::getClassTypeID(),
    FmMaterialProperty::getClassTypeID(),
    FmJointBase::getClassTypeID(),
    FmRigidJoint::getClassTypeID(),
    FmRevJoint::getClassTypeID(),
    FmBallJoint::getClassTypeID(),
    FmFreeJoint::getClassTypeID(),
    FmPrismJoint::getClassTypeID(),
    FmCylJoint::getClassTypeID(),
    FmCamJoint::getClassTypeID(),
    FmLoad::getClassTypeID(),
    FmEngine::getClassTypeID(),
    FmSensorBase::getClassTypeID(),
    FmAxialSpring::getClassTypeID(),
    FmAxialDamper::getClassTypeID(),
    FmStrainRosette::getClassTypeID(),
    FmUserDefinedElement::getClassTypeID()
  };

  // Lambda function for loading plugin libraries (user-defined functions/elements)
  auto&& loadPlugin = [](const std::string& plugin)
  {
    char signature[128];
    bool loaded = false;
    if (FFaUserFuncPlugin::instance()->validate(plugin,128,signature))
      loaded = FFaUserFuncPlugin::instance()->load(plugin);
    else if (FiUserElmPlugin::instance()->validate(plugin,128,signature))
      loaded = FiUserElmPlugin::instance()->load(plugin);
    else
      ListUI <<"\nWarning : Ignoring plugin specification \""<< plugin <<"\"\n";
    if (loaded) ListUI <<"          "<< signature <<"\n";
    return loaded;
  };

  // Try to load the specified plugin(s)
  if (plugin1) loadPlugin(plugin1);
  if (plugin2) loadPlugin(plugin2);
}


DLLexport(void) FmNew (const char* newFile)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmNew(";
  if (newFile) std::cout << newFile;
  std::cout <<")"<< std::endl;
#endif
  if (FmDB::getFreeBaseID() > 1)
    cleanUpMemory();

  FmMechanism* mech = NULL;
  std::string newName = newFile ? newFile : "untitled.fmm";
  if (Fedem::loadTemplate(newName) > 1)
  {
    mech = FmDB::getMechanismObject();
    initFuncMap();
    Fedem::loadParts(); // In case a model with FE parts is used as template
  }
  else
  {
    mech = FmDB::newMechanism();
    mech->syncPath(newName, newFile ? true : false);
  }
  if (newFile)
    openAssociatedLogFile(newFile);
  FmDB::getActiveAnalysis();

  // Store paths to plugin libraries, if loaded
  const char* udePlugin = FiUserElmPlugin::instance()->getLibrary();
  const char* udfPlugin = FFaUserFuncPlugin::instance()->getLibrary();
  if (udePlugin) mech->activeElementPlugin.setValue(udePlugin);
  if (udfPlugin) mech->activeFunctionPlugin.setValue(udfPlugin);
}


DLLexport(bool) FmOpen (const char* fmmFile)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmOpen("<< fmmFile <<")"<< std::endl;
#endif
  if (FmDB::getFreeBaseID() > 1)
    cleanUpMemory();

  openAssociatedLogFile(fmmFile);
  if (Fedem::loadModel(fmmFile,fmmFile,'W') <= 0)
    return false;

  if (initFuncMap())
  {
    ListUI <<"\n --> External function mapping:\n";
    for (size_t channelIdx = 0; channelIdx < funcMap.size(); channelIdx++)
      if (!funcMap[channelIdx].empty())
        ListUI <<"     "<< 1+channelIdx <<" "<< funcMap[channelIdx] <<"\n";
  }

  return Fedem::loadParts();
}


DLLexport(void) FmClose (bool removeSingletons = false)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmClose()"<< std::endl;
#endif
  cleanUpMemory();
  if (removeSingletons)
  {
    FmDB::removeInstances();
    FFaCmdLineArg::removeInstance();
    FFaUserFuncPlugin::removeInstance();
    FiUserElmPlugin::removeInstance();
  }
}


DLLexport(int) FmCount (int objType)
{
  return FmDB::getObjectCount(classType(objType));
}


DLLexport(int) FmGetObjects (int* baseId, int objType, const char* tag = NULL)
{
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,classType(objType),NULL,tag);
  for (FmModelMemberBase* obj : objs)
    *(baseId++) = obj->getBaseID();

#ifdef FM_DEBUG
  std::cout <<"FmGetObjects("<< objType;
  if (tag) std::cout <<",\","<< tag <<"\"";
  std::cout <<"): "<< objs.size() << std::endl;
#endif
  return objs.size();
}


DLLexport(int) FmTagObjects (const int* baseId, int n, const char* tag)
{
  int nTagged = 0;
  for (int ix = 0; ix < n; ix++)
  {
    FmModelMemberBase* obj = FmDB::findObject(baseId[ix]);
    if (obj && obj->setTag(tag))
      nTagged++;
  }

#ifdef FM_DEBUG
  std::cout <<"FmTagObjects("<< n <<",\""<< tag <<"\""<<"): "
            << nTagged << std::endl;
#endif
  return nTagged;
}


/*!
  \brief Helper searching for an object of given type and base- or user ID.
  \details If \a id is negative while \a assumeUserId is \e true,
  then \a -id is interpreted as the base ID of the object to search for.
*/

template<class T> bool FmFind (int id, T*& obj, bool assumeUserId = false)
{
  if (!assumeUserId) // Assume base ID
    obj = dynamic_cast<T*>(FmDB::findObject(id));
  else if (id >= 0) // Assume user ID
    obj = dynamic_cast<T*>(FmDB::findID(T::getClassTypeID(),id));
  else // Assume the absolute value is the base ID
    obj = dynamic_cast<T*>(FmDB::findObject(-id));

  return obj != NULL;
}


/*!
  \brief Helper searching for a function with given user ID.
  \details If \a fid is negative, its absolute value is interpreted
  as the base ID instead.
*/

static FmEngine* FmFindFunction (int fid)
{
  FmEngine* engine = NULL;
  if (fid != 0 && !FmFind(fid,engine,true))
  {
    ListUI <<" *** Error: No function with";
    if (fid > 0)
      ListUI <<" user ID "<< fid;
    else
      ListUI <<" base ID "<< -fid;
    ListUI <<"\n";
  }

  return engine;
}


DLLexport(bool) FmReduce (char* rdbDir, int baseId)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmReduce("<< baseId <<")"<< std::endl;
#endif
  FmAnalysis* analy = FmDB::getActiveAnalysis(false);
  FmMechanism* mech = FmDB::getMechanismObject(false);
  if (!analy || !mech)
  {
    ListUI <<"\n\n===> Empty model. Nothing to reduce here.\n";
    return false;
  }

  FmPart* part;
  if (!FmFind(baseId,part))
  {
    ListUI <<"\n\n===> No FE part with baseId "<< baseId <<".\n";
    return false;
  }
  else if (!part->isFEPart())
  {
    ListUI <<"\n\n===> "<< part->getIdString(true) <<" is not an FE part.\n";
    return false;
  }
  else if (part->setValidBaseFTLFile().empty())
  {
    ListUI <<"\n\n===> No FE data file for "<< part->getIdString(true) <<"\n";
    return false;
  }

  if (Fedem::checkReducerFiles(part,analy->needMassMatrix()) > 0)
  {
    ListUI <<" ==> "<< part->getIdString(true) <<" is already reduced.\n";
    rdbDir[0] = '\0';
    return true;
  }

  std::string msg = Fedem::createReducerInput(analy,mech,part,"fedem_reducer");
  bool success = msg.find("===> ") > 0;
  if (success)
  {
    ListUI <<" ==> Successfully created reducer input files in "
           <<"\n     "<< msg;
    strcpy(rdbDir,msg.c_str());
  }
  else
    ListUI << msg;

  ListUI <<"\n";
  return success;
}


DLLexport(bool) FmSync (int baseId)
{
#ifdef FM_DEBUG
  std::cout <<"FmSync("<< baseId <<")"<< std::endl;
#endif
  FmPart* part;
  if (!FmFind(baseId,part))
  {
    ListUI <<"\n\n===> No FE part with baseId "<< baseId <<".\n";
    return false;
  }
  else if (part->isFEPart())
    return part->syncRSD();

  return true; // silently ignore for generic (or suppressed) parts
}


DLLexport(void) FmSolveSetup (double tStart, double tInc, double tStop,
                              double tQuasi, double eInc, int nModes,
                              const char* add_opts = NULL)
{
#ifdef FM_DEBUG
  std::cout <<"FmSolveSetup("<< tStart <<","<< tInc <<","<< tStop
            <<","<< tQuasi <<","<< eInc <<","<< nModes <<")"<< std::endl;
#endif
  FmAnalysis* analy = FmDB::getActiveAnalysis();

  analy->setStartTime(tStart);
  analy->setEndTime(tStop);
  analy->setTimeIncrement(tInc);
  analy->solveInitEquil.setValue(tQuasi >= tStart);
  analy->quasistaticEnable.setValue(tQuasi > tStart);
  if (tQuasi > tStart)
  {
    analy->quasistaticMode.setValue(tQuasi < tStop);
    if (tQuasi < tStop)
      analy->setQuasistaticUpToTime(tQuasi);
    else if (tQuasi > tStop)
      analy->setEndTime(tQuasi);
  }

  analy->setSolveEigenvalueFlag(nModes > 0);
  if (nModes > 0)
  {
    analy->setRequestedEigenmodes(nModes);
    analy->setEigenvalueSolutionTimeInterval(eInc);
  }

  if (add_opts)
  {
    std::string& my_opts = analy->solverAddOpts.getValue();
    if (my_opts.empty())
      analy->solverAddOpts.setValue(add_opts);
    else if (my_opts.find(add_opts) == std::string::npos) // avoid adding same options multiple times
      my_opts.append(std::string(" ") + std::string(add_opts));
  }
}


DLLexport(void) FmSolverTol (double eN, double dN, double vN, double rN)
{
#ifdef FM_DEBUG
  std::cout <<"FmSolverTol("<< eN <<","<< dN <<","<< vN <<","<< rN
            <<")"<< std::endl;
#endif
  FmAnalysis* analy = FmDB::getActiveAnalysis();
  analy->setTolEnergySum(eN,FmSolverConvergence::CONV_ALL_OF);
  analy->setTolDisplacementNorm(dN,FmSolverConvergence::CONV_ALL_OF);
  analy->setTolVelocityNorm(vN,FmSolverConvergence::CONV_ALL_OF);
  analy->setTolResidualNorm(rN,FmSolverConvergence::CONV_ALL_OF);
}


DLLexport(bool) FmSolve (char* rdbDir, bool keepRes,
                         const char* udePlugin, const char* udfPlugin)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmSolve("<< (keepRes ? "True)" : "False)") << std::endl;
#endif
  FmAnalysis* analy = FmDB::getActiveAnalysis(false);
  FmMechanism* mech = FmDB::getMechanismObject(false);
  if (!analy || !mech)
  {
    ListUI <<"\n\n===> Empty model. Nothing to solve here.\n";
    return false;
  }

  FmResultStatusData* currentRSD = mech->getResultStatusData();
  if (!currentRSD->isEmpty(true)) currentRSD->incrementTaskVer();

  if (!udePlugin && !mech->activeElementPlugin.getValue().empty())
  {
    const std::string& plugin = mech->activeElementPlugin.getValue();
    if (FiUserElmPlugin::instance()->validate(plugin))
      udePlugin = plugin.c_str();
    else
      ListUI <<"  ** Ignoring invalid user-defined element plugin: "<< plugin <<"\n";
  }
  if (!udfPlugin && !mech->activeFunctionPlugin.getValue().empty())
  {
    const std::string& plugin = mech->activeFunctionPlugin.getValue();
    if (FFaUserFuncPlugin::instance()->validate(plugin))
      udfPlugin = plugin.c_str();
    else
      ListUI <<"  ** Ignoring invalid user-defined function plugin: "<< plugin <<"\n";
  }

  Strings plugins, rdbPath;
  plugins.reserve(2);
  if (udePlugin)
  {
    ListUI <<"  => User-defined element plugin: "<< udePlugin <<"\n";
    plugins.push_back(udePlugin);
  }
  if (udfPlugin)
  {
    ListUI <<"  => User-defined function plugin: "<< udfPlugin <<"\n";
    plugins.push_back(udfPlugin);
  }
  std::string msg = Fedem::createSolverInput(analy,mech,NULL,"fedem_solver",
                                             plugins,rdbPath,false,keepRes);
  bool success = msg.find("fedem_solver") == 0;
  if (success)
  {
    ListUI <<" ==> Successfully created solver input files in "
           << mech->getAbsModelRDBPath();
    for (const std::string& dir : rdbPath) ListUI <<"\n     "<< dir;
    strcpy(rdbDir,rdbPath.front().c_str());
  }
  else
    ListUI << msg;

  ListUI <<"\n";
  return success;
}


DLLexport(bool) FmSave (const char* fmmFile = NULL)
{
#ifdef FM_DEBUG
  std::cout <<"\nFmSave(";
  if (fmmFile) std::cout << fmmFile;
  std::cout <<")"<< std::endl;
#endif

  FmMechanism* mech = FmDB::getMechanismObject(false);
  if (!mech)
  {
    ListUI <<"\n\n===> Empty model. Nothing to save here.\n";
    return false;
  }

  std::string modelFile = mech->getModelFileName();
  bool savingAs = false;
  if (fmmFile)
    savingAs = modelFile != fmmFile;
  else if (modelFile.empty())
  {
    ListUI <<"\n\n===> File name not specified. Model not saved.\n";
    return false;
  }

  if (savingAs)
  {
    // Update the mechanism to reflect the pathname changes
    std::string oldModelP = mech->getAbsModelFilePath();
    mech->syncPath(fmmFile);
    // Translate all relative pathnames according to the new model file location
    const std::string& newModelP = mech->getAbsModelFilePath();
    FmDB::translateRelativePaths(oldModelP,newModelP);
    // Open new log-file
    openAssociatedLogFile(fmmFile);
    modelFile = mech->getModelFileName();
  }
  bool isModelSaved = false;
  ListUI <<"===> Saving "<< modelFile <<"\n";

  // Save the model in <modelFile>.tmp so we don't loose the old file
  // in case of write failure due to disk full, etc.
  std::string tempFile = modelFile + ".tmp";
  std::ofstream s(tempFile.c_str(),std::ios::out);
  if (s)
  {
    std::vector<FmPart*> allParts;
    FmDB::getAllParts(allParts);
    for (FmPart* part : allParts)
      part->saveFEData();

    std::set<std::string> obsoleteFiles;
    FmResultStatusData  diskRSD;
    FmResultStatusData* currentRSD = mech->getResultStatusData();
#ifdef FM_DEBUG
    std::cout <<"\n   * Syncronizing the RDB: "<< currentRSD->getPath()
              <<" "<< currentRSD->getTaskName()
              <<" "<< currentRSD->getTaskVer() << std::endl;
#endif
    diskRSD.setPath(currentRSD->getPath());
    diskRSD.syncFromRDB(currentRSD->getCurrentTaskDirName(true,true),
                        currentRSD->getTaskName(), currentRSD->getTaskVer(),
                        &obsoleteFiles);
#ifdef FM_DEBUG
    std::set<std::string> rsdfiles, rdbfiles;
    currentRSD->getAllFileNames(rsdfiles);
    diskRSD.getAllFileNames(rdbfiles);
    std::cout <<"\n   * Files referred in model file:";
    for (const std::string& file : rsdfiles) std::cout <<"\n\t"<< file;
    std::cout <<"\n   * Files on disk:";
    for (const std::string& file : rdbfiles) std::cout <<"\n\t"<< file;
    std::cout <<"\n   * Obsolete files:";
    for (const std::string& file : obsoleteFiles) std::cout <<"\n\t"<< file;
    std::cout <<"\n"<< std::endl;
#endif
    currentRSD->copy(&diskRSD);

    for (const std::string& file : obsoleteFiles)
      if (!FmFileSys::deleteFile(file))
        ListUI <<"  -> Problems deleting file "<< file <<"\n";

    FmSubAssembly::mainFilePath = mech->getAbsModelFilePath();
    FmDB::updateModelVersionOnSave(false);
    isModelSaved = FmDB::reportAll(s);
    s.close();
  }

  if (isModelSaved)
  {
    FmFileSys::renameFile(modelFile, modelFile+".bak");
    FmFileSys::renameFile(tempFile, modelFile);
    ListUI <<"  -> Model saved in ";
  }
  else
  {
    FmFileSys::deleteFile(tempFile);
    ListUI <<"  -> Error: Could NOT save ";
  }
  ListUI << modelFile <<"\n";
  return isModelSaved;
}


////////////////////////////////////////////////////////////////////////////////
// Modelling functions
////////////////////////////////////////////////////////////////////////////////

DLLexport(int) FmCreateTriad (const char* description,
                              double x, double y, double z,
                              double rx = 0.0, double ry = 0.0, double rz = 0.0,
                              int owner = 0)
{
  FmTriad* triad = Fedem::createTriad(FaVec3(x,y,z),FmDB::findObject(owner));
  if (!triad) return 0;

  if (owner == 0 && fabs(rx)+fabs(ry)+fabs(rz) > 1.0e-6)
  {
    FaMat33 orientation;
    triad->setOrientation(orientation.eulerRotateZYX(FaVec3(rx,ry,rz)));
  }

  if (description)
    triad->setUserDescription(description);

  return triad->getBaseID();
}


DLLexport(int) FmTriadOnNode (const char* description, int node, int part)
{
  FmPart* ownerPart;
  if (!FmFind(part,ownerPart) || !ownerPart->isFEPart(true))
  {
    ListUI <<" *** Error: No FE part with base ID "<< part <<".\n";
    return -node;
  }

  // Check if there already is a triad for this node
  FmTriad* triad = ownerPart->getTriadAtNode(node);
  if (!triad)
  {
    FFlNode* feNode = ownerPart->getNode(node);
    if (!feNode)
    {
      ListUI <<" *** Error: No node "<< node <<" in FE "
             << ownerPart->getIdString(true) <<"\n";
      return -node;
    }

    // Create triad at the nodal point
    FaVec3 nodePos = ownerPart->getGlobalCS() * feNode->getPos();
    if (!(triad = Fedem::createTriad(nodePos,ownerPart)))
      return 0;
  }

  if (description)
    triad->setUserDescription(description);

  return triad->getBaseID();
}


DLLexport(int) FmCreateBeam (const char* description,
                             int t1, int t2, int cs = 0)
{
  FmTriad* triad1;
  if (!FmFind(t1,triad1))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmTriad* triad2;
  if (!FmFind(t2,triad2))
  {
    ListUI <<" *** Error: No triad with base ID "<< t2 <<".\n";
    return -t2;
  }

  FmBeam* beam = Fedem::createBeam(triad1,triad2);
  if (!beam) return 0;

  if (description)
    beam->setUserDescription(description);

  FmBeamProperty* bProp;
  if (FmFind(cs,bProp))
    beam->setProperty(bProp);

  return beam->getBaseID();
}


DLLexport(int) FmCreateBeamProperty (const char* description, int imat,
                                     int nprop, const double* prop)
{
  FmBeamProperty* pBeam = new FmBeamProperty();
  if (!pBeam) return 0;

  if (imat > 0)
  {
    // Pipe cross section, with material reference
    pBeam->crossSectionType.setValue(FmBeamProperty::PIPE);
    pBeam->material.setRef(dynamic_cast<FmMaterialProperty*>(FmDB::findObject(imat)));
    if (nprop > 0) pBeam->Do.setValue(prop[0]);
    if (nprop > 1) pBeam->Di.setValue(prop[1]);
    if (nprop > 3) pBeam->ShrRed.setValue(std::make_pair(prop[2],prop[3]));
    if (nprop > 5) pBeam->ShrCentre.setValue(std::make_pair(prop[4],prop[5]));
  }
  else
  {
    // Generic cross section
    pBeam->crossSectionType.setValue(FmBeamProperty::GENERIC);
    if (nprop > 0) pBeam->EA.setValue(prop[0]);
    if (nprop > 2) pBeam->EI.setValue(std::make_pair(prop[1],prop[2]));
    if (nprop > 3) pBeam->GIt.setValue(prop[3]);
    if (nprop > 4) pBeam->Mass.setValue(prop[4]);
    if (nprop > 5) pBeam->RoIp.setValue(prop[5]);
    if (nprop > 7) pBeam->GAs.setValue(std::make_pair(prop[6],prop[7]));
    if (nprop > 9) pBeam->ShrCentre.setValue(std::make_pair(prop[8],prop[9]));
  }

  ListUI <<"Creating Beam cross section property.\n";
  pBeam->connect();
  pBeam->updateDependentValues();

  if (description)
    pBeam->setUserDescription(description);

  return pBeam->getBaseID();
}


DLLexport(int) FmCreateMaterialProperty (const char* description,
                                         int nprop, const double* prop)
{
  FmMaterialProperty* pMat = new FmMaterialProperty();
  if (!pMat) return 0;

  if (nprop < 3) return -1;

  if (!pMat->updateProperties(prop[0],prop[1],pMat->G.getValue(),prop[2]))
  {
    pMat->erase();
    return -2;
  }

  ListUI <<"Creating material property.\n";
  pMat->connect();

  if (description)
    pMat->setUserDescription(description);

  return pMat->getBaseID();
}


/*!
  \brief Static helper that creates a polyline function object.
*/

static FmfLinVar* createPolyline (int n, const double* x, const double* y,
                                  int extrapol_type)
{
  if (n < 1) return NULL;

  FmfLinVar* f = new FmfLinVar();
  for (int i = 0; i < n; i++)
    f->addXYset(x[i],y[i]);
  f->setExtrapolationType(extrapol_type);
  f->connect();
  return f;
}


DLLexport(int) FmCreateSpring (const char* description, int t1, int t2,
                               double constLengthOrDefl, bool useConstDefl,
                               double init_Stiff_Coeff, int& spring_charac,
                               int sz, const double* x, const double* y,
                               int extrapol_type, int lcid = 0)
{
  FmTriad* triad1;
  if (!FmFind(t1,triad1))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmTriad* triad2;
  if (!FmFind(t2,triad2))
  {
    ListUI <<" *** Error: No triad with base ID "<< t2 <<".\n";
    return -t2;
  }

  FmAxialSpring* spring = Fedem::createAxialSpring(triad1,triad2);
  if (!spring) return 0;

  if (description)
    spring->setUserDescription(description);

  spring->setInitLengthOrDefl(constLengthOrDefl,useConstDefl);
  spring->setInitStiff(init_Stiff_Coeff);

  FmMathFuncBase* plx = NULL;
  if (spring_charac > 0)
  {
    // The Base Id of the spring stiffness function to use is given.
    // Check that its usage flag is valid
    if (FmFind(spring_charac,plx))
      if (plx->getFunctionUse() < FmMathFuncBase::SPR_TRA_STIFF ||
          plx->getFunctionUse() > FmMathFuncBase::SPR_TRA_FORCE)
        plx = NULL;
  }
  else
  {
    // Create a new spring stiffness function
    // spring stiffness as polyline function
    if ((plx = createPolyline(sz,x,y,extrapol_type)))
    {
      // -spring_charac is the spring function usage index
      plx->setFunctionUsage(FmMathFuncBase::SPR_TRA_STIFF-spring_charac);
      if (description)
        plx->setUserDescription(description);
      // Return the base Id of created function for subsequent springs
      spring_charac = plx->getBaseID();
    }
  }

  spring->setSpringCharOrStiffFunction(plx);

  // Check if a stress-free length function is specified.
  // Notice that a positive lcid value is assumed to be the FmEngine user ID
  // whereas a negative value is interpreted as the base ID.
  FmEngine* e;
  if (FmFind(lcid,e,true))
    spring->setEngine(e);

  return spring->getBaseID();
}


DLLexport(int) FmCreateDamper (const char* description, int t1, int t2,
                               bool def_vel_damper,
                               double init_Damp_Coeff, int& damp_charac,
                               int sz, const double* x, const double* y,
                               int extrapol_type)
{
  FmTriad* triad1;
  if (!FmFind(t1,triad1))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmTriad* triad2;
  if (!FmFind(t2,triad2))
  {
    ListUI <<" *** Error: No triad with base ID "<< t2 <<".\n";
    return -t2;
  }

  FmAxialDamper* damper = Fedem::createAxialDamper(triad1,triad2);
  if (!damper) return 0;

  if (description)
    damper->setUserDescription(description);

  damper->isDefDamper.setValue(def_vel_damper);
  damper->setInitDamp(init_Damp_Coeff);

  FmMathFuncBase* plx = NULL;
  if (damp_charac > 0)
  {
    // The Base Id of the damper function to use is given.
    // Check that its usage flag is valid
    if (FmFind(damp_charac,plx))
      if (plx->getFunctionUse() < FmMathFuncBase::DA_TRA_COEFF ||
          plx->getFunctionUse() > FmMathFuncBase::DA_TRA_FORCE)
        plx = NULL;
  }
  else
  {
    // Create a new damper function
    // damping coefficient as polyline function
    if ((plx = createPolyline(sz,x,y,extrapol_type)))
    {
      plx->setFunctionUsage(FmMathFuncBase::DA_TRA_COEFF+damp_charac);
      if (description)
        plx->setUserDescription(description);
      // Return the base Id of created function for subsequent dampers
      damp_charac = plx->getBaseID();
    }
  }

  damper->setFunction(plx);

  return damper->getBaseID();
}


DLLexport(int) FmCreateJoint (const char* description, int jType,
                              int t1, int* t2 = NULL, int nr_t2 = 0)
{
  FmTriad* follower;
  if (!FmFind(t1,follower))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmBase* triad1 = t2 && t2[0] > 0 ? FmDB::findObject(t2[0]) : NULL;
  FaVec3 jointPnt = follower->getGlobalTranslation();
  FmJointBase* jnt = NULL;

  if (jType <= 10) // point-to-point joint (rigid, revolute, ball or free joint)
    jnt = Fedem::createJoint(classType(jType),triad1,follower,&jointPnt);
  else if (jType <= 12 && t2 && nr_t2 >= 2) // prismatic or cylindric joint
  {
    FmBase* triad2 = t2[1] > 0 ? FmDB::findObject(t2[1]) : NULL;
    jnt = Fedem::createJoint(classType(jType),triad1,triad2,FaVec3(),follower);
  }
  if (!jnt) return -jType;

  if (description)
    jnt->setUserDescription(description);

  FmSMJointBase* sjoint = dynamic_cast<FmSMJointBase*>(jnt);
  if (sjoint && !sjoint->isOfType(FmFreeJoint::getClassTypeID()))
  {
    // Check if the two joint triads are co-located
    FmTriad* triad = sjoint->getItsMasterTriad();
    if (!triad->getGlobalTranslation().equals(jointPnt,1.0e-8))
      sjoint->setSlaveMovedAlong(false);
  }

  FmMMJointBase* mjoint = dynamic_cast<FmMMJointBase*>(jnt);
  if (mjoint && nr_t2 > 2)
  {
    // Additional glider triads for prismatic/cylindric joint
    FmTriad* triad = NULL;
    for (int i = 2; i < nr_t2; i++)
      if (FmFind(t2[i],triad))
        mjoint->addAsMasterTriad(triad);
  }

  return jnt->getBaseID();
}


/*!
  \brief Static helper that creates a general function object.
*/

static FmEngine* createGeneralFunction (FmMathFuncBase* func,
                                        const char* description,
                                        const char* tag = NULL)
{
  if (func)
  {
    func->connect();
    if (func->getFunctionUse() == FmMathFuncBase::NONE)
      func->setFunctionUse(FmMathFuncBase::GENERAL);
  }
  FmEngine* eng = new FmEngine();
  eng->setFunction(func);
  if (func)
    eng->setParentAssembly(func->getParentAssembly());
  eng->connect();

  if (description)
    eng->setUserDescription(description);

  if (tag)
    eng->setTag(tag);

  return eng;
}


/*!
  \brief Static helper that creates a general function object.
  \return User ID of the function, or base ID if \a returnBaseId is \e true.
*/

static int createFunction (bool returnBaseId, FmMathFuncBase* func,
                           const char* description, const char* tag)
{
  FmEngine* eng = createGeneralFunction(func,description,tag);
  return returnBaseId ? eng->getBaseID() : eng->getID();
}


DLLexport(int) FmCreateLoad (const char* description, int lType,
                             int t1, double dx, double dy, double dz,
                             const char* magnitude = NULL, int f1 = 0)
{
  FmTriad* triad1;
  if (!FmFind(t1,triad1))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmLoad* load = Fedem::createLoad(lType,triad1->getGlobalTranslation(),
                                   FaVec3(dx,dy,dz),triad1);
  if (!load) return 0;

  if (description)
    load->setUserDescription(description);

  // Check if a magnitude function is specified.
  // Notice that a positive f1 value is assumed to be the FmEngine user ID
  // whereas a negative value is interpreted as the base ID.
  FmEngine* e = FmFindFunction(f1);
  if (e) load->setEngine(e);

  if (e || !magnitude)
    return load->getBaseID();

  // Check if magnitude is a constant
  char* endPtr = NULL;
  double loVal = strtod(magnitude,&endPtr);
  if (strlen(endPtr) > 0) // assume it is an expression
  {
    FmMathFuncBase* f = new FmfMathExpr(magnitude);
    f->setParentAssembly(load->getParentAssembly());
    load->setEngine(createGeneralFunction(f,description));
  }
  else // constant load magnitude
    load->setInitLoad(loVal);

  return load->getBaseID();
}


DLLexport(int) FmCreateMathExprFunc (const char* descr, const char* tag,
                                     const char* expression,
                                     bool baseId = false)
{
  ListUI <<"Creating Math expression function.\n";
  return createFunction(baseId,new FmfMathExpr(expression),descr,tag);
}


DLLexport(int) FmCreateExternalFunc (const char* descr, const char* tag,
                                     bool baseId = false)
{
  ListUI <<"Creating external function.\n";
  return createFunction(baseId,new FmfExternalFunction(),descr,tag);
}


DLLexport(int) FmCreateSineFunc (const char* descr, const char* tag,
                                 const double* para, bool baseId = false)
{
  ListUI <<"Creating Sinusoidal function.\n";
  FmfSinusoidal* f = new FmfSinusoidal();
  f->setFrequency(para[0]);
  f->setPeriodDelay(para[1]);
  f->setAmplitude(para[2]);
  f->setAmplitudeDisplacement(para[3]);
  f->setMaxTime(para[4]);
  return createFunction(baseId,f,descr,tag);
}


DLLexport(int) FmCreateLinearFunc (const char* descr, const char* tag,
                                   const double* para, bool baseId = false)
{
  if (fabs(para[0]) <= 1.0e-12)
  {
    ListUI <<"Creating Constant function.\n";
    return createFunction(baseId,new FmfConstant(para[1]),descr,tag);
  }
  if (para[3] > para[2])
  {
    ListUI <<"Creating Limited Ramp function.\n";
    FmfLimRamp* f = new FmfLimRamp();
    f->setSlope(para[0]);
    f->setAmplitudeDisplacement(para[1]);
    f->setDelay(para[2]);
    f->setRampEnd(para[3]);
    return createFunction(baseId,f,descr,tag);
  }
  else if (fabs(para[1]) > 1.0e-12 || fabs(para[2]) > 1.0e-12)
  {
    ListUI <<"Creating Ramp function.\n";
    FmfRamp* f = new FmfRamp();
    f->setSlope(para[0]);
    f->setAmplitudeDisplacement(para[1]);
    f->setDelay(para[2]);
    return createFunction(baseId,f,descr,tag);
  }
  else
  {
    ListUI <<"Creating Scale function.\n";
    return createFunction(baseId,new FmfScale(para[0]),descr,tag);
  }
}


DLLexport(int) FmCreatePolyFunc (const char* descr, const char* tag,
                                 int sz, const double* x, const double* y,
                                 int extrapol_type, bool baseId = false)
{
  ListUI <<"Creating Polyline function.\n";
  return createFunction(baseId,createPolyline(sz,x,y,extrapol_type),descr,tag);
}


DLLexport(int) FmCreateDeviceFunc (const char* descr, const char* tag,
                                   const char* file_name, const char* chn_name,
                                   double scale, bool zero_adjust, double shift,
                                   bool baseId = false)
{
  ListUI <<"Creating Polyline-from-file function.\n";
  FmfDeviceFunction* f = new FmfDeviceFunction(file_name,chn_name);
  f->scaleFactor.setValue(scale);
  f->zeroAdjust.setValue(zero_adjust);
  f->verticalShift.setValue(shift);
  f->setFunctionUse(FmMathFuncBase::DRIVE_FILE);
  return createFunction(baseId,f,descr,tag);
}


/*!
  \brief Static helper that changes the argument of a general function object.
*/

static bool setArgument (FmEngine* engine, int id1, int id2, int var, int dof)
{
  FmIsMeasuredBase* object[2] = { NULL, NULL };
  if (!FmFind(id1,object[0]))
  {
    ListUI <<" *** Error: No measurable object with base ID "<< id1 <<".\n";
    return false;
  }
  if (id2 > 0 && !FmFind(id2,object[1]))
  {
    ListUI <<" *** Error: No measurable object with base ID "<< id2 <<".\n";
    return false;
  }

  if (object[1])
  {
    // Relative sensor
    engine->setSensor(Fedem::createSensor(object[0],object[1]));
    // Translate possible simple-sensor DOF and variable identifiers
    // to corresponding relative-sensor equivalents
    if (dof < FmIsMeasuredBase::REL)
      dof += FmIsMeasuredBase::REL_X;
    if (var <= FmIsMeasuredBase::POS || var == FmIsMeasuredBase::REL_POS)
      var = FmIsMeasuredBase::DISTANCE;
    else if (var <= FmIsMeasuredBase::GLOBAL_VEL)
      var = FmIsMeasuredBase::VEL;
    else if (var <= FmIsMeasuredBase::GLOBAL_ACC)
      var = FmIsMeasuredBase::ACCEL;
  }
  else
  {
    // Simple sensor
    engine->setSensor(Fedem::createSensor(object[0]));
    if (object[0]->isOfType(FmTriad::getClassTypeID()))
    {
      // Translate possible relative-sensor DOF identifiers
      // to the corresponding simple-sensor equivalents
      if (var == FmIsMeasuredBase::DISTANCE)
        var = FmIsMeasuredBase::POS;
      else if (var == FmIsMeasuredBase::VEL)
        var = FmIsMeasuredBase::GLOBAL_VEL;
      else if (var == FmIsMeasuredBase::ACCEL)
        var = FmIsMeasuredBase::GLOBAL_ACC;
    }
    else if (object[0]->isOfType(FmJointBase::getClassTypeID()))
    {
      // Translate possible triad DOF identifiers
      // to the corresponding joint DOF equivalents
      if (var <= FmIsMeasuredBase::POS || var == FmIsMeasuredBase::DISTANCE)
        var = FmIsMeasuredBase::REL_POS;
      else if (var <= FmIsMeasuredBase::GLOBAL_VEL)
        var = FmIsMeasuredBase::VEL;
      else if (var <= FmIsMeasuredBase::GLOBAL_ACC)
        var = FmIsMeasuredBase::ACCEL;
    }
  }
  engine->setEntity(var);
  engine->setDof(dof);

  if (engine->isDriveFile())
    engine->getFunction()->setFunctionUse(FmMathFuncBase::GENERAL);

  return engine->getSensor() != NULL;
}


DLLexport(bool) FmSetFunctionArg (int id, int var, int dof, int i1, int i2 = 0)
{
  FmEngine* engine = FmFindFunction(id);
  if (!engine) return false;

  return setArgument(engine,i1,i2,var,dof);
}


DLLexport(int) FmCreateSensor (const char* description, const char* tag,
                               int var, int dof, int id1, int id2 = 0)
{
  // Create a 1:1 function of the specified argument
  FmEngine* e = new FmEngine(false);
  if (!setArgument(e,id1,id2,var,dof))
  {
    e->erase();
    return -id1;
  }

  e->setParentAssembly(e->getSensor()->getParentAssembly());
  e->myOutput.setValue(true);
  e->connect();

  if (description)
    e->setUserDescription(description);

  if (tag)
    e->setTag(tag);

  return e->getID(); // Note: Here returning user ID instead
}


DLLexport(int) FmLoadPart (const char* feDataFile,
                           const char* description = NULL)
{
  if (!feDataFile || !FmFileSys::isReadable(feDataFile))
  {
    ListUI <<" *** Error: Can not read FE data file";
    if (feDataFile) ListUI <<" "<< feDataFile;
    ListUI <<"\n";
    return -1;
  }

  ListUI <<"Creating Part.\n";
  FmPart* fePart = new FmPart();
  fePart->connect();
  fePart->myCalculateMass.setValue(FmPart::FROM_FEM);
  if (description)
    fePart->setUserDescription(description);
  else
    fePart->setUserDescription(FFaFilePath::getBaseName(feDataFile,true));
  if (fePart->importPart(feDataFile,NULL,true))
    return fePart->getBaseID();

  fePart->erase();
  return -2;
}


DLLexport(int) FmCreateStrainRosette (const char* description,
                                      int id, int nnod, const int* nodes,
                                      const double* dir, double angle,
                                      bool startAtZero)
{
  FmPart* part;
  if (!FmFind(id,part) || !part->isFEPart(true))
  {
    ListUI <<" *** Error: No FE part with base ID "<< id <<".\n";
    return -id;
  }

  ListUI <<"Creating Strain Rosette.\n";
  FmStrainRosette* rosette = new FmStrainRosette();
  rosette->rosetteLink.setRef(part);
  rosette->rosetteType.setValue(FmStrainRosette::SINGLE_GAGE);
  rosette->numNodes.setValue(nnod);
  rosette->node1.setValue(nodes[0]);
  rosette->node2.setValue(nodes[1]);
  rosette->node3.setValue(nodes[2]);
  rosette->node4.setValue(nnod > 3 ? nodes[3] : 0);
  rosette->angleOrigin.setValue(FmStrainRosette::LINK_VECTOR);
  rosette->angleOriginVector.setValue(dir);
  rosette->angle.setValue(angle);
  rosette->removeStartStrains.setValue(startAtZero);
  rosette->connect();

  if (rosette->syncWithFEModel().back())
  {
    ListUI <<" *** Error: Invalid node numbers ("<< nodes[0];
    for (int i = 1; i < nnod; i++) ListUI <<","<< nodes[i];
    ListUI <<") on FE "<< part->getIdString(true) <<"\n";
    rosette->erase();
    return -1;
  }

  if (description)
    rosette->setUserDescription(description);

  return rosette->getBaseID();
}


DLLexport(int) FmCreateUDE2 (const char* description, int t1, int t2)
{
  FmTriad* triad1;
  if (!FmFind(t1,triad1))
  {
    ListUI <<" *** Error: No triad with base ID "<< t1 <<".\n";
    return -t1;
  }

  FmTriad* triad2;
  if (!FmFind(t2,triad2))
  {
    ListUI <<" *** Error: No triad with base ID "<< t2 <<".\n";
    return -t2;
  }

  char typeName[64];
  int eTypes[10];
  int nTypes = FiUserElmPlugin::instance()->getElementTypes(10,eTypes);
  for (int i = 0; i < nTypes; i++)
    if (FiUserElmPlugin::instance()->getTypeName(eTypes[i],64,typeName) == 2)
    {
      FmUserDefinedElement* uelm = new FmUserDefinedElement();
      uelm->connect();
      uelm->init(eTypes[i],typeName,{triad1,triad2});
      if (description)
        uelm->setUserDescription(description);
      return uelm->getBaseID();
    }

  ListUI <<" *** Error: No 2-noded user-defined element available.\n";
  return 0;
}


DLLexport(int) FmCreateAssembly (const char* description, int n, const int* id)
{
  std::vector<FmModelMemberBase*> members(n,NULL);
  for (int i = 0; i < n; i++)
    if (!FmFind(id[i],members[i]))
      ListUI <<"  ** Warning: No object with base ID "<< id[i] <<" (ignored).\n";

  FmSubAssembly* subAss = Fedem::createSubAssembly(members);
  subAss->setUserDescription(description);

  return subAss->getBaseID();
}


DLLexport(int) FmGetNode (int id, const double* pos)
{
  FmPart* part;
  if (!FmFind(id,part) || !part->isFEPart(true))
  {
    ListUI <<" *** Error: No FE part with base ID "<< id <<".\n";
    return -id;
  }

  FFlNode* node = part->getClosestNode(pos);
#ifdef FM_DEBUG
  std::cout <<"FmGetNode("<< pos[0] <<","<< pos[1] <<","<< pos[2] <<"): ";
  if (!node)
    std::cout <<"(none)";
  else
    std::cout << node->getID();
  std::cout << std::endl;
#endif
  return node ? node->getID() : 0;
}


DLLexport(bool) FmGetPosition (int id, double* pos)
{
  FmIsPositionedBase* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No positioned object with base ID "<< id <<".\n";
    return false;
  }

  FaVec3 X = object->getGlobalCS().translation();
  for (int i = 0; i < 3; i++) pos[i] = X[i];

  return true;
}


DLLexport(bool) FmMoveObject (int id, const double* delta,
                              int traRefId = 0, int rotRefId = 0)
{
  FmIsPositionedBase* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No movable object with base ID "<< id <<".\n";
    return false;
  }

  FmIsPositionedBase* traRef = NULL;
  if (traRefId > 0 && !FmFind(traRefId,traRef))
  {
    ListUI <<" *** Error: No movable object with base ID "<< traRefId <<".\n";
    return false;
  }
  else if (traRef)
    object->setPosRef(traRef);

  FmIsPositionedBase* rotRef = NULL;
  if (rotRefId > 0 && !FmFind(rotRefId,rotRef))
  {
    ListUI <<" *** Error: No movable object with base ID "<< rotRefId <<".\n";
    return false;
  }
  else if (rotRef)
    object->setRotRef(rotRef);

  // Update the location attribute of this object,
  // assuming delta contain Cartesian coordinate offsets
  // and Euler Z-Y-X angle increments (in degrees).
  // If a rotation reference object is provided, the Euler Z-Y-X angles
  // are considered relative to the coordinate system of the referenced object.
  FFa3DLocation loc = object->getLocation();
  loc.changePosType(FFa3DLocation::CART_X_Y_Z);
  loc.changeRotType(FFa3DLocation::EUL_Z_Y_X);
  loc[0] += FaVec3(delta);
  if (rotRef)
    loc[1] = FaVec3(delta+3);
  else
    loc[1] += FaVec3(delta+3);

#ifdef FM_DEBUG
  std::cout <<"\nFmMoveObject("<< id <<","<< traRefId <<","<< rotRefId
            <<"):"<< loc << std::endl;
#endif
  object->setLocation(loc);

  return true;
}


DLLexport(bool) FmAddMass (int id, int nMass, const double* mass, int fid = 0)
{
  FmTriad* triad;
  if (!FmFind(id,triad))
  {
    ListUI <<" *** Error: No triad with base ID "<< id <<".\n";
    return false;
  }
  else if (nMass < 1)
  {
    ListUI <<" *** Error: Empty mass array for triad "<< id <<".\n";
    return false;
  }

  for (int dof = 0; dof < 3; dof++)
  {
    triad->setAddMass(dof,mass[0]);
    if (1+dof < nMass)
      triad->setAddMass(3+dof,mass[1+dof]);
  }

  // Check if a mass scaling function is specified.
  // Notice that a positive fid value is assumed to be the FmEngine user ID
  // whereas a negative value is interpreted as the base ID.
  FmEngine* engine = FmFindFunction(fid);
  if (engine)
    triad->setMassEngine(engine);

  return true;
}


DLLexport(bool) FmConstrainObject (int id, int dof, int dofStatus)
{
  FmHasDOFsBase* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No object with DOFs and base ID "<< id <<".\n";
    return false;
  }

  if (dof == FmHasDOFsBase::Z_TRANS && object->isOfType(FmRevJoint::getClassTypeID()))
    static_cast<FmRevJoint*>(object)->setHasTzDOF(true);

  if (dof < FmHasDOFsBase::MAX_DOF)
    object->setStatusForDOF(dof,dofStatus);
  else for (dof = 0; dof < FmHasDOFsBase::MAX_DOF; dof++)
    object->setStatusForDOF(dof,dofStatus);

  return true;
}


DLLexport(bool) FmDofProperty (int id, int dof, int propertyType,
                               double value, int fid = 0)
{
  FmHasDOFsBase* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No object with DOFs and base ID "<< id <<".\n";
    return false;
  }

  FmEngine* engine = NULL;
  if (fid != 0 && (propertyType == 1 || propertyType == 4))
    if (!(engine = FmFindFunction(fid)))
      return false;

  // Convenience lambda function for generating error message.
  auto&& jointError = [id,dof](const char* prop)
  {
    ListUI <<" *** Error: Can't assign "<< prop <<" in DOF "<< dof
           <<" for Joint ["<< id <<"].\n"
           <<"            It needs to be set as SPRING_CONSTRAINED first.\n";
    return false;
  };

  FmJointBase* jnt = dynamic_cast<FmJointBase*>(object);
  switch (propertyType)
  {
    case 0: // Initial velocity
      if (object->getStatusOfDOF(dof) != FmHasDOFsBase::FIXED)
        object->setInitVel(dof,value);
      else
      {
        ListUI <<" *** Error: Can't assign initial velocity to Fixed DOF "
               << dof <<" in object ["<< id <<"].\n";
        return false;
      }
      break;

    case 1: // Motion/load magnitude
      if (object->getStatusOfDOF(dof) == FmHasDOFsBase::PRESCRIBED)
      {
        if (engine)
          object->getMotionAtDOF(dof,true)->setEngine(engine);
        else
          object->getMotionAtDOF(dof,true)->setInitMotion(value);
      }
      else if (object->getStatusOfDOF(dof) != FmHasDOFsBase::FIXED)
      {
        if (engine)
          object->getLoadAtDOF(dof,true)->setEngine(engine);
        else
          object->getLoadAtDOF(dof,true)->setInitLoad(value);
      }
      else
      {
        ListUI <<" *** Error: Can't assign load to Fixed DOF "
               << dof <<" in object ["<< id <<"].\n";
        return false;
      }
      break;

    case 2: // Spring stiffness
      if (jnt && jnt->getStatusOfDOF(dof) >= FmHasDOFsBase::SPRING_CONSTRAINED)
        jnt->getSpringAtDOF(dof,true)->setInitStiff(value);
      else
        return jointError("spring stiffness");
      break;

    case 3: // Damping coefficient
      if (jnt && jnt->getStatusOfDOF(dof) >= FmHasDOFsBase::SPRING_CONSTRAINED)
        jnt->getDamperAtDOF(dof,true)->setInitDamp(value);
      else
        return jointError("damping coefficient");
      break;

    case 4: // Stress-free length/angle control
      if (jnt && jnt->getStatusOfDOF(dof) >= FmHasDOFsBase::SPRING_CONSTRAINED)
      {
        // Note: Not creating a spring object here
        FmJointSpring* spr = jnt->getSpringAtDOF(dof);
        if (!spr) break; // Silently ignore if not existing

        if (engine)
          spr->setEngine(engine);
        else
          spr->setInitLengthOrDefl(value,true);
      }
      else
        return jointError("stress-free length change");
      break;

    default: // Logic error, should never get here
      return false;
  }

  return true;
}


DLLexport(bool) FmStructDamp (int id, double alpha1, double alpha2)
{
  FmLink* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No link with base ID "<< id <<".\n";
    return false;
  }

  object->alpha1.setValue(alpha1);
  object->alpha2.setValue(alpha2);

  return true;
}


DLLexport(bool) FmReduceOpts (int id, int nComp, bool consMass)
{
  FmPart* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No part with base ID "<< id <<".\n";
    return false;
  }

  object->nGenModes.setValue(nComp);
  object->useConsistentMassMatrix.setValue(consMass);

  return true;
}


DLLexport(bool) FmRecoverOpts (int id, int recoveryFlag, bool amend)
{
  FmPart* object;
  if (!FmFind(id,object))
  {
    ListUI <<" *** Error: No part with base ID "<< id <<".\n";
    return false;
  }

  if (recoveryFlag < 0 || recoveryFlag > 3)
    ListUI <<"  ** Warning: Invalid part recovery flag "<< recoveryFlag
           <<" (ignored).\n";
  else if (recoveryFlag > 0 && amend)
  {
    recoveryFlag |= object->recoveryDuringSolve.getValue();
    object->recoveryDuringSolve.setValue(recoveryFlag);
  }
  else
    object->recoveryDuringSolve.setValue(recoveryFlag);

  return true;
}


DLLexport(bool) FmGetFuncTag (int channel, char* tag)
{
  if (channel-- > (int)funcMap.size())
    return false;

  strcpy(tag,funcMap[channel].c_str());
  return true;
}
