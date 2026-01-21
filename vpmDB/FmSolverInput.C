// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSolverInput.H"
#include "vpmDB/FmSolverParser.H"
#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFileSys.H"

#include "FFaLib/FFaCmdLineArg/FFaOptionFileCreator.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaOS/FFaTag.H"

#include <fstream>


bool Fedem::validFileCheck(const std::string& filename,
                           unsigned long int wantCS,
                           std::string* missingFiles, int* wrongCS)
{
  FILE* fp = fopen(filename.c_str(),"rb");
  if (!fp)
  {
    if (missingFiles)
    {
#ifdef FM_DEBUG
      std::cout <<" --> Checksum: File check FAILED: ["<< filename
                <<"]"<< std::endl;
#endif
      if (missingFiles->empty())
        *missingFiles = filename;
      else
        *missingFiles += ",\n     " + filename;
    }
    return false;
  }

  unsigned int cs = 0;
  if (wantCS > 0)
  {
    std::string tag;
    FFaTag::read(fp,tag,cs);
  }
  fclose(fp);

  if (wantCS == 0)
    return true; // Silently ignore file checksum
  else if (cs == wantCS)
  {
#if FM_DEBUG > 2
    std::cout <<" --> Checksum PASSED "<< filename <<" cs="<< cs << std::endl;
#endif
    return true;
  }

#ifdef FM_DEBUG
  std::cout <<" --> Checksum FAILED "<< filename <<" cs="<< cs << std::endl;
#endif
  if (wrongCS)
    ++(*wrongCS); // accepting incorrect file checksum
  else
    return false;

  return true;
}


/*!
  \return -1 : Some or all files are missing
  \return  0 : Some or all files are either missing or have checksum mismatch
  \return  1 : All files are present and with correct checksum
  \return  2 : All files are present, but with unknown checksum status
  \return  3 : All files are present, but some have incorrect checksum
*/

int Fedem::checkReducerFiles(const FmPart* part,
                             bool needMassMatrix,
                             char checkingWhich,
                             bool preparingForBatch,
                             unsigned long int wantCS)
{
  const FmResultStatusData& partRSD = part->myRSD.getValue();
  if (partRSD.isEmpty()) return 0;

#ifdef FM_DEBUG
  std::cout <<" --> Checking files and checksum ("<< checkingWhich
            <<") in "<< partRSD.getCurrentTaskDirName() << std::endl;
#endif

  // Skip checksum control when FE-data is not loaded and no cached checksum
  // is available either. If reduced files are found, they are assumed OK.
  unsigned long int theCS = part->overrideChecksum.getValue() ? 0 : wantCS;

  // Check that we in fact have a directory here - if not just skip tests
  std::string rdbPath = partRSD.getCurrentTaskDirName(false,true);
  FFaFilePath::makeItAbsolute(rdbPath,part->getAbsFilePath());
  bool valid = FmFileSys::verifyDirectory(rdbPath,false);
  rdbPath += FFaFilePath::getPathSeparator();
  std::set<std::string> redFiles = partRSD.getFileSet();

  int wrongCS = -1; // Flag for accepting incorrect checksum
  if (redFiles.find("fedem_reducer.res") != redFiles.end())
  {
    // Find which Fedem version this part was reduced in,
    // by parsing the header of the fedem_reducer.res file
    char cline[128];
    std::ifstream is(rdbPath + "fedem_reducer.res", std::ios::in);
    while (is.getline(cline,128))
      if (!strncmp(cline,"     Module version:",20))
      {
        FFaVersionNumber reducerVersion(cline+20);
#ifdef FM_DEBUG
        std::cout << part->getIdString(true) <<" was reduced with Fedem "
                  << reducerVersion.getString() << std::endl;
#endif
        // Due to an error in the checksum algorithm implemented in R7.2.2,
        // any mismatch is accepted if reduced in Fedem R7.5.1 or older,
        // but not older than R7.2.2
        if (reducerVersion < FFaVersionNumber(7,5,2) &&
            reducerVersion > FFaVersionNumber(7,2,2))
          wrongCS = 0; // Checksum mismatch will be accepted
        break;
      }
  }

  // Lambda function checking the validity of a file in the part RSD
  auto&& checkFile = [&valid,redFiles,rdbPath,theCS,&wrongCS](const std::string& file)
  {
    if (file.empty() || redFiles.find(file) == redFiles.end())
      valid = false;
    else if (valid)
      valid = Fedem::validFileCheck(rdbPath+file, theCS, NULL,
                                    wrongCS < 0 ? NULL : &wrongCS);
  };

  if (checkingWhich == 'A' || checkingWhich == 'S')
  {
    // Check files needed for dynamics or quasi-static simulation

    checkFile(part->SMatFile.getValue());

    if (needMassMatrix)
      checkFile(part->MMatFile.getValue());

    if (FmDB::getGrav().length() > 1.0e-8)
      checkFile(part->GMatFile.getValue());

    if (part->hasLoads())
      checkFile(part->LMatFile.getValue());
  }

  if (checkingWhich == 'A' || checkingWhich == 'R')
  {
    // Check files needed for part result recovery

    checkFile(part->BMatFile.getValue());

    int ngen = part->nGenModes.getValue();
    if (ngen < 0 && !part->useNonlinearReduction.getValue())
      // Static gravity modes are used
      checkFile(part->DMatFile.getValue());
    else if (ngen > 0)
      // Component modes are used
      checkFile(part->EMatFile.getValue());

    checkFile(part->SAMdataFile.getValue());
  }

  if (!valid && preparingForBatch && wantCS) // Check if a checksum file exists
    valid = Fedem::validFileCheck(rdbPath + part->getBaseFTLName() + ".chk");

  if (valid) // All files were found
    return wantCS > 0 ? (wrongCS > 0 ? 3 : 1) : 2;
  else if (part->overrideChecksum.getValue())
    return -1;

#ifdef FM_DEBUG
  std::cout <<" --> No valid solution in part RSD "<< rdbPath << std::endl;
#endif
  return 0;
}


std::string Fedem::createReducerInput(FmAnalysis* analysis,
                                      FmMechanism* mech,
                                      FmPart* part,
                                      const std::string& solverName,
                                      bool preparingForBatch,
                                      unsigned long int wantCS)
{
  std::string baseName = part->getBaseFTLName();
  if (baseName.empty())
    return "===> Logic error, base name not set for " + part->getIdString();

  std::string partPath = part->getAbsFilePath(true);
  if (partPath.empty())
    return "===> Failed to create input directory for " + part->getIdString();

  FmResultStatusData& partRSD = part->myRSD.getValue();
  partRSD.setTaskName(baseName);
  if (!analysis->overwriteFEParts.getValue() && !partRSD.isEmpty())
    // Create an empty part RSD for this part
    partRSD.setTaskVer(FmFileSys::getNextDirIncrement(partPath,baseName));

  std::string rdbPath = partRSD.getCurrentTaskDirName(false,true);
  FFaFilePath::makeItAbsolute(rdbPath,partPath);
  if (FmFileSys::verifyDirectory(rdbPath))
    rdbPath += FFaFilePath::getPathSeparator();
  else
    return "===> Could not access directory " + rdbPath;

  // Calculation options
  FFaOptionFileCreator fcoArgs(rdbPath + solverName + ".fco");

  // Check for memory setting on this part:
  // - If in memory, use old method,
  // - not in memory, use reference to base part file (assumed it is up to date)

  if (part->isFELoaded(true))
  {
    part->reducedFTLFile.setValue(baseName + ".ftl");
    part->exportPart(rdbPath+part->reducedFTLFile.getValue(),true,false,true);
    fcoArgs.add("-linkfile",part->reducedFTLFile.getValue());
  }
  else
  {
    FFaFilePath::appendToPath(partPath,part->baseFTLFile.getValue());
    if (!FmFileSys::isReadable(partPath))
      return "===> Could not access FE data file " + partPath +
             "\n     You must either Save the model first or"
             " switch the FE-Data Settings to \"Loaded\" for this part.\n";

    fcoArgs.add("-linkfile",FFaFilePath::getRelativeFilename(rdbPath,partPath));

    // Get all external FE nodes
    std::string extNodes;
    std::vector<FmTriad*> localTriads;
    part->getTriads(localTriads);
    for (FmTriad* triad : localTriads)
      if (extNodes.empty())
        extNodes = FFaNumStr(triad->FENodeNo.getValue());
      else
        extNodes += ", " + FFaNumStr(triad->FENodeNo.getValue());
    fcoArgs.add("-extNodes", "<" + extNodes + ">");
  }

  // Memory settings for the recovery matrix and equation solver
  if (!analysis->useRamSizeBmat.getValue())
    fcoArgs.add("-Bramsize",0);
  else if (!analysis->autoRamSizeBmat.getValue())
    fcoArgs.add("-Bramsize",analysis->ramSizeBmat.getValue());
  const std::string& addOptions = analysis->reducerAddOpts.getValue();
  if (analysis->useRamSizeGSF.getValue())
  {
    // Use the out-of-core GSF equation solver
    if (addOptions.find("-gsfSolver") == std::string::npos)
      fcoArgs.add("-gsfSolver",2);
    if (analysis->autoRamSizeGSF.getValue())
      fcoArgs.add("-cachesize",0);
    else if (analysis->ramSizeGSF.getValue() > 0)
      fcoArgs.add("-cachesize",analysis->ramSizeGSF.getValue());
    // else switch off out-of-core for GSF
  }
  // else use the SPR solver (default)

  int ngen = part->nGenModes.getValue();
  fcoArgs.add("-neval",        part->nEigvalsCalc.getValue());
  fcoArgs.add("-ngen",         ngen > 0 ? ngen : 0);
  fcoArgs.add("-tolEigval",    part->tolEigenval.getValue());
  fcoArgs.add("-tolFactorize", part->tolFactorize.getValue());
  fcoArgs.add("-lumpedmass",  !part->useConsistentMassMatrix.getValue());
  fcoArgs.add("-factorMass",   part->factorizeMassMxEigSol.getValue());

  if (part->useNonlinearReduction.getValue())
  {
    // Nonlinear reduction options
    std::string cfemPath = part->nonlinearDataFileName.getValue();
    FFaFilePath::makeItAbsolute(cfemPath,mech->getAbsModelFilePath());
    fcoArgs.add("-CfemFile",FFaFilePath::getRelativeFilename(rdbPath,cfemPath));
    fcoArgs.add("-numCfemSolutions",part->numberOfNonlinearSolutions.getValue());
  }

  fcoArgs.writeOptFile();

  // Output options
  FFaOptionFileCreator fopArgs(rdbPath + solverName + ".fop");
  if (part->expandModeShapes.getValue())
    fopArgs.add("-linkId",  part->getBaseID());
  fopArgs.add("-Bmatfile",  baseName + "_B.fmx");
  fopArgs.add("-Bmatprecision", (int)part->recoveryMatrixSavePrecision.getValue());
  if (ngen > 0)
    fopArgs.add("-eigfile", baseName + "_E.fmx");
  fopArgs.add("-gravfile",  baseName + "_G.fmx");
  if (part->hasLoads())
    fopArgs.add("-loadfile",baseName + "_L.fmx");
  fopArgs.add("-samfile",   baseName + "_SAM.fsm");
  fopArgs.add("-stiffile",  baseName + "_S.fmx");
  fopArgs.add("-massfile",  baseName + "_M.fmx");
  if (part->useNonlinearReduction.getValue())
  {
    fopArgs.add("-dispfile", baseName + "_D.fmx");
    fopArgs.add("-forcefile", baseName + "_F.fmx");
    fopArgs.add("-numStatesFile", baseName + "_numStates.txt");
  }
  else if (ngen < 0)
    fopArgs.add("-dispfile", baseName + "_D.fmx");
  if (part->expandModeShapes.getValue())
    fopArgs.add("-frsFile", baseName + ".frs");
  fopArgs.add("-resfile",   solverName + ".res");
  fopArgs.writeOptFile();

  // Additional options, if any
  if (!addOptions.empty())
  {
    FFaOptionFileCreator faoArgs(rdbPath + solverName + ".fao");
    faoArgs.addComment("Additional user defined options to " + solverName);
    faoArgs.add(addOptions,"",false);
    faoArgs.writeOptFile();
  }

  if (preparingForBatch && wantCS > 0)
  {
    // Store the part checksum in a temporary file
    std::string fileName = rdbPath + baseName + ".chk";
    FILE* fp = fopen(fileName.c_str(),"wb");
    FFaTag::write(fp,"#FEDEM link checksum",20,wantCS);
    fclose(fp);

    // Set file names supposed to be generated by the batch execution.
    // This needs to be done here, such the dynamics solver input file is
    // generated correctly by the "Prepare for batch execution" command.
    part->SMatFile = baseName + "_S.fmx";
    if (analysis->needMassMatrix())
      part->MMatFile = baseName + "_M.fmx";
    if (FmDB::getGrav().length() > 1.0e-8)
      part->GMatFile = baseName + "_G.fmx";
    if (part->hasLoads())
      part->LMatFile = baseName + "_L.fmx";
    part->SAMdataFile = baseName + "_SAM.fsm";
    part->BMatFile = baseName + "_B.fmx";
    if (ngen > 0)
      part->EMatFile = baseName + "_E.fmx";
    if (part->useNonlinearReduction.getValue())
    {
      part->DMatFile = baseName + "_D.fmx";
      part->FMatFile = baseName + "_F.fmx";
    }
    else if (ngen < 0)
      part->DMatFile = baseName + "_D.fmx";
  }

  return rdbPath;
}


std::string Fedem::createSolverInput(FmAnalysis* analysis,
                                     FmMechanism* mech,
                                     FmSimulationEvent* sev,
                                     const std::string& solverName,
                                     const std::vector<std::string>& plugins,
                                     std::vector<std::string>& rdbPath,
                                     bool preparingForBatch, bool keepOldRes)
{
  FmResultStatusData* topRSD = sev ? sev->getResultStatusData() : mech->getResultStatusData();
  if (topRSD->getTaskName() == "noname") topRSD->setTaskName("response");

  // Add sub-directories for time history, eigenvalues
  // and frequency domain simulation
  FmResultStatusData* th1RSD = topRSD->addSubTask("timehist_prim");
  FmResultStatusData* th2RSD = topRSD->addSubTask("timehist_sec");
  FmResultStatusData* eigRSD = NULL;
  FmResultStatusData* freqRSD = NULL;
  if (analysis->solveEigenvalues.getValue())
    eigRSD = topRSD->addSubTask("eigval");
  if (analysis->solveFrequencyDomain.getValue())
    freqRSD = topRSD->addSubTask("freqdomain");

  // Make sure the disk is clean if the solver RSD is empty
  std::string resFile = solverName + ".res";
  std::string mainPath = topRSD->getCurrentTaskDirName(true);
  if (FmFileSys::isDirectory(mainPath))
  {
    keepOldRes &= FmFileSys::isFile(FFaFilePath::appendFileNameToPath(mainPath,resFile));
    if (topRSD->isEmpty() && FmFileSys::getDirs(rdbPath,mainPath,NULL,true))
      for (const std::string& path : rdbPath)
      {
        ListUI <<" ==> Clearing existing RDB-folder "<< path;
        int nDelete = FmFileSys::removeDir(path);
        if (nDelete > 0)
          ListUI <<" with "<< nDelete <<" file(s)\n";
        else if (nDelete < 0)
          ListUI <<" failed\n";
        else
          ListUI <<"\n";
      }
  }
  else
    keepOldRes = false;

  // Create the RDB paths
  rdbPath.clear();
  rdbPath.reserve(6);
  rdbPath.push_back(topRSD->getPath());
  rdbPath.push_back(mainPath);
  rdbPath.push_back(th1RSD->getCurrentTaskDirName(true));
  rdbPath.push_back(th2RSD->getCurrentTaskDirName(true));
  if (eigRSD)
    rdbPath.push_back(eigRSD->getCurrentTaskDirName(true));
  if (freqRSD)
    rdbPath.push_back(freqRSD->getCurrentTaskDirName(true));

  for (std::string& path : rdbPath)
    if (FmFileSys::verifyDirectory(path))
      path += FFaFilePath::getPathSeparator();
    else
      return "===> Could not access directory " + path;

  rdbPath.erase(rdbPath.begin());
  std::string currentSolve(solverName), restartFiles;

  // Check for restart run
  int restartNo = 0;
  bool restart = analysis->doRestart.getValue() && !th1RSD->isEmpty() && !th2RSD->isEmpty();
  if (restart)
  {
    restartNo = FmFileSys::getNextIncrement(mainPath,"res");
    currentSolve += FFaNumStr("_%d",restartNo);
    resFile = currentSolve + ".res";

    // Find paths to frs-files to restart from
    std::string th1Path = th1RSD->getCurrentTaskDirName();
    std::string th2Path = th2RSD->getCurrentTaskDirName();
    const std::set<std::string>& th1Files = th1RSD->getFileSet();
    const std::set<std::string>& th2Files = th2RSD->getFileSet();
    std::set<std::string>::const_iterator it = th1Files.begin();
    restartFiles = "<\"" + FFaFilePath::appendFileNameToPath(th1Path,*it);
    for (++it; it != th1Files.end(); ++it)
      restartFiles += "\",\"" + FFaFilePath::appendFileNameToPath(th1Path,*it);
    for (it = th2Files.begin(); it != th2Files.end(); ++it)
      restartFiles += "\",\"" + FFaFilePath::appendFileNameToPath(th2Path,*it);
    restartFiles += "\">";
  }
  else if (keepOldRes)
  {
    // We don't want to overwrite any existing res-files.
    // This will be the case when doing micro-batches in the cloud.
    // To be able to extract res-files from not only the last time window.
    int res = FmFileSys::getNextIncrement(mainPath,"res");
    resFile = solverName + FFaNumStr("_%d",res) + ".res";
  }

  // Find the next increment for frs-file to avoid overwriting existing files
  std::vector<std::string> searchForIncrement(rdbPath.begin()+1,rdbPath.end());
  int increment = FmFileSys::getNextIncrement(searchForIncrement,"frs");

  // Current model input files (not updated in restart runs)
  mainPath += FFaFilePath::getPathSeparator();
  std::string fmmName = mainPath + topRSD->getTaskName() + ".bak.fmm";
  std::string fsiName = mainPath + solverName + ".fsi";

  if (!restart || !FmFileSys::isReadable(fmmName))
  {
    // Write fmm file - used for backup
    std::ofstream s(fmmName.c_str(),std::ios::out);
    if (!s) return "===> Could not write fmm backup file.";
    FmSubAssembly::mainFilePath = mainPath;
    FmDB::reportAll(s,false);
    s.close();
  }
  if (!restart || !FmFileSys::isReadable(fsiName))
  {
    // Write fsi file - fedem solver model input
    FmSolverParser solverParser(fsiName.c_str());

    // Path correction
    std::string relPath = FFaFilePath::getRelativeFilename(mainPath,mech->getAbsModelFilePath());
#if FM_DEBUG > 1
    std::cout <<"\trdbPath:\t"<< mainPath
              <<"\n\tmodelFilePath:\t"<< mech->getAbsModelFilePath()
              <<"\n\trelative path:\t"<< relPath << std::endl;
#endif

    solverParser.setRDBPath(mainPath);
    solverParser.setRelPathCorrection(relPath);
    if (solverParser.writeFullFile() < 0)
      return "===> Could not write solver input file\n     " + fsiName;
  }

  // Calculation options
  FFaOptionFileCreator fcoArgs(mainPath + currentSolve + ".fco");
  fcoArgs.add("-fsifile", FFaFilePath::getRelativeFilename(mainPath,fsiName));

  // Plugin libraries (user-defined functions, etc.)
  if (plugins.size() == 1)
    fcoArgs.add("-plugin", plugins.front());
  else if (!plugins.empty())
  {
    std::string fileList = "<\"" + plugins.front();
    for (size_t i = 1; i < plugins.size(); i++)
      fileList += "\",\"" + plugins[i];
    fileList += "\">";
    fcoArgs.add("-plugin", fileList);
  }

  fcoArgs.addComment("Initial static equilibrium parameters");
  fcoArgs.add("-initEquilibrium" , analysis->solveInitEquil.getValue());
  fcoArgs.add("-tolInitEquil"    , analysis->staticEqlTol.getValue());
  fcoArgs.add("-limInitEquilStep", analysis->iterStepLimit.getValue());
  fcoArgs.add("-stressStiffEqu"  , analysis->useEquStressStiffening.getValue());

  if (analysis->smoothRamp.getValue()) {
    fcoArgs.addComment("Dynamic ramp-up parameters");
    fcoArgs.add("-rampSteps", analysis->rampSteps.getValue());
    fcoArgs.add("-rampGravity", analysis->rampGrav.getValue());
    fcoArgs.add("-rampData", DoubleVec({
          analysis->rampVmax.getValue(),
          analysis->rampLength.getValue(),
          analysis->rampPause.getValue()
        }));
  }

  fcoArgs.addComment("Time integration parameters");
  fcoArgs.add("-timeStart", analysis->startTime.getValue());
  if (!analysis->dynamicsEnable.getValue())
    fcoArgs.add("-timeEnd", analysis->startTime.getValue());
  else if (!analysis->stopTimeEnable.getValue())
    fcoArgs.add("-timeEnd", analysis->startTime.getValue()-1.0);
  else
    fcoArgs.add("-timeEnd", analysis->stopTime.getValue());
  fcoArgs.add("-timeInc"  , analysis->timeIncr.getValue());
  if (analysis->getEngine() || analysis->doCutback.getValue())
    fcoArgs.add("-minInc" , analysis->minTimeIncr.getValue());

  switch (analysis->newmarkDamping.getValue()) {
  case FmAnalysis::HHT_ALPHA:
    fcoArgs.add("-alphaNewmark", analysis->newmarkFactors.getValue().first);
    break;
  case FmAnalysis::GENERALIZED_ALPHA:
    fcoArgs.add("-NewmarkFlag", 200);
    fcoArgs.add("-alphaNewmark", analysis->newmarkFactors.getValue());
    break;
  default:
    fcoArgs.add("-alphaNewmark", 0.0);
  }

  if (analysis->quasistaticEnable.getValue())
  {
    if (analysis->quasistaticMode.getValue())
      fcoArgs.add("-quasiStatic", analysis->quasistaticUpToTime.getValue());
    else
      fcoArgs.add("-quasiStatic", analysis->stopTime.getValue());
  }

  if (analysis->doCutback.getValue())
  {
    fcoArgs.add("-cutbackFactor", analysis->cutbackFactor.getValue());
    fcoArgs.add("-cutbackSteps" , analysis->cutbackSteps.getValue());
  }

  if (restart)
  {
    fcoArgs.add("-restarttime", analysis->restartTime.getValue());
    fcoArgs.add("-restartfile", restartFiles);
  }

  fcoArgs.addComment("Newton-Raphson iteration parameters");
  fcoArgs.add("-stressStiffDyn", analysis->useDynStressStiffening.getValue());
  fcoArgs.add("-centripForceCorr", analysis->useMassCorrection.getValue());

  fcoArgs.add("-nupdat", analysis->minMatrixUpdates.getValue());
  fcoArgs.add("-maxit" , analysis->maxNumIt.getValue());
  fcoArgs.add("-minit" , analysis->minNumIt.getValue());
  fcoArgs.add("-maxSeqNoUpdate", analysis->maxSequentialNoMatrixUpdates.getValue());
  if (analysis->ignoreTolerance.getValue())
    fcoArgs.add("-numit", analysis->fixedNumIt.getValue());

  if (!analysis->useFixedMatrixUpdates.getValue())
    fcoArgs.add("-tolUpdateFactor", analysis->tolMatrixUpdateFactor.getValue());

  if (analysis->tolVelProp.getValue() > 0.0)
    fcoArgs.add("-tolVelProp", analysis->tolVelProp.getValue());

  // Lambda function printing the convergence criteria values
  auto&& tol_print = [&fcoArgs](const char* text, const FFaField<FmSolverConvergence>& tol)
  {
    switch (tol.getValue().policy) {
    case FmSolverConvergence::CONV_ALL_OF:
      fcoArgs.add(text,tol.getValue().value);
      break;
    case FmSolverConvergence::CONV_ONE_OF:
      fcoArgs.add(text,-tol.getValue().value);
      break;
    default:
      break;
    }
  };

  tol_print("-tolDispNorm",analysis->tolDisplacementNorm);
  tol_print("-tolDispTra", analysis->tolDisplacementTra);
  tol_print("-tolDispRot", analysis->tolDisplacementRot);
  tol_print("-tolVelNorm", analysis->tolVelocityNorm);
  tol_print("-tolResNorm", analysis->tolResidualNorm);
  tol_print("-tolResTra",  analysis->tolResidualTra);
  tol_print("-tolResRot",  analysis->tolResidualRot);
  tol_print("-tolEnerMax", analysis->tolEnergyMax);
  tol_print("-tolEnerSum", analysis->tolEnergySum);

  if (analysis->solveEigenvalues.getValue())
  {
    fcoArgs.addComment("Eigenvalue solution parameters");
    if (analysis->dynamicsEnable.getValue())
      fcoArgs.add("-eiginc",       analysis->eigenSolveTimeInterval.getValue());
    fcoArgs.add("-numEigModes",    analysis->numEigenmodes.getValue());
    fcoArgs.add("-damped",         analysis->dampedEigenvalues.getValue());
    fcoArgs.add("-eigenshift",     analysis->eigenvalueShiftFactor.getValue());
    fcoArgs.add("-addBC_eigensolver", analysis->useBCsOnEigenvalues.getValue());
    fcoArgs.add("-stressStiffEig", analysis->useEigStressStiffening.getValue());
  }

  if (analysis->solveFrequencyDomain.getValue())
  {
    fcoArgs.addComment("Frequency domain solution parameters");
    fcoArgs.add("-frequency_domain", true);
    if (analysis->dynamicsEnable.getValue() && !analysis->solveEigenvalues.getValue())
      fcoArgs.add("-eiginc", analysis->eigenSolveTimeInterval.getValue());
  }

  // Get unit conversion options
  if (mech->modelDatabaseUnits.getValue().isValid())
  {
    fcoArgs.addComment("Unit mapping options");
    double scale = 1.0;
    if (mech->modelDatabaseUnits.getValue().convert(scale, "MASS"))
      fcoArgs.add("-scaleToKG", scale);
    scale = 1.0;
    if (mech->modelDatabaseUnits.getValue().convert(scale, "LENGTH"))
      fcoArgs.add("-scaleToM", scale);
    scale = 1.0;
    if (mech->modelDatabaseUnits.getValue().convert(scale, "TIME"))
      fcoArgs.add("-scaleToS", scale);
  }

  if (analysis->useExternalFuncFile.getValue())
  {
    fcoArgs.addComment("File to read external function values from");
    std::string fileName = analysis->externalFuncFileName.getValue();
    FFaFilePath::makeItAbsolute(fileName,mech->getAbsModelFilePath());
    fcoArgs.add("-externalfuncfile", FFaFilePath::getRelativeFilename(mainPath,fileName));
  }

  // Lambda function returning full existing path to a recovery frs-file
  auto&& getFrsPath = [topRSD](const std::string& taskName, FmPart* part)
  {
    FmResultStatusData* recRSD = topRSD->addSubTask(taskName);
    FmResultStatusData* lnkRSD = recRSD ? recRSD->addSubTask(part->getTaskName()) : NULL;

    std::string path = topRSD->getCurrentTaskDirName(true);
    if (!FmFileSys::verifyDirectory(topRSD->getPath()) || !FmFileSys::verifyDirectory(path))
      return "===> Could not access directory " + path;

    FFaFilePath::appendToPath(path, recRSD ? recRSD->getCurrentTaskDirName() : taskName);
    if (!recRSD || !FmFileSys::verifyDirectory(path))
      return "===> Could not access directory " + path;

    FFaFilePath::appendToPath(path, lnkRSD ? lnkRSD->getCurrentTaskDirName() : part->getTaskName());
    if (!lnkRSD || !FmFileSys::verifyDirectory(path))
      return "===> Could not access directory " + path;

    return FFaFilePath::appendFileNameToPath(path,"th_s.frs");
  };

  const std::string& addOptions = analysis->solverAddOpts.getValue();

  // Check if frs-output of recovery results has been enabled
  // through the additional solver options
  size_t iStr = addOptions.rfind("-partDeformation");
  bool saveStr = iStr == std::string::npos; // use "==" here since default value is 1
  if (!saveStr && iStr+17 < addOptions.size())
    if (!(saveStr = atoi(addOptions.substr(iStr+17).c_str()) > 0))
    {
      iStr = addOptions.rfind("-partVMStress");
      saveStr = iStr == std::string::npos; // use "==" here since default value is 1
      if (!saveStr && iStr+14 < addOptions.size())
        saveStr = atoi(addOptions.substr(iStr+14).c_str())%2 > 0;
    }

  iStr = addOptions.rfind("-allGages");
  bool saveGages = iStr != std::string::npos; // use "!=" here since default is "-"
  if (saveGages && iStr+9 < addOptions.size())
    saveGages = addOptions[iStr+9] == ' ' || addOptions[iStr+9] == '+';

  // Check for stress recovery in the loop
  int doRecovery = 0;
  std::vector<std::string> frsNames;
  std::vector<FmPart*> allParts;
  FmDB::getAllParts(allParts);
  for (FmPart* part : allParts)
    if (part->recoveryDuringSolve.getValue()%2 > 0)
    {
      if (saveStr)
      {
        std::string frsFile = getFrsPath("timehist_rcy",part);
        if (frsFile.find("===> Could not") == 0)
          return frsFile;
        frsNames.push_back(FFaFilePath::getRelativeFilename(mainPath,frsFile));
      }
      doRecovery |= 1;
    }

  // Check for gage recovery in the loop
  for (FmPart* part : allParts)
    if (part->recoveryDuringSolve.getValue() > 1)
    {
      if (saveGages)
      {
        std::string frsFile = getFrsPath("timehist_gage_rcy",part);
        if (frsFile.find("===> Could not") == 0)
          return frsFile;
        frsNames.push_back(FFaFilePath::getRelativeFilename(mainPath,frsFile));
      }
      doRecovery |= 2;
    }

  if (doRecovery)
  {
    fcoArgs.addComment("Recovery parameters");
    fcoArgs.add("-recovery", doRecovery);
  }

  fcoArgs.writeOptFile();

  // Lambda function for creating an event-dependent output file name
  auto&& eventName = [sev,mech](const std::string& fileName)
  {
    if (sev) return sev->eventName(fileName);

    std::string newName(fileName);
    return FFaFilePath::makeItAbsolute(newName,mech->getAbsModelFilePath());
  };

  // Output options
  FFaOptionFileCreator fopArgs(mainPath + currentSolve + ".fop");
  fopArgs.add("-frs1file" , FFaFilePath::getRelativeFilename(mainPath, rdbPath[1] + std::string("th_p.frs")));
  fopArgs.add("-frs2file" , FFaFilePath::getRelativeFilename(mainPath, rdbPath[2] + std::string("th_s.frs")));
  fopArgs.add("-ctrlfile" , FFaFilePath::getRelativeFilename(mainPath, rdbPath[2] + std::string("ctrl.frs")));
  if (rdbPath.size() > 3 && eigRSD)
    fopArgs.add("-modesfile", FFaFilePath::getRelativeFilename(mainPath, rdbPath[3] + std::string("ev_p.frs")));
  if (rdbPath.size() > 3 && freqRSD)
    fopArgs.add("-freqfile", FFaFilePath::getRelativeFilename(mainPath, rdbPath.back() + std::string("fd_p.frs")));
  fopArgs.add("-resfile"  , resFile);
  fopArgs.add("-rdbinc"   , increment);

  if (analysis->autoSolverVTFExport.getValue())
  {
    std::string vtfFile = eventName(analysis->solverVTFname.getValue());
    fopArgs.add("-VTFfile", FFaFilePath::getRelativeFilename(mainPath,vtfFile));
  }

  // Add output file(s) for stress recovery, if any
  if (frsNames.size() == 1)
    fopArgs.add("-frs3file", frsNames.front());
  else if (!frsNames.empty())
  {
    std::string frsFiles = "<\"" + frsNames.front();
    for (size_t i = 1; i < frsNames.size(); i++)
      frsFiles += "\",\"" + frsNames[i];
    frsFiles += "\">";
    fopArgs.add("-frs3file", frsFiles);
  }

  if (analysis->autoCurveExportSwitch.getValue())
  {
    std::string expName = eventName(analysis->autoCurveExportFileName.getValue());
    fopArgs.addComment("Curve auto-export parameters");
    fopArgs.add("-curveFile",FFaFilePath::getRelativeFilename(mainPath,fmmName));
    fopArgs.add("-curvePlotFile",FFaFilePath::getRelativeFilename(mainPath,expName));
    switch (analysis->autoCurveExportFileFormat.getValue())
      {
      case FmAnalysis::RPC_LITTLE_ENDIAN:
        fopArgs.add("-curvePlotType",3);
        break;
      case FmAnalysis::RPC_BIG_ENDIAN:
        fopArgs.add("-curvePlotType",4);
        break;
      case FmAnalysis::ASCII_MULTI_COLUMN:
        fopArgs.add("-curvePlotType",5);
        if (addOptions.find("-curvePlotPrec") == std::string::npos)
          fopArgs.add("-curvePlotPrec",1); // Single precision output as default
        break;
      }
  }
  fopArgs.writeOptFile();

  if (preparingForBatch)
  {
    // Create empty solver frs-files such that the RSD is ready for batch recovery setup
    std::string f1 = rdbPath[1] + FFaNumStr("th_p_%d.frs",increment);
    std::string f2 = rdbPath[2] + FFaNumStr("th_s_%d.frs",increment+1);
    std::ofstream s1(f1.c_str(),std::ios::out);
    std::ofstream s2(f2.c_str(),std::ios::out);
    if (rdbPath.size() > 3 && eigRSD)
    {
      std::string f3 = rdbPath[3] + FFaNumStr("ev_p_%d.frs",increment+2);
      std::ofstream s3(f3.c_str(),std::ios::out);
    }
    if (rdbPath.size() > 3 && freqRSD)
    {
      std::string f4 = rdbPath.back() + FFaNumStr("fd_p_%d.frs",increment+3);
      std::ofstream s4(f4.c_str(),std::ios::out);
    }
  }

  // Additional options
  if (!addOptions.empty())
  {
    FFaOptionFileCreator faoArgs(mainPath + currentSolve + ".fao");
    faoArgs.addComment("Additional user-defined options to " + solverName);
    faoArgs.add(addOptions,"",false);
    faoArgs.writeOptFile();
  }

  return currentSolve;
}
