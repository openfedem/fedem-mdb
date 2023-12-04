// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmModelLoader.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFileSys.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmStrainRosette.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


int Fedem::loadTemplate (const std::string& newName,
                         const std::string& defaultName,
                         bool writeLogFile)
{
  // Get the template model file name
  const char* envTemplateFileName = getenv("FEDEM_TEMPLATE_FILE");
  std::string tName = envTemplateFileName ? envTemplateFileName : defaultName;
  if (tName.empty())
    return 0; // Silently ignore if no template specified

  if (!FmFileSys::isFile(tName))
  {
    ListUI <<"Warning : Could not open template file "<< tName <<"\n"
           <<"          Check that the environment variable FEDEM_TEMPLATE_FILE"
           <<" is set correctly.\n";
    return 0;
  }

  // Check that the directory of the given model file exists, create it if not
  if (!FmFileSys::verifyDirectory(FFaFilePath::getPath(newName)))
    return -1;

  // Open a temporary log-file for a copy of the Output List messages
  if (writeLogFile) FFaMsg::getMessager().openListFile();

  ListUI <<"===> Reading template file: "<< tName <<"\n";
  FFaMsg::pushStatus("Reading template");
  FmDB::readAll(tName,true); // ignoring template file version
  FFaMsg::popStatus();

  // Update to make model point to the actual place to save, etc.
  FmMechanism* mech = FmDB::getMechanismObject();
  mech->syncPath(newName, newName.find("untitled") == std::string::npos);

  // Check if a blade folder exist for this model-file, remove it if so
  const std::string& bladeFolderPath = mech->getAbsBladeFolderPath();
  if (FmFileSys::isDirectory(bladeFolderPath))
  {
    ListUI << "===> Blade directory "<< bladeFolderPath
           <<" alread exists, and is deleted.\n";
    FmFileSys::removeDir(bladeFolderPath);
  }

  return 1;
}


int Fedem::loadModel (const std::string& name,
                      const std::string& logName,
                      char ignoreFileVersion)
{
  if (!logName.empty())
  {
    // Open a log-file for a copy of the Output List messages
    std::string logFileName = FFaFilePath::getBaseName(logName) + ".log";
    FFaMsg::getMessager().openListFile(logFileName.c_str());
  }

  // Check for existence of the given model file
  bool existingFile = FmFileSys::isFile(name);
  FmMechanism* mech = NULL;
  if (existingFile)
    mech = FmDB::getMechanismObject();
  else
    mech = FmDB::newMechanism();

  // Set up all model and RDB file names
  mech->syncPath(name,!existingFile);

  if (existingFile)
  {
    ListUI <<"===> Reading "<< name <<"\n";
    FFaMsg::pushStatus("Reading mechanism");
    bool status = FmDB::readAll(name,ignoreFileVersion);
    FFaMsg::popStatus();
    if (!status) {
      ListUI <<"===> Reading model file failed.\n\n";
      return -1;
    }
  }
  else // A new mechanism entry was created
    ListUI <<"===> New model with name: "<< name <<"\n";

  // If the current model has a blade-design associated with it,
  // check if the blade-design path is in the blade-folder of the model.
  // If not, the model has been moved, and we must update the blade-design path.
  FmTurbine* pTurbine = FmDB::getTurbineObject();
  FmBladeDesign* pDesign = pTurbine ? dynamic_cast<FmBladeDesign*>(pTurbine->bladeDef.getPointer()) : NULL;
  if (pDesign)
  {
    std::string bladeFile   = pDesign->getModelFileName();
    std::string bladeFolder = mech->getAbsBladeFolderPath();
    if (FFaFilePath::getPath(bladeFile,false) != bladeFolder)
    {
      // Update path if there actually is a blade with the correct name
      // located in the blade-folder
      std::string newPath = FFaFilePath::appendFileNameToPath(bladeFolder,FFaFilePath::getFileName(bladeFile));
      if (FmFileSys::isFile(newPath))
      {
        ListUI <<" ==> New blade path: "<< newPath <<"\n";
        pDesign->myModelFile.setValue(newPath);
        pDesign->writeToFMM(newPath);
      }
    }
  }

  return existingFile;
}


bool Fedem::loadParts(bool forceLoad)
{
  FmMechanism* mech = FmDB::getMechanismObject(false);
  if (!mech)
  {
    ListUI <<"\n\n===> Empty model.\n";
    return false;
  }

  // Check availability of model part repository, if used
  std::string mlr = mech->modelLinkRepository.getValue();
  if (!mlr.empty())
  {
    FFaFilePath::checkName(mlr);
    FFaFilePath::makeItAbsolute(mlr,mech->getAbsModelFilePath());
    if (!FmFileSys::isDirectory(mlr))
    {
      // The model part repository specified in the model file
      // does not exist, check if provided via environment variable
      const char* fedemLDB = getenv("FEDEM_LDB");
      if (fedemLDB && FmFileSys::isDirectory(fedemLDB) && mlr != fedemLDB)
      {
        ListUI <<"Note: Changing FE part repository of this model"
               <<"\n      from "<< mech->modelLinkRepository.getValue()
               <<"\n        to "<< fedemLDB <<"\n";
        mech->modelLinkRepository.setValue(fedemLDB);
      }
      else
      {
        ListUI <<"\n\n===> Could not locate FE model repository\n     "<< mlr
               <<"\n     Open the model in the Fedem GUI to resolve this.\n";
        return false;
      }
    }
  }

  std::vector<FmPart*> allParts;
  if (forceLoad)
    FmDB::getAllParts(allParts);
  else
  {
    FmDB::getUnsavedParts(allParts);
    if (!allParts.empty())
      ListUI <<"\n\n===> Some files could not be found in the FE model repository.\n"
             <<"     Forcing re-import of FE model data for the unsaved parts.\n"
             <<"     This might take a while for big models...\n";
  }

  if (allParts.empty())
    return true;

  FFaMsg::list("===> Reading FE parts\n");
  FFaMsg::pushStatus("Loading FE/Cad data");
  FFaMsg::enableSubSteps(allParts.size());
  std::vector<std::string> erroneousParts;

  // Actually load the FE data
  int partNr = 0;
  for (FmPart* part : allParts)
  {
    FFaMsg::setSubStep(++partNr);

    // Load FE data if it is an FE part. If it is a generic part, use
    // the visualization file if it exists. If not, use the CAD visualization.
    // If that is not present either, use the FE data.

    bool loadFEdata = false;
    bool loadCadData = false;
    if (!part->useGenericProperties.getValue())
      loadFEdata = true;
    else if (part->visDataFile.getValue().empty())
    {
      if (!part->baseCadFileName.getValue().empty())
        loadCadData = true;
      else if (!part->baseFTLFile.getValue().empty())
        loadFEdata = true;
    }

    if (loadFEdata)
    {
      if (!part->openFEData())
        erroneousParts.push_back(part->getLinkIDString());
      else if (!part->useGenericProperties.getValue())
        part->lockLevel.setValue(FmPart::FM_DENY_LINK_USAGE);
    }
    else if (loadCadData)
    {
      if (!part->openCadData())
        erroneousParts.push_back(part->getLinkIDString());
    }

    part->updateTriadTopologyRefs(true,1);
  }

  FFaMsg::disableSubSteps();
  FFaMsg::setSubTask("");
  FFaMsg::popStatus();

  // Now that all FE data is loaded we can syncronize the strain rosettes
  FmStrainRosette::syncStrainRosettes();

  // Syncronize the FE part RSD with actual contents on disk
  FmDB::getFEParts(allParts);
  for (FmPart* part : allParts)
    part->syncRSD();

  if (erroneousParts.empty())
    return true;

  size_t nMax = 30; // To limit the size of the message box
  std::string msg = "The following parts could not be loaded due to errors in their\n"
    "respective FE data files (see output list for details):";
  for (size_t i = 0; i < erroneousParts.size() && i < nMax; i++)
    msg += "\n\t" + erroneousParts[i];
  if (erroneousParts.size() > nMax)
    msg += "\n\t...";

  FFaMsg::list("===> FE model loading failed.\n\n",true);
  FFaMsg::dialog(msg,FFaMsg::ERROR);
  return false;
}
