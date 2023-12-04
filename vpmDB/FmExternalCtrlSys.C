// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <fstream>

#include "vpmDB/FmExternalCtrlSys.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmMechanism.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"


Fmd_DB_SOURCE_INIT(FcEXTERNAL_CTRL_SYSTEM, FmExternalCtrlSys, FmIsMeasuredBase);


FmExternalCtrlSys::FmExternalCtrlSys()
{
  Fmd_CONSTRUCTOR_INIT(FmExternalCtrlSys);

  FFA_FIELD_DEFAULT_INIT(myFilePath,      "FILE");
  FFA_FIELD_DEFAULT_INIT(myFromWorkspace, "FROM_WORKSPACE");
  FFA_FIELD_DEFAULT_INIT(myToWorkspace,   "TO_WORKSPACE");

  FFA_REFERENCELIST_FIELD_INIT(myEnginesField, myEngines, "ENGINES");
  myEngines.setAutoSizing(false);

  inputFileHasChanged = false;
}

//---------------------------------------------------

FmExternalCtrlSys::~FmExternalCtrlSys()
{
  this->disconnect();
}

//---------------------------------------------------

void FmExternalCtrlSys::setFilePath(const std::string& path)
{
  if (!myFilePath.setValue(path)) return;

  inputFileHasChanged = true;
  this->setUserDescription(FFaFilePath::getBaseName(path,true));
  this->readSimulinkFile();
}

//---------------------------------------------------

bool FmExternalCtrlSys::readSimulinkFile()
{
  std::string file = myFilePath.getValue();
  FFaFilePath::makeItAbsolute(file,FmDB::getMechanismObject()->getAbsModelFilePath());

  // construct an ifstream input file object
  std::ifstream fileStream(file.c_str());
  if (!fileStream) {
    fileError = "Can not open the Simulink file " + file;
    return false;
  }

  fileStream.seekg(0,std::ios::beg);

  std::string buffer, blockType;
  bool reachedSystem = false;

  // tmp vectors
  Strings fromWS, toWS;
  myToWorkspace.setValue(toWS);
  myFromWorkspace.setValue(fromWS);

  // Runs as long as the end of file hasn't been reached
  while (!fileStream.eof()) {
    // Reading one word at a time
    fileStream >> buffer;
    if (buffer == "System")
      reachedSystem = true;

    if (reachedSystem && buffer == "BlockType") {
      fileStream >> blockType;
      if (blockType == "FromWorkspace") {

	while (blockType != "VariableName" && blockType != "}")
	  fileStream >> blockType;

	if (blockType == "VariableName") {
	  fileStream >> blockType;
	  // erasing first and last char(of type ")
	  blockType.erase(0,1);
	  blockType.erase(blockType.size()-1,1);
	  // increasing size of fromWorkspace with 1,and inserting
	  // the variable name of the block used in simulink
	  //myFromWorkspace.push_back(blockType.c_str());
	  fromWS.push_back(blockType);
	}
      }
      if (blockType == "ToWorkspace") {

	while (blockType != "VariableName" && blockType != "}")
	  fileStream >> blockType;
	
	if (blockType == "VariableName") {
	  // getting the name of the variable
	  fileStream >> blockType;
	  // erase(where to begin,number of chars to erase)
	  // erasing first and last char(of type ")
	  blockType.erase(0,1);
	  blockType.erase(blockType.size()-1,1);
	  // increasing size of fromWorkspace with 1,and inserting
	  // the variable name of the block used in simulink
	  //myToWorkspace.push_back(blockType.c_str());
	  toWS.push_back(blockType);
	}
      }
    }
  }

  if (toWS.empty()) {
    fileError = "The Simulink file " + file +
      " used by " + this->getIdString() +
      "\ndoesn't seem to contain any \"ToWorkspace\" blocks";
    return false;
  }
  else if (fromWS.empty())
    fileError = "The Simulink file " + file +
      " used by " + this->getIdString() +
      "\ndoesn't seem to contain any \"FromWorkspace\" blocks";
  else
    fileError = "";

  myFromWorkspace.setValue(fromWS);
  myToWorkspace.setValue(toWS);

  std::vector<FmEngine*> v;
  myEngines.getPtrs(v);
  v.resize(fromWS.size());
  myEngines.setPtrs(v);

  return true;
}

//---------------------------------------------------

void FmExternalCtrlSys::getEntities(std::vector<FmSensorChoice>& toFill, int)
{
  toFill.clear();
  unsigned short int i = 0;
  for (const std::string& ws : myToWorkspace.getValue())
    toFill.push_back(FmSensorChoice(i++,ws));
}

//---------------------------------------------------

void FmExternalCtrlSys::setEngine(FmEngine* ctrl, size_t location)
{
  if (location < myEngines.size())
    myEngines[location] = ctrl;
  else
    myEngines.push_back(ctrl);
}

//---------------------------------------------------

bool FmExternalCtrlSys::getEngines(std::vector<FmEngine*>& v) const
{
  v.clear();
  myEngines.getPtrs(v);
  return !v.empty();
}

//---------------------------------------------------

/*!
  Reorders the engines.
  The original order of toBeOrdered corresponds to the order in fromThis,
  and is changed into the order of toThis. If a keyword is missing,
  or there are additional keywords, the corresponding pointer is set to NULL.
*/

static void reorderEngines(const Strings& fromThis, const Strings& toThis,
                           std::vector<FmEngine*>& toBeOrdered)
{
  // sanity check first
  if (fromThis.empty())
    return;

  if (toThis.empty()) {
    toBeOrdered.clear();
    return;
  }

  if (toBeOrdered.empty()) {
    toBeOrdered.resize(toThis.size(),NULL);
    return;
  }

  size_t i, j;
  std::vector<FmEngine*> tmp;

  for (i = 0; i < toThis.size(); i++) {
    for (j = 0; j < fromThis.size(); j++)
      if (toThis[i] == fromThis[j]) { // assumes all slots have different names...
	tmp.push_back(toBeOrdered[j]);
	break;
      }

    if (j == fromThis.size())
      tmp.push_back(NULL);
  }
  toBeOrdered = tmp;
}

//---------------------------------------------------

/*!
  To be called after fmm file parsing is complete.
  Reads the Simulink file, and checks for differences
  between the ToWS and FromWS vectors before and after
  reading file (in case the file has changed).
  Handles differences in order quite nicely :-)
*/

bool FmExternalCtrlSys::completeAfterParse()
{
  fileError = "";

  // no file path
  if (myFilePath.getValue().empty()) {
    myFromWorkspace.setValue(Strings());
    myToWorkspace.setValue(Strings());
    myEngines.clear();
    return true;
  }

  // Storing vectors read from file to detect any
  // deviance from last time the file was read
  Strings fromWS = myFromWorkspace.getValue();
  Strings toWS   = myToWorkspace.getValue();

  // Errors when reading file
  if (!this->readSimulinkFile()) {

    // Causes:
    // 1. File is unreadable
    // 2. Could not create file stream
    // 3. No to ws blocks
    fileError += "\nYou will have to correct the error before\n"
      "running the dynamics solver.";
    return false;
  }

  std::vector<FmEngine*> engines;
  myEngines.getPtrs(engines);
  Strings tmp;
  bool allPresent = true;
  bool somePresent = false; // handy?

  // Possible errors and actions
  // TODO

  // Checking from WS first
  tmp = myFromWorkspace.getValue();

  // Vectors are not equal.
  if (!(fromWS == tmp)) {
    // First check what went wrong, then take action accordingly
    // All or some present
    for (const std::string& ws : fromWS)
      if (std::find(tmp.begin(),tmp.end(),ws) == tmp.end())
	allPresent = false;
      else
	somePresent = true;

    fileError = "The Simulink file " + myFilePath.getValue() +
      " used by " + this->getIdString() +
      "\nhas changed since last save.\n\n";

    // Take action
    if (allPresent) {
      // Just re-order
      reorderEngines(fromWS, tmp, engines);
      fileError += "The individual ordering of the \"FromWorkpace\" blocks has changed, or\n"
	"additional blocks have been added. The existing connections are maintained.";
    }
    else {
      if (somePresent)
	reorderEngines(fromWS, tmp, engines);
      else // none present
	engines = std::vector<FmEngine*>(tmp.size(), NULL);

      fileError += "One or more of the \"FromWorkspace\" blocks have been\n"
	"removed or renamed. See this object's definition.";
    }
    myEngines.setPtrs(engines);
  }

  tmp = myToWorkspace.getValue();
  Strings missingTexts;

  // Checking for equality between the to WS vectors
  if (!(toWS == tmp)) {
    // All present
    for (const std::string& ws : toWS)
      if (std::find(tmp.begin(),tmp.end(),ws) == tmp.end()) {
	missingTexts.push_back(ws);
	allPresent = false;
      }

    if (!allPresent) {
      FmSensorBase* sensor = this->getSimpleSensor(true);
      sensor->getEngines(engines);
      std::vector<FmEngine*> warningEngines;
      // loop over all engines and search for some using the missing texts
      for (FmEngine* engine : engines)
	if (std::find(missingTexts.begin(),missingTexts.end(),engine->getEntityName()) != missingTexts.end())
	  warningEngines.push_back(engine);

      // If none referred to the missing sinks, all is ok.
      if (!warningEngines.empty()) {
	if (fileError.empty())
	  fileError = "The Simulink file " + myFilePath.getValue() +
	    " used by ",this->getIdString() +
	    "\nhas changed since last save.\n\n";
	else
	  fileError += "\n\n";

	fileError += "The number of \"ToWorkspace\" blacks has decreased."
	  " The Functions that were connected\n"
	  "to the the now missing \"ToWorkspace\" blocks on " + sensor->getIdString() +
	  "  must be re-visited.\n";

	if (warningEngines.size() > 1)
	  fileError += "The following Functions were connected to these \"ToWorkspace\" blocks:";
	else
	  fileError += "The following Function was connected to this \"ToWorkspace\" block:";

	for (FmEngine* engine : warningEngines)
	  fileError += "\n\t" + engine->getIdString(true);
      }
    }
  }

  return fileError.empty();
}

//---------------------------------------------------

std::ostream& FmExternalCtrlSys::writeFMF(std::ostream& os)
{
  os <<"EXTERNAL_CTRL_SYSTEM\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}

//---------------------------------------------------

bool FmExternalCtrlSys::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}

//---------------------------------------------------

bool FmExternalCtrlSys::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmExternalCtrlSys::getClassTypeID());
}

//---------------------------------------------------

bool FmExternalCtrlSys::readAndConnect(std::istream& is, std::ostream&)
{
  FmExternalCtrlSys* obj = new FmExternalCtrlSys();
  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFaFilePath::checkName(obj->myFilePath.getValue());

  obj->connect();
  return true;
}

//---------------------------------------------------

int FmExternalCtrlSys::printSolverEntry(FILE* fp)
{
  const Strings& fromWS = this->getFromWorkspace();
  if (fromWS.size() != myEngines.size()) {
    ListUI <<"\n---> INTERNAL ERROR: "<< this->getIdString() <<" is inconsistent.\n";
    return 1;
  }

  fprintf(fp, "&EXTERNAL_CONTROL_SYSTEM\n");
  this->printID(fp);

  std::string path = this->getFilePath();
  FFaFilePath::makeItAbsolute(path,FmDB::getMechanismObject()->getAbsModelFilePath());
  fprintf(fp, "  fileName = '%s'\n", path.c_str());
  fprintf(fp, "  sysType = 'MATLAB'\n");

  // Only writing if there is an engine selected
  size_t j;
  fprintf(fp, "  engineInID =");
  for (j = 0; j < myEngines.size(); j++)
    if (myEngines.getPtr(j))
      fprintf(fp, " %d", myEngines[j]->getBaseID());
  fprintf(fp, "\n  match =");
  for (j = 0; j < fromWS.size(); j++)
    if (myEngines.getPtr(j))
      fprintf(fp, " '%s'", fromWS[j].c_str());

  fprintf(fp, "\n/\n\n");
  return 0;
}
