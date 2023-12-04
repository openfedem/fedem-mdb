// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <fstream>

#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmStructAssembly.H"
#include "vpmDB/FmSoilPile.H"
#include "vpmDB/FmJacket.H"
#include "vpmDB/FmRiser.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmFuncTree.H"
#include "vpmDB/FmResultBase.H"
#include "vpmDB/FmIsPositionedBase.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFileSys.H"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


std::string FmSubAssembly::mainFilePath;
std::pair<int,int> FmSubAssembly::old2newAssID(0,0);
FmSubAssembly::FmHeadMap* FmSubAssembly::tmpHeadMap = NULL;


Fmd_DB_SOURCE_INIT(FcSUBASSEMBLY, FmSubAssembly, FmModelMemberBase);


FmSubAssembly::FmSubAssembly(bool isDummy)
  : FmModelMemberBase(isDummy), myFuncTree(NULL)
{
  Fmd_CONSTRUCTOR_INIT(FmSubAssembly);

  if (isDummy) return; // No fields or sub-trees in dummy objects

  FFA_FIELD_DEFAULT_INIT(myModelFile,"MODEL_FILE_NAME");

  FmDB::initHeadMap(myHeadMap,myFuncTree);

  for (const FmHeadMap::value_type head : myHeadMap)
    head.second->setParentAssembly(this);

  myFuncTree->setParentAssembly(this);
}


FmSubAssembly::~FmSubAssembly()
{
  this->disconnect();
}


bool FmSubAssembly::eraseOptions()
{
  // Erase the rings in reverse order
  FmHeadMap sortedMap;
  FmHeadMap::reverse_iterator it;
  FmDB::sortHeadMap(myHeadMap,sortedMap);
  for (it = sortedMap.rbegin(); it != sortedMap.rend(); ++it)
  {
    if (it->second->eraseRingMembers())
      ListUI <<" --> All "<< it->second->getUITypeName() <<" erased.\n";
#ifdef FM_DEBUG
    else
      ListUI <<"     No "<< it->second->getUITypeName() <<" in this model.\n";
#endif
    myHeadMap.erase(it->second->getRingMemberType());
    delete it->second;
  }

  delete myFuncTree;

  return this->FmModelMemberBase::eraseOptions();
}


bool FmSubAssembly::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmSubAssembly::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmSubAssembly::getClassTypeID());
}


FmSubAssembly* FmSubAssembly::duplicate(FmSubAssembly* parent) const
{
  if (!this->isCopyable()) return NULL;

  // Create a new sub-assembly of similar type as this one
  ListUI <<"Copying Subassembly "<< this->getInfoString() <<".\n";
  FmSubAssembly* newAss;
  if (dynamic_cast<const FmStructAssembly*>(this))
    newAss = new FmStructAssembly();
  else if (dynamic_cast<const FmRiser*>(this))
    newAss = new FmRiser();
  else if (dynamic_cast<const FmJacket*>(this))
    newAss = new FmJacket();
  else if (dynamic_cast<const FmSoilPile*>(this))
    newAss = new FmSoilPile();
  else
    newAss = new FmSubAssembly();

  // Copy all data fields of this sub-assembly
  newAss->clone(const_cast<FmSubAssembly*>(this),FmBase::SHALLOW);
  newAss->setUserDescription("Copy of " + this->getInfoString());
  if (parent) newAss->setParentAssembly(parent);
  newAss->connect();

  // Get old and new ID path for the child objects of the copied sub-assembly
  std::vector<int> oldAssID, newAssID;
  this->getAssemblyID(oldAssID);
  newAss->getAssemblyID(newAssID);
  oldAssID.push_back(this->getID());
  newAssID.push_back(newAss->getID());

  // Lambda function for cloning a DB object. Note that all references are
  // copied in an unresolved state (DEEP_UNRESOLVED), since they must be updated
  // to refer to objects in the newly copied sub-assembly before they are resolved.
  auto&& cloneIt = [&oldAssID,&newAssID,newAss](FmBase* obj)
  {
    FmBase* newObj = obj->clone(FmBase::DEEP_UNRESOLVED);
    if (newObj)
    {
      newObj->setID(obj->getID());
      newObj->updateReferences(oldAssID,newAssID);
      newObj->setParentAssembly(newAss);
    }
    return newObj;
  };

  // First, copy child sub-assemblies, if any, to resolve references depth-first
  FmHeadMap::const_iterator it = myHeadMap.find(FmSubAssembly::getClassTypeID());
  if (it != myHeadMap.end())
    for (FmBase* pt = it->second->getNext(); pt != it->second; pt = pt->getNext())
      static_cast<FmSubAssembly*>(pt)->duplicate(newAss);

  // Then, copy other child objects of this sub-assembly
  FmBase* newObj = NULL;
  for (const FmHeadMap::value_type& h : myHeadMap)
    if (h.second->getRingMemberType() != FmSubAssembly::getClassTypeID() &&
        h.second->getRingMemberType() != FmElementGroupProxy::getClassTypeID())
    {
      for (FmBase* pt = h.second->getNext(); pt != h.second; pt = pt->getNext())
        if ((newObj = cloneIt(pt)))
        {
          newObj->connect();
          if (pt->isOfType(FmPart::getClassTypeID()))
          {
            // Clone element groups of this Part and connect them to the new one
            FmElementGroupProxy* newGroup = NULL;
            std::vector<FmElementGroupProxy*> groups;
            static_cast<FmPart*>(pt)->getElementGroups(groups);
            for (FmElementGroupProxy* group : groups)
              if ((newGroup = dynamic_cast<FmElementGroupProxy*>(cloneIt(group))))
              {
                // For implicit groups, we can point to the same actual object
                // since the underlying FE model also is the same one
                newGroup->setRealObject(group->getRealObject());
                newGroup->connect(newObj);
              }
          }
        }
        else
          std::cerr <<" *** FmSubAssembly::duplicate(): "<< pt->getInfoString()
                    <<" is not copied."<< std::endl;
    }

  // Resolve all references in the new sub-assembly
  newAss->resolveAfterRead();
  return newAss;
}


bool FmSubAssembly::mergeOldHeadMapAndConnect()
{
  std::vector<int> assID;
  this->getAssemblyID(assID);
  FmBase* old = FmDB::findID(this->getTypeID(),this->getID(),assID);
  FmSubAssembly* oldAss = dynamic_cast<FmSubAssembly*>(old);
  if (!oldAss || oldAss == this) return false;

  // Copy the head map from the old object into this sub-assembly
  myHeadMap = oldAss->myHeadMap;
  this->sendSignal(MODEL_MEMBER_CHANGED);
  delete oldAss;

  return this->connect();
}


bool FmSubAssembly::hasObjects(int typeId) const
{
  return FmDB::hasObjectsOfType(typeId,&myHeadMap);
}


bool FmSubAssembly::isListable(bool resultView) const
{
  if (resultView)
    return this->hasObjects(FmResultBase::getClassTypeID());
  else
    return this->hasObjects(FmSimulationModelBase::getClassTypeID());
}


std::ostream& FmSubAssembly::writeFMF(std::ostream& os)
{
  os <<"SUBASSEMBLY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,myHeadMap);
  else
    this->writeFMF(myModelFile.getValue());

  return os;
}


bool FmSubAssembly::writeFMF(const std::string& fileName) const
{
  if (fileName.empty()) return false;

  std::string fullName(fileName);
  if (!mainFilePath.empty() && FmFileSys::verifyDirectory(mainFilePath))
    FFaFilePath::makeItAbsolute(fullName,mainFilePath);
  if (FmFileSys::isFile(fullName))
    FmFileSys::renameFile(fullName,fullName+".bak");

  std::string metaData = "!Submodel: " + this->getIdString();
  std::ofstream fs(fullName.c_str(),std::ios::out);
  if (FmDB::reportAll(fs,false,myHeadMap,metaData.c_str()))
    return true;

  ListUI <<" ==> Failure writing Subassembly file: "<< fullName <<"\n";
  return false;
}


bool FmSubAssembly::readAndConnect(std::istream& is, std::ostream&)
{
  FmSubAssembly* obj = new FmSubAssembly();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This sub-assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
                << std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


bool FmSubAssembly::localParse(const char* keyWord, std::istream& activeStatement,
                               FmSubAssembly* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmSubAssembly::readFMF(const std::string& fileName, FmSubAssembly** subAss)
{
  if (fileName.empty()) return true;

  // This sub-assembly is stored in a separate model file.
  // Now read this file before continuing with the master file.

  std::string fullName(fileName);
  const std::string& modelFilePath = FmDB::getMechanismObject()->getAbsModelFilePath();
  FFaFilePath::makeItAbsolute(fullName,modelFilePath);
  std::ifstream fs(fullName.c_str(),std::ios::in);
  if (fs)
    ListUI <<"  => Reading Subassembly file "<< fullName <<"\n";
  else
  {
    ListUI <<" ==> Could not open Subassembly file: "<< fullName
           <<"\n     Check that the file exists and that you have"
           <<" the proper read permissions to it.\n";
    return false;
  }

  // Check the first line that this is a valid model file
  char line[256];
  fs.getline(line,80);
  std::string firstLine(line);
  if (firstLine.find("FEDEMMODELFILE") == std::string::npos)
  {
    ListUI <<"===> ERROR: The first line should contain the heading FEDEMMODELFILE.\n";
    return false;
  }

  FmSubAssembly* newAss = NULL;
  if (subAss)
  {
    // We are importing this sub-assembly into a new model.
    // Try to parse assembly ID and type from the meta data.
    int uid = 0;
    fs.getline(line,256);
    while (line[0] == '!' && !fs.eof() && uid == 0)
      if (strncmp(line,"!Submodel:",10) == 0)
      {
        if (strncmp(line+11,"Beamstring",10) == 0) {
          newAss = new FmRiser();
          uid = atoi(line+23);
        }
        else if (strncmp(line+11,"Jacket",6) == 0) {
          newAss = new FmJacket();
          uid = atoi(line+19);
        }
        else if (strncmp(line+11,"Soil Pile",9) == 0) {
          newAss = new FmSoilPile();
          uid = atoi(line+22);
        }
        else if (strncmp(line+11,"Structure Assembly",18) == 0) {
          newAss = new FmStructAssembly();
          uid = atoi(line+31);
        }
        else if (strncmp(line+11,"Sub Assembly",12) == 0) {
          newAss = new FmSubAssembly();
          uid = atoi(line+25);
        }
      }
      else
        fs.getline(line,256);

    if (newAss)
      newAss->setID(uid);
    else
    {
      // This is a regular model file which now is imported as a sub-assembly
      newAss = new FmStructAssembly();
      FmModelMemberBase::ignoreBaseID = true;
    }

    *subAss = newAss;
    std::string aFile(FFaFilePath::getRelativeFilename(modelFilePath,fullName));
    if (!newAss->connect())
    {
      // A sub-assembly with this user ID aready exists.
      // Must assign a new unique ID to avoid conflicts.
      newAss->setID(0);
      newAss->connect();
      old2newAssID = std::make_pair(uid,newAss->getID());
      FmModelMemberBase::ignoreBaseID = true;
      uid = old2newAssID.second;

      // Cannot use the same file name since the references will be different
      size_t dotPos = aFile.find_last_of('.');
      if (dotPos < aFile.size())
        aFile.insert(dotPos,FFaNumStr("_%d",newAss->getID()));
      else
        aFile.append(FFaNumStr("_%d",newAss->getID()));
    }
    if (uid > 0)
      newAss->myModelFile.setValue(aFile);
    else // Make all objects into sub-objects of this assembly
    {
      old2newAssID = std::make_pair(0,newAss->getID());
      tmpHeadMap = &newAss->myHeadMap;
    }
    ListUI <<"  => Created "<< newAss->getIdString() <<"\n";
  }

  // Now parse the file and connect the objects of this assembly
  int readStat = FmDB::readFMF(fs);
  FmModelMemberBase::ignoreBaseID = false;
  if (readStat < 1)
  {
    ListUI <<" ==> Failure reading Subassembly file: "<< fullName <<"\n";
    if (newAss)
      newAss->erase();
    if (subAss)
      *subAss = NULL;
  }
  else if (newAss)
  {
    // Imported sub-assembly.
    // Set part-specific repository for FE parts, if any,
    // such that their reduced data is reused in the new model.
    std::vector<FmPart*> allParts;
    FmDB::getAllParts(allParts,newAss);
    for (FmPart* part : allParts)
      if (part->isFEPart() && part->myRepository.getValue().empty())
      {
        std::string linkDB = FFaFilePath::getBaseName(fileName,true) + "_RDB";
        FFaFilePath::appendToPath(linkDB,"link_DB");
        part->myRepository.setValue(linkDB);
      }

    // Translate relative pathnames according to the current model
    FmDB::translateRelativePaths(FFaFilePath::getPath(fullName,false),
                                 modelFilePath,newAss);

    if (tmpHeadMap)
    {
      // Resolve references that are local within this sub-assembly
      // before resolving the sub-assembly itself
      FFaDynCB2<bool&,FmBase*> headCB;
      FFaDynCB1<FmBase*> allCB = FFaDynCB1S(FmDB::resolveObject,FmBase*);
      FmDB::forAllInDB(headCB,allCB,newAss->getHeadMap());

      // Now set the new sub-assembly as parent assembly for all top-level objects
      std::vector<FmModelMemberBase*> allObjs;
      FmDB::getAllOfType(allObjs,FmModelMemberBase::getClassTypeID(),newAss);
      for (FmModelMemberBase* obj : allObjs)
        if (!obj->getParentAssembly())
          obj->setParentAssembly(newAss);
    }
    else if (old2newAssID.first != old2newAssID.second)
    {
      // Update assembly ID in the references before resolving them
      std::vector<FmModelMemberBase*> allObjs;
      FmDB::getAllOfType(allObjs,FmModelMemberBase::getClassTypeID(),newAss);
      for (FmModelMemberBase* obj : allObjs)
        obj->updateReferences(old2newAssID.first,old2newAssID.second);
    }
  }

  tmpHeadMap = NULL;
  old2newAssID = std::make_pair(0,0);
  return readStat > 0;
}


void FmSubAssembly::resolveAfterRead()
{
  FFaMsg::setSubTask("Resolving topology");

  // Resolve the conflicting baseIDs, if any
  FmModelMemberBase::resolveBaseIDProblems();

  FFaDynCB2<bool&,FmBase*> headCB;
  FFaDynCB1<FmBase*> allCB;

  // Resolve references that are read through a field
  allCB = FFaDynCB1S(FmDB::resolveObject,FmBase*);
  FmDB::forAllInDB(headCB,allCB,&myHeadMap);

  // Set up the other references and connections.
  // Make sure objects are initialized after resolving if necessary
  allCB = FFaDynCB1S(FmDB::initAfterResolveObject,FmBase*);
  FmDB::forAllInDB(headCB,allCB,&myHeadMap);

  // Make sure 3D location and coordinate systems are in sync
  std::vector<FmModelMemberBase*> allPosBases;
  FmDB::getAllOfType(allPosBases,FmIsPositionedBase::getClassTypeID(),this);
  for (FmModelMemberBase* obj : allPosBases)
    static_cast<FmIsPositionedBase*>(obj)->updateLocation();
  FmDB::getAllOfType(allPosBases,FmSubAssembly::getClassTypeID(),this);
  for (FmModelMemberBase* obj : allPosBases)
    static_cast<FmSubAssembly*>(obj)->updateLocation('T');

  FFaMsg::setSubTask("");
}
