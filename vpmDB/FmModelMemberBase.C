// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmModelMemberBase.H"
#include "vpmDB/FmModelMemberConnector.H"
#include "vpmDB/FmResultBase.H"
#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include <regex>


bool FmModelMemberBase::inInteractiveErase = false;
bool FmModelMemberBase::ignoreBaseID = false;
int FmModelMemberBase::baseIDProblemCount = 0;
std::map<FmModelMemberBase*,int> FmModelMemberBase::baseIDProblems;

#ifdef FM_DEBUG
#define DBGID(p) p->getTypeIDName() <<" "<< p->getID() <<" ("<< p->getBaseID() <<")"
#endif

Fmd_DB_SOURCE_INIT(FcMODEL_MEMBER_BASE, FmModelMemberBase, FmBase);


FmModelMemberBase::FmModelMemberBase(bool isDummy) : FmBase(isDummy)
{
  Fmd_CONSTRUCTOR_INIT(FmModelMemberBase);

  // Note that myBaseID is not added as a field here, as it needs to
  // be preserved only for objects with results (FmIsPlottedBase).
  myBaseID.setValue(-1);

  if (!isDummy) // No fields in dummy objects
    FFA_FIELD_DEFAULT_INIT(myTag,"TAG");
}


FFaSwitchBoardConnector* FmModelMemberBase::getSignalConnector()
{
  return FmSignalConnector::instance();
}


void FmModelMemberBase::getItemAssemblyPath(std::vector<int>& assID) const
{
  this->getAssemblyID(assID);
}


void FmModelMemberBase::printID(FILE* myFile, bool withBaseID) const
{
  std::vector<int> assID;
  this->getAssemblyID(assID);
  if (withBaseID)
    fprintf(myFile,"  id = %d\n", this->getBaseID());
  fprintf(myFile,"  extId = %d", this->getID());
  for (int id : assID) fprintf(myFile," %d",id);
  fprintf(myFile,"\n");

  // Print only the first line if multi-line description
  std::string descr = this->getUserDescription(128);
  if (!descr.empty()) fprintf(myFile,"  extDescr = '%1.128s'\n", descr.c_str());
}


void FmModelMemberBase::onMainConnected()
{
#ifdef FM_DEBUG
  std::cout <<"FmModelMemberBase::onMainConnected() "<< DBGID(this) << std::endl;
#endif

  if (this == this->getPrev()) return;

  if (ignoreBaseID || this->getBaseID() == -1)
    this->setBaseID(FmDB::getFreeBaseID());

  if (!FmDB::insertInBaseIDMap(this)){
    FmModelMemberBase* objInWay = FmDB::findObject(this->getBaseID());

#ifdef FM_DEBUG
    std::cout <<"\tProblems inserting baseID "<< this->getBaseID()
              <<"\n\tObject in the way: "<< DBGID(objInWay) << std::endl;
#endif
    if (!objInWay->isOfType(FmIsPlottedBase::getClassTypeID())) {
      // Blocking object is not plotable, just give it a new baseID
      FmDB::removeFromBaseIDMap(objInWay);
      FmDB::insertInBaseIDMap(this);
      objInWay->setBaseID(FmDB::getFreeBaseID());
      FmDB::insertInBaseIDMap(objInWay);
    }
    else {
      baseIDProblems[this] = this->getBaseID();
#ifdef FM_DEBUG
      std::cout <<"\tAdded to baseIDProblemMap: "<< this->getBaseID() << std::endl;
#endif
    }
  }

  this->mainConnectedEvent();

  if (!myFields.empty())
    this->sendSignal(MODEL_MEMBER_CONNECTED);
}


void FmModelMemberBase::onMainAboutToDisconnect()
{
#ifdef FM_DEBUG
  std::cout <<"FmModelMemberBase::onMainAboutToDisconnect() "<< DBGID(this) << std::endl;
#endif

  if (this == this->getPrev()) return;

  FmModelMemberBase* thisObjInMap = FmDB::findObject(this->getBaseID());
  if (thisObjInMap == this || thisObjInMap == NULL) // else we have trouble.
    FmDB::removeFromBaseIDMap(this);

  this->mainDisconnectedEvent();

  if (!myFields.empty())
    this->sendSignal(MODEL_MEMBER_DISCONNECTED);
}


void FmModelMemberBase::onMainDisconnected()
{
#ifdef FM_DEBUG
  std::cout <<"FmModelMemberBase::onMainDisconnected() "<< DBGID(this) << std::endl;
#endif

  if (!myFields.empty())
    this->sendSignal(MODEL_MEMBER_FINISHED_DISCONNECTED);
}


void FmModelMemberBase::onChanged()
{
#ifdef FM_DEBUG
  std::cout <<"FmModelMemberBase::onChanged() "<< DBGID(this) << std::endl;
#endif

  this->changedEvent();

  if (!myFields.empty())
    this->sendSignal(MODEL_MEMBER_CHANGED);
}


bool FmModelMemberBase::eraseOptions()
{
  // Print a log to the Output List view when interactively
  // erasing objects from either the Object or Result views.

  if (inInteractiveErase && this->getID() > 0)
  {
    if (this->isOfType(FmResultBase::getClassTypeID()))
      ListUI <<"  -> Erasing "<< this->getIdString() <<"\n";
    else if (this->isOfType(FmSimulationModelBase::getClassTypeID()))
      if (static_cast<FmSimulationModelBase*>(this)->isListable())
        ListUI <<"  -> Erasing "<< this->getIdString() <<"\n";
  }

  return true;
}


void FmModelMemberBase::sendSignal(Signal sig)
{
  FFaSwitchBoardCall(FmSignalConnector::instance(),sig,this);
}


bool FmModelMemberBase::cloneLocal(FmBase* obj, int depth)
{
#ifdef FM_DEBUG
  std::cout <<"FmModelMemberBase::cloneLocal() "<< DBGID(this) << std::endl;
#endif

  if (!obj->isOfType(FmModelMemberBase::getClassTypeID()))
    return false;

  // cloneObj is the object that we are cloning from.
  // This is usually an object being read from the model file,
  // while "this" is a temporary "dummy" object created by
  // the instant resolving step in the model file read action
  FmModelMemberBase* cloneObj = static_cast<FmModelMemberBase*>(obj);
#ifdef FM_DEBUG
  std::cout <<"\tCloning model member "<< DBGID(cloneObj) << std::endl;
#endif

  // update for objects from "pre R2.5" stage, when the baseID was not saved
  if (cloneObj->getBaseID() == -1)
    cloneObj->setBaseID(FmDB::getFreeBaseID());

  if (depth < FmBase::DEEP_APPEND)
    return true;

  if (cloneObj != cloneObj->getPrev()) {
    // object to clone from is connected - big time trouble
#ifdef FM_DEBUG
    std::cerr <<"WARNING: FmModelMemberBase::cloneLocal [cloneObj connected]"<< std::endl;
#endif
    // Naa, no big trouble if doing a simple copy of some object into another.
    // But no messing with the baseID should be done. Return here anyway.
    // Changed return value from false to true, and removed the message.
    // JJS 2004-06-11
    return true;
  }

  if (this == this->getPrev()) {
    // this object is not connected, just copy the baseID from the cloned object
    this->setBaseID(cloneObj->getBaseID());
    return true;
  }

  FmModelMemberBase* thisObjInMap = FmDB::findObject(this->getBaseID());
  if (thisObjInMap == this || thisObjInMap == NULL)
    FmDB::removeFromBaseIDMap(this);

  // There is a blocking object (inserted in baseIDmap after this is connected).
  // Can't remove this->baseID from baseIDmap.
  // We then check for the objects with cloneObj->baseID
  FmModelMemberBase* cloneObjInMap = FmDB::findObject(cloneObj->getBaseID());
  if (!cloneObjInMap) {
    // free position - insert.
    this->setBaseID(cloneObj->getBaseID());
    FmDB::insertInBaseIDMap(this);
  }
  else if (!cloneObjInMap->isOfType(FmIsPlottedBase::getClassTypeID())) {
    // blocking obj is not plotable, push it to the end of the baseIDmap
    FmDB::removeFromBaseIDMap(cloneObjInMap);
    this->setBaseID(cloneObj->getBaseID());
    FmDB::insertInBaseIDMap(this);
    cloneObjInMap->setBaseID(FmDB::getFreeBaseID());
    FmDB::insertInBaseIDMap(cloneObjInMap);
  }
  else {
    this->setBaseID(cloneObj->getBaseID());
    baseIDProblems[this] = this->getBaseID();
#ifdef FM_DEBUG
    std::cout <<"\tAdded to baseIDProblemMap: "<< DBGID(this)
              <<"\n\tObject in the way: "<< DBGID(cloneObjInMap) << std::endl;
#endif
  }
  return true;
}


/*!
  This method contains the 'BaseId magic created by Jens' (moved from FmDB).
  It should only be invoked immediately after reading a model from file.

  All objects that has experienced not to be inserted in the baseIDmap,
  because the baseID was already taken, are now inserted with a new baseID.
  Usually the objects that were in the way have disappeared due to cloning etc.
  It is mostly (only) the "quasi"static objects that are created on demand
  that are the problems, because they are both created and read from file.
  Time Sensor, Analysis, Mechanism and GlobalViewSettings.
*/

void FmModelMemberBase::resolveBaseIDProblems()
{
  if (baseIDProblems.empty()) return;

#ifdef FM_DEBUG
  std::cout <<"\nFmModelMemberBase::resolveBaseIDProblems() "
	    << baseIDProblems.size() << std::endl;
#endif

  int troubleCounter = 0;
  for (const std::pair<FmModelMemberBase*,int>& bit : baseIDProblems)
    if (!FmDB::insertInBaseIDMap(bit.first))
    {
      if (bit.first->isOfType(FmIsPlottedBase::getClassTypeID()))
      {
	if (++troubleCounter == 1)
	{
	  FFaMsg::dialog("Problems encountered while loading mechanism.\n"
			 "Please check the Output List window.");
	  ListUI <<"\n===> WARNING: MODEL FILE CURVE REFERENCES HAVE CHANGED:\n"
		 <<"     If some of the following objects are referred by curves, you should\n"
		 <<"     revisit the definition (check object ID in topology view with object\n"
		 <<"     in curve definition).\n";
	}
	ListUI <<"  -> "<< bit.first->getIdString(true) <<"\n";
      }

      bit.first->setBaseID(FmDB::getFreeBaseID());
      FmDB::insertInBaseIDMap(bit.first);
#ifdef FM_DEBUG
      FmModelMemberBase* objInWay = FmDB::findObject(bit.second);
      std::cout <<"\tTrouble: "<< DBGID(bit.first) << std::endl;
      if (objInWay) std::cout <<"\tObject in the way: "<< DBGID(objInWay) << std::endl;
#endif
    }
#ifdef FM_DEBUG
    else
      std::cout <<"\t"<< DBGID(bit.first) <<" OK"<< std::endl;
#endif

  baseIDProblems.clear();
}


bool FmModelMemberBase::moveTo(FmSubAssembly* newAss)
{
  if (this->getParentAssembly() == newAss)
    return false;

  this->mainDisconnect();
  this->setParentAssembly(newAss);
  if (this->mainConnect(this->isOfType(FmElementGroupProxy::getClassTypeID())))
  {
    this->onChanged();
    return true;
  }

  // Failed to reconnect, have to erase this object
  // since we don't want to have disconnected objects around
  inInteractiveErase = true;
  this->erase();
  inInteractiveErase = false;
  return false;
}


bool FmModelMemberBase::isTagged(const std::string& tag) const
{
  if (myTag.getValue().find(tag) != std::string::npos)
    return true;
  else // Try a regular expression match
    return std::regex_match(myTag.getValue(), std::regex(tag.c_str()));
}
