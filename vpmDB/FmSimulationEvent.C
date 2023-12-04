// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstring>

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmaps.H"


Fmd_DB_SOURCE_INIT(FcSIMULATION_EVENT, FmSimulationEvent, FmModelMemberBase);


FmSimulationEvent::FmSimulationEvent()
{
  Fmd_CONSTRUCTOR_INIT(FmSimulationEvent);

  FFA_FIELD_INIT(myProbability, 1.0, "PROBABILITY");
  FFA_REFERENCELIST_FIELD_INIT(myRefsField, myRefs, "OBJECTS");
  FFA_FIELD_DEFAULT_INIT(myRSD, "RESULT_STATUS_DATA");

  myInitialRSD = NULL;
}


FmSimulationEvent::~FmSimulationEvent()
{
  this->disconnect();

  for (FmSimulationModelBase* obj : myObjs)
    if (obj) obj->erase();

  if (myInitialRSD)
    delete myInitialRSD;
}


const char** FmSimulationEvent::getListViewPixmap() const
{
  return myRSD.getValue().isEmpty() ? NULL : event_xpm;
}


FmResultStatusData* FmSimulationEvent::getResultStatusData(bool current)
{
  if (current)
    return &myRSD.getValue();

  if (!myInitialRSD)
    myInitialRSD = new FmResultStatusData();

  return myInitialRSD;
}


bool FmSimulationEvent::isModified(const FmSimulationModelBase* obj) const
{
  return myRefs.hasPtr(obj);
}


void FmSimulationEvent::getObjects(std::vector<FmSimulationModelBase*>& objs) const
{
  myRefs.getPtrs(objs);
}


void FmSimulationEvent::getManipulatorObjects(std::vector<FmSimulationModelBase*>& objs)
{
  objs.clear();
  objs.reserve(myObjs.size());
  for (FmSimulationModelBase* obj : myObjs)
    if (obj) objs.push_back(obj);
}


void FmSimulationEvent::activate(bool doActivate, bool doNotify)
{
  for (size_t i = 0; i < myRefs.size(); i++)
    if (myRefs[i].isResolved() && myObjs[i])
    {
      if (doActivate)
	myRefs[i]->copyFields(myObjs[i]);
      else
	myRefs[i]->resetFields(myObjs[i]);

      myRefs[i]->initAfterParse();
    }

  if (doNotify)
    // Bugfix #468: Modify all referred objects first before doing onEventSwitched,
    // in case some objects depends on modified data in other objects.
    for (size_t i = 0; i < myRefs.size(); i++)
      if (myRefs[i].isResolved() && myObjs[i])
      {
        myRefs[i]->onEventSwitched(this);
        myRefs[i]->onChanged();
      }
}


std::ostream& FmSimulationEvent::writeFMF(std::ostream& os)
{
  os <<"SIMULATION_EVENT\n{\n";
  this->writeFields(os);
  for (FmSimulationModelBase* obj : myObjs)
  {
    os <<"BREAK;\n";
    if (obj)
      obj->writeFields(os);
  }
  os <<"}\n\n";
  return os;
}


bool FmSimulationEvent::readAndConnect(std::istream& is, std::ostream&)
{
  FmSimulationEvent* obj = new FmSimulationEvent();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmSimulationEvent::localParse(const char* keyWord, std::istream& activeStatement,
				   FmSimulationEvent* obj)
{
  if (strcmp(keyWord,"BREAK") == 0)
    obj->tmpFields.push_back(TmpFieldMap());
  else if (obj->tmpFields.empty())
    return parentParse(keyWord, activeStatement, obj);
  else if (strcmp(keyWord,"MEAN_PERIOD") == 0)
    return localParse("PEAK_PERIOD", activeStatement, obj);
  else if (strcmp(keyWord,"LONGEST_PERIOD") == 0 ||
	   strcmp(keyWord,"SHORTEST_PERIOD") == 0)
  {
    // Conversion of the two obsolete fields into the new pair field
    // (these are only found in models created in the R6-i6 version)
    FDictIt strit = FieldContainerDict::instance()->find("PERIOD_RANGE");
    if (strit == FieldContainerDict::instance()->end()) return false;

    std::string tmp;
    activeStatement >> tmp;
    if (*tmp.rbegin() == '\0')
      tmp.erase(tmp.size()-1); // Erase trailing null-character

    // Caution: This works only if both keywords are specified,
    // and in alphabetic order (i.e. LONGEST_PERIOD first)
    std::string& value = obj->tmpFields.back()[strit];
    if (value.empty())
      value = "(shortest) " + tmp;
    else // Assuming LONGEST_PERIOD has been read
      value.replace(0,10,tmp);
  }
  else
  {
    // Parse fields pertaining to an altered object
    FDictIt strit = FieldContainerDict::instance()->find(keyWord);
    if (strit == FieldContainerDict::instance()->end())
    {
      ListUI <<"  -> ERROR: \""<< keyWord
	     <<"\" is not a defined data field name in this model.\n";
      return false;
    }
    char buf[BUFSIZ];
    activeStatement.getline(buf,BUFSIZ);
    obj->tmpFields.back()[strit] = buf;
  }
  return true;
}


bool FmSimulationEvent::addFieldValue(FmSimulationModelBase* obj,
				      const std::string& fieldName,
				      const std::string& fieldValue)
{
  FDictIt strit = FieldContainerDict::instance()->find(fieldName);
  if (strit == FieldContainerDict::instance()->end())
  {
    ListUI <<"  -> ERROR: \""<< fieldName
	   <<"\" is not a defined data field name in this model.\n";
    return false;
  }

#ifdef FM_DEBUG
  std::cout << this->getIdString() <<": "<< obj->getIdString() <<", "
	    << fieldName <<" = "<< fieldValue << std::endl;
#endif

  int i = myRefs.size();
  if (!myRefs.hasPtr(obj,&i))
    myRefs.push_back(obj);

  if (i >= (int)tmpFields.size())
    tmpFields.resize(i+1);

  tmpFields[i][strit] = fieldValue;
  return true;
}


void FmSimulationEvent::initAfterResolve()
{
  myObjs.resize(myRefs.size(),NULL);
  for (size_t i = 0; i < myRefs.size() && i < tmpFields.size(); i++)
    if (myRefs[i].isResolved() && !tmpFields[i].empty())
    {
      // Duplicate the referred object without copying any of its field values
      FmSimulationModelBase* oldObj = myObjs[i];
      if (!(myObjs[i] = myRefs[i]->copy(FmBase::NOTHING)))
      {
	ListUI <<"  -> ERROR: Event modification of "<< myRefs[i]->getUITypeName()
	       <<" objects is not supported\n";
	continue;
      }

      // Insert the special field values of this event
      std::vector<FDictIt> altered;
      for (const TmpFieldMap::value_type& tmp : tmpFields[i])
	// Try to parse the field value string, non-data fields are disallowed
	if (myObjs[i]->parseField(*tmp.first,tmp.second))
	  altered.push_back(tmp.first);
	else
	  ListUI <<"  -> ERROR: \""<< *tmp.first
		 <<" is not a data field in "<< myRefs[i]->getIdString() <<"\n";

      myRefs[i]->setAsDefault(altered);
      if (oldObj)
      {
	// Include the previously defined fields
	oldObj->getFields(altered);
	myObjs[i]->copyFields(oldObj);
	oldObj->erase();
      }

      // Remove all fields from the duplicated object which are not altered
      myObjs[i]->removeFieldsExceptFor(altered);
      //myObjs[i]->initAfterParse(); //Bugfix #278: Removed. Think this is safe
    }

  tmpFields.clear();

  // Set the root path of result files associated with this event
  FFaNumStr event("event_%03d",this->getID());
  FFaFilePath::makeItAbsolute(event,FmDB::getMechanismObject()->getAbsModelRDBPath());
  myRSD.getValue().setPath(event);
}


/*!
  Modify the given \a fileName to identify this simulation event:
  <prefix>.<ext> --> <prefix>_event_<ID>.<ext>
  where <ID> is the user-ID of this event.
  Ensure it has an absolute path, assuming it is relative to the model file.
*/

std::string FmSimulationEvent::eventName(const std::string& fileName) const
{
  std::string eName(fileName);
  size_t dotPos = eName.rfind('.');
  if (dotPos < eName.size())
    eName.insert(dotPos,FFaNumStr("_event_%d",this->getID()));
  else if ((dotPos = eName.find_last_of("/\\")) == eName.size()-1)
    eName.insert(dotPos,FFaNumStr("_event_%d",this->getID()));
  else
    eName.append(FFaNumStr("_event_%d",this->getID()));

  FFaFilePath::makeItAbsolute(eName,FmDB::getMechanismObject()->getAbsModelFilePath());
  return eName;
}
