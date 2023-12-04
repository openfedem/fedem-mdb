// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "eventCreator.H"
#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmDB.H"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


/*!
  Parses simulation event definitions from given input stream.
*/

bool FWP::createEvents(std::istream& is)
{
  FFaMsg::pushStatus("Reading event definitions");

  int errors = 0;
  std::map<FmSimulationEvent*,bool> failedEvents;

  while (FaParse::skipWhiteSpaceAndComments(is,true))
  {
    // Read the object type name and ID
    std::string typeName, heading;
    is >> typeName;
    if (!std::getline(is,heading)) break;

    // Get user ID and (possibly) assembly ID of the object
    int userId;
    std::vector<int> assId;
    std::istringstream objID(heading);
    objID >> userId;
    while (objID)
    {
      assId.push_back(userId);
      objID >> userId;
    }
    if (assId.empty()) break;
    userId = assId.front();
    assId.erase(assId.begin());

    // Convert model file keyword to its corresponding internal typename
    size_t i = 3;
    if (typeName.find("FUNC_") == 0)
      typeName.replace(0,5,"Fcf");
    else if (typeName.find("CONTROL_") == 0)
      typeName.replace(0,8,"Fcc");
    else
    {
      i = 2;
      typeName.insert(0,"Fc");
    }

    // Find the simulation model object that is modified in the events
    FmSimulationModelBase* smObj = NULL;
    int typeId = FFaTypeCheck::getTypeIDFromName(typeName.c_str());
    if (typeId < 0)
      FFaMsg::dialog("Unknown object type name \"" + typeName.substr(i) + "\".",
                     FFaMsg::ERROR);
    else
    {
      typeName.erase(0,i);
      FmBase* object = FmDB::findID(typeId,userId,assId);
      if (!object)
        FFaMsg::dialog("Non-existing object " + typeName +
                       " [" + heading + "].",FFaMsg::ERROR);
      else if (!object->isOfType(FmSimulationModelBase::getClassTypeID()))
        FFaMsg::dialog("Can not define event data for " + typeName +
                       " objects.",FFaMsg::ERROR);
      else
        smObj = static_cast<FmSimulationModelBase*>(object);
    }

    if (!smObj)
    {
      errors++;
      ListUI <<"  -> ERROR: Can not define event data for "
             << typeName <<" ["<< userId <<"].\n";
    }

    // Read header with data field specifications
    if (!FaParse::skipWhiteSpaceAndComments(is,true)) break;
    if (!std::getline(is,heading)) break;

    // Find the field names to be modified for this object
    size_t nameField = 0;
    size_t probabilityField = 0;
    std::vector<std::string> fields;
    if (heading.find("EVENTS") == 0)
      for (i = 6; i+1 < heading.size(); i++)
        if (!isspace(heading[i]))
        {
          size_t j = i+1;
          while (j < heading.size() && !isspace(heading[j])) j++;
          fields.push_back(heading.substr(i,j-i));

          if (fields.back() == "EVENT_NAME")
            nameField = fields.size();
          else if (fields.back() == "PROBABILITY")
            probabilityField = fields.size();

          // Convert some obsolete field names to new names here
          else if (fields.back() == "MEAN_PERIOD")
            fields.back() = "PEAK_PERIOD";

          i = j;
        }

    if (!smObj || fields.empty()) break;

    ListUI <<"     "<< smObj->getIdString();
    for (const std::string& fld : fields) ListUI <<" "<< fld;
    ListUI <<"\n";

    // Now parse the event data and create/update the simulation event objects
    FmSimulationEvent* event = NULL;
    while (FaParse::skipWhiteSpaceAndComments(is,true))
    {
      char c = is.get();
      is.putback(c);
      if (isalpha(c)) break;

      userId = 0;
      is >> userId;
      FmBase* obj = FmDB::findID(FmSimulationEvent::getClassTypeID(),userId);
      if (obj)
        event = static_cast<FmSimulationEvent*>(obj);
      else if (userId > 0)
      {
        event = new FmSimulationEvent();
        event->setID(userId);
        event->connect();
      }
      else
        break;

      int lerr = errors;
      std::string eventName;
      double probability = 0.0;
      for (size_t i = 0; i < fields.size(); i++)
      {
        if (i+1 == probabilityField)
        {
          is >> probability;
          continue;
        }

        std::string data;
        if (i+1 == fields.size())
        {
          // Parse the last field which is allowed to contain whitespaces also.
          // Read until end of line.
          char cline[BUFSIZ];
          is.getline(cline,BUFSIZ);
          data = cline;
        }
        else
        {
          // Parse a single field (until next whitespace)
          is >> data;
          if (data[0] == '"' && data.size() > 1 && data[data.size()-1] != '"')
          {
            // This is a text field within a pair of "'s.
            // The >> operator only reads until the first whitespace,
            // so we must continue char-by-char until the trailing ".
            for (char c = is.get(); c != '"' && is; c = is.get())
              data += c;
            data += '"';
          }
        }

        if (i+1 == nameField)
        {
          size_t p1 = data.find_first_of("\"");
          size_t p2 = data.size();
          if (p1 < p2)
            p2 = data.find_first_of("\"",++p1);
          else
            p1 = data.find_first_not_of(" \t");
          eventName = data.substr(p1,p2-p1);
        }
        else if (!event->addFieldValue(smObj,fields[i],data))
          errors++;
      }

      if (errors > lerr)
        failedEvents[event] = obj ? true : false;
      else
      {
        if (!eventName.empty())
        {
          event->setUserDescription(eventName);
          event->onChanged();
        }
        if (probability > 0.0)
          event->setProbability(probability);
      }
    }
  }

  FFaMsg::popStatus();
  if (errors == 0) return true;

  // Clean up the failed events to maintain model consistency
  for (const std::pair<FmSimulationEvent*,bool>& fail : failedEvents)
    if (fail.second)
      fail.first->clearTmpFields();
    else
      fail.first->erase();

  ListUI <<"===> Detected "<< errors
         <<" errors when parsing the event definition file.\n";
  FFaMsg::list("     The simulation events are probably incomplete.\n",true);

  return false;
}


static void isSimulationEvent(bool& retVal, FmBase* obj)
{
  retVal = obj->isOfType(FmSimulationEvent::getClassTypeID());
}


/*!
  Finalizes the simulation event definitions by resolving the field values.
*/

void FWP::resolveEvents()
{
  FFaMsg::pushStatus("Resolving event data fields");
  FFaDynCB2<bool&,FmBase*> headCB = FFaDynCB2S(isSimulationEvent,bool&,FmBase*);
  FFaDynCB1<FmBase*> allCB = FFaDynCB1S(FmDB::initAfterResolveObject,FmBase*);
  FmDB::forAllInDB(headCB,allCB);
  FFaMsg::popStatus();
}
