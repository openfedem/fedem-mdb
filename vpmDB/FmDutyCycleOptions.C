// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaTokenizer.H"
#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmDB.H"
#include "vpmDB/FmDutyCycleOptions.H"


Fmd_DB_SOURCE_INIT(FcDUTYCYCLEOPTIONS, FmDutyCycleOptions, FmSimulationModelBase);


FmDutyCycleOptions::FmDutyCycleOptions()
{
  Fmd_CONSTRUCTOR_INIT(FmDutyCycleOptions);

  FFA_FIELD_INIT(myEquivUnitScale, 1.0, "EQUIV_UNIT_SCALE");
  FFA_FIELD_DEFAULT_INIT(myEquivUnit, "EQUIV_UNIT");
  FFA_FIELD_DEFAULT_INIT(myEvents, "EVENTS");
  FFA_REFERENCELIST_FIELD_INIT(myLinksField, myLinks, "LINKS");
  myLinks.setAutoSizing(false);
}


FmDutyCycleOptions::~FmDutyCycleOptions()
{
  this->disconnect();
}


/*!
  Adds one event
*/

void FmDutyCycleOptions::addEvent(const std::string& file, const std::string& aname,
				  double repeats, bool master)
{
  myEvents.getValue().addEvent(file, aname, (int)repeats, master);
}


/*!
  Returns the events present
*/

const std::map<std::string,EventData>& FmDutyCycleOptions::getEvents() const
{
  return myEvents.getValue().getEvents();
}


/*!
  Checks if one event is present
*/

bool FmDutyCycleOptions::eventPresent(const std::string& file) const
{
  return myEvents.getValue().eventPresent(file);
}


/*!
  Removes an event
*/

void FmDutyCycleOptions::removeEvent(const std::string& file)
{
  myEvents.getValue().removeEvent(file);
}


std::string FmDutyCycleOptions::getMasterEvent() const
{
  return myEvents.getValue().getMasterEvent();
}


/*!
  Removes all events
*/

void FmDutyCycleOptions::removeAllEvents()
{
  myEvents.getValue().clear();
}


/*!
  Sets the links to be processed
*/

void FmDutyCycleOptions::setLinks(const std::vector<FmLink*>& links)
{
  myLinks.setPtrs(links);
}


/*!
  Adds a link. Will only add if the link isn't already present
*/

void FmDutyCycleOptions::addLink(FmLink* link)
{
  if (!myLinks.hasPtr((FFaFieldContainer*)link))
    myLinks.push_back(link);
}


/*!
  Retrieves std::vector of links
*/

std::vector<FmLink*> FmDutyCycleOptions::getLinks() const
{
  std::vector<FmLink*> v;
  myLinks.getPtrs(v);
  return v;
}


/*!
  Clears the list of links
*/

void FmDutyCycleOptions::clearLinks()
{
  myLinks.clear();
}


/*!
  Sets the equivalent unit
  The equivalent unit consists of a number and a unit,
  such as e.g. "1 Day", "5 rounds" etc.
*/

void FmDutyCycleOptions::setEquivalentUnit(double scale, const std::string& unit)
{
  myEquivUnitScale.setValue(scale);
  myEquivUnit.setValue(unit);
}


/*!
  Returns the integer part of the eq unit
*/

double FmDutyCycleOptions::getEquivUnitScale() const
{
  return myEquivUnitScale.getValue();
}


/*!
  Returns the std::string part of the eq unit
*/

std::string FmDutyCycleOptions::getEquivUnit() const
{
  return myEquivUnit.getValue();
}


bool FmDutyCycleOptions::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


std::ostream& FmDutyCycleOptions::writeFMF(std::ostream& os)
{
  os <<"DUTYCYCLEOPTIONS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmDutyCycleOptions::readAndConnect(std::istream& is, std::ostream&)
{
  FmDutyCycleOptions* obj = new FmDutyCycleOptions();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  return obj->cloneOrConnect();
}


bool FmDutyCycleOptions::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmDutyCycleOptions::getClassTypeID());
}


bool FmDutyCycleEvents::operator==(const FmDutyCycleEvents& ev) const
{
  return this == &ev;
}


void FmDutyCycleEvents::clear()
{
  myEvents.clear();
}


void FmDutyCycleEvents::addEvent(const std::string& file, const std::string& name,
				 double repeats, bool master)
{
  myEvents[file] = EventData(name, repeats, master);
}


void FmDutyCycleEvents::removeEvent(const std::string& file)
{
  myEvents.erase(file);
}


bool FmDutyCycleEvents::eventPresent(const std::string& file) const
{
  return myEvents.find(file) != myEvents.end();
}


std::string FmDutyCycleEvents::getMasterEvent() const
{
  std::map<std::string,EventData>::const_iterator it;
  for (it = myEvents.begin(); it != myEvents.end(); ++it)
    if (it->second.isMaster)
      return it->first;

  return "";
}


const std::map<std::string,EventData>& FmDutyCycleEvents::getEvents() const
{
  return myEvents;
}


void FmDutyCycleEvents::write(std::ostream& os) const
{
  if (myEvents.empty()) return;

  os << "\n<\n";
  bool makeComma = false;
  std::map<std::string,EventData>::const_iterator it;
  for (it = myEvents.begin(); it != myEvents.end(); ++it) {
    if (makeComma)
      os << ",\n";
    makeComma = true;
    os << "\t<\"" << (*it).first << "\","; //file
    os << "\""  << (*it).second.name << "\","; // name
    os << (*it).second.repeats << ","; // repeats
    os << (*it).second.isMaster << ">";
  }
  os << "\n>";
}


void FmDutyCycleEvents::read(std::istream& is)
{
  char c;
  while (is.get(c) && isspace(c)); // read to non-whitespace

  if (c == '<')
    this->processTokens(FFaTokenizer(is,'<','>',','));
}


void FmDutyCycleEvents::processTokens(const std::vector<std::string>& tokens)
{
  std::string file, name;
  int rep = 0, master = 0;
  for (size_t i = 0; i < tokens.size(); i++)
    if (tokens[i][0] == '<')
      // first character is '<', further processing is required
      this->processTokens(FFaTokenizer(tokens[i],'<','>',','));
    else if (i == 0)
      file = tokens[i];
    else if (i == 1)
      name = tokens[i];
    else if (i == 2)
      rep = atof(tokens[i].c_str());
    else if (i == 3)
      master = atoi(tokens[i].c_str());

  if (!file.empty())
    myEvents[file] = EventData(name, rep, master > 0);
}


std::ostream& operator<<(std::ostream& os, const FmDutyCycleEvents& events)
{
  events.write(os);
  return os;
}


std::istream& operator>>(std::istream& is, FmDutyCycleEvents& events)
{
  events.read(is);
  return is;
}
