// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_DUTY_CYCLE_OPTIONS_H
#define FM_DUTY_CYCLE_OPTIONS_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmLink.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"
#include <map>


/*!
  Container class for all the events
*/

struct EventData
{
  EventData(const std::string& n="", int r=0, bool master=false) : name(n), repeats(r), isMaster(master) {}
  std::string name;
  double repeats;
  bool isMaster;
};

class FmDutyCycleEvents
{
public:
  FmDutyCycleEvents() {}
  FmDutyCycleEvents(const std::string& path, const std::string& name,
		    double repeats, bool master);

  bool operator==(const FmDutyCycleEvents& ev) const;

  void clear();
  void addEvent(const std::string& file, const std::string& name,
		double repeats, bool master);
  void removeEvent(const std::string& file);
  bool eventPresent(const std::string& file) const;
  std::string getMasterEvent() const;

  const std::map<std::string,EventData>& getEvents() const;

  void write(std::ostream& os) const;
  void read(std::istream& is);

protected:
  void processTokens(const std::vector<std::string>& tokens);

  std::map<std::string,EventData> myEvents;

  friend std::ostream& operator<<(std::ostream& os, const FmDutyCycleEvents& events);
  friend std::istream& operator>>(std::istream& is, FmDutyCycleEvents& events);
};


class FmDutyCycleOptions : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmDutyCycleOptions();

  void addEvent(const std::string& file, const std::string& name,
		double repeats, bool master);
  const std::map<std::string,EventData>& getEvents() const;
  bool eventPresent(const std::string& file) const;
  std::string getMasterEvent() const;
  void removeEvent(const std::string& file);
  void removeAllEvents();

  void setLinks(const std::vector<FmLink*>& links);
  void addLink(FmLink* link);
  void clearLinks();
  std::vector<FmLink*> getLinks() const;

  void   setEquivalentUnit(double scale, const std::string& unit);
  double getEquivUnitScale() const;
  std::string getEquivUnit() const;

  virtual const char* getUITypeName() const { return "DutyCycleOptions"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  virtual ~FmDutyCycleOptions();

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<double>                myEquivUnitScale;
  FFaField<std::string>           myEquivUnit;
  FFaReferenceList<FmLink>        myLinks;
  FFaField<FFaReferenceListBase*> myLinksField;
  FFaField<FmDutyCycleEvents>     myEvents;
};

#endif
