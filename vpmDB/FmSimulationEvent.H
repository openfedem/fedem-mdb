// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SIMULATION_EVENT_H
#define FM_SIMULATION_EVENT_H

#include "vpmDB/FmModelMemberBase.H"
#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmResultStatusData.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmSimulationEvent : public FmModelMemberBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSimulationEvent();

  virtual bool connect() { return this->eraseOrConnect(); }

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual void initAfterResolve();

  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  FmResultStatusData* getResultStatusData(bool current = true);
  bool isTouched() const { return myInitialRSD != 0; }

  virtual const char* getUITypeName() const { return "Simulation event"; }
  virtual const char** getListViewPixmap() const;

  bool addFieldValue(FmSimulationModelBase* obj,
		     const std::string& fieldName,
		     const std::string& fieldValue);

  void clearTmpFields() { tmpFields.clear(); }

  double getProbability() const { return myProbability.getValue(); }
  void setProbability(double p) { myProbability.setValue(p); }

  bool isModified(const FmSimulationModelBase* obj) const;
  void getObjects(std::vector<FmSimulationModelBase*>& objs) const;
  void getManipulatorObjects(std::vector<FmSimulationModelBase*>& objs);

  void activate(bool doActivate, bool doNotify = true);

  std::string eventName(const std::string& fileName) const;

protected:
  virtual ~FmSimulationEvent();

  FmSimulationEvent(const FmSimulationEvent&) = delete;
  FmSimulationEvent& operator=(const FmSimulationEvent&) = delete;

  static bool localParse(const char* keyWord, std::istream& activeStatement,
			 FmSimulationEvent* obj);

private:
  FFaField<double> myProbability;

  FFaField<FFaReferenceListBase*>         myRefsField;
  FFaReferenceList<FmSimulationModelBase> myRefs;
  std::vector<FmSimulationModelBase*>     myObjs;

  FFaField<FmResultStatusData> myRSD;
  FmResultStatusData*          myInitialRSD;

  typedef std::map<FDictIt,std::string,FDictLess> TmpFieldMap;

  std::vector<TmpFieldMap> tmpFields;
};

#endif
