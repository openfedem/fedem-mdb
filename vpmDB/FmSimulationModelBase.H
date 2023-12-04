// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SIMULATION_MODEL_BASE_H
#define FM_SIMULATION_MODEL_BASE_H

#include "vpmDB/FmModelMemberBase.H"

class FmSimulationEvent;


class FmSimulationModelBase : public FmModelMemberBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual void initAfterResolve();

  // For list view filtering
  virtual bool isListable() const { return false; }

  // For simulation event handling
  virtual void onEventSwitched(const FmSimulationEvent*) {}
  virtual void initAfterParse() {}
  virtual bool parseField(const std::string& keyWord,
                          const std::string& fieldValue);

  void removeFieldsExceptFor(const std::vector<FDictIt>& keep);
  void setAsDefault(const std::vector<FDictIt>& fieldNames);
  void getFields(std::vector<FDictIt>& fieldNames) const;

  bool isEventModified() const;
  std::string getObjectInfo() const;

  FmSimulationModelBase* copy(int cloneDepth) const;
  virtual FmBase* clone(int depth) const { return this->copy(depth); }
  using FmBase::clone;

  static std::string relPathCorrection; // for writing of solver input file

protected:
  FmSimulationModelBase();
  virtual ~FmSimulationModelBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmSimulationModelBase* obj);
};


class FmStructPropertyBase : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual bool isListable() const { return true; }
  virtual FmBase* duplicate() const;

protected:
  FmStructPropertyBase();
  virtual ~FmStructPropertyBase() {}
};

#endif
