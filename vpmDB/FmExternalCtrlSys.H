// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_EXTERNAL_CTRL_SYS_H
#define FM_EXTERNAL_CTRL_SYS_H

#include "vpmDB/FmIsMeasuredBase.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmExternalCtrlSys : public FmIsMeasuredBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmExternalCtrlSys();

  // ---  Get variables  ---
  bool getEngines(std::vector<FmEngine*>& v) const;
  FmEngine* getEngine(size_t idx) const { return myEngines.getPtr(idx); }
  const std::string& getFilePath() const { return myFilePath.getValue(); }
  const Strings& getFromWorkspace() const { return myFromWorkspace.getValue(); }
  const Strings& getToWorkspace() const { return myToWorkspace.getValue(); }

  virtual const char* getUITypeName() const {return "External Control System";}

  virtual bool isListable() const { return true; }

  void resetFlags() { inputFileHasChanged = false; fileError.clear(); }
  bool inputFileChanged() const { return inputFileHasChanged; }
  bool hasFileErrors() const { return !fileError.empty(); }
  const std::string& getErrorString() const { return fileError; }

  virtual void getEntities(std::vector<FmSensorChoice>& toFill, int);

  // ---  Set variables  ---
  void setEngine(FmEngine* ctrl, size_t location);
  void setFilePath(const std::string& path);
  void setFromWorkspace(const Strings& ws) { myFromWorkspace.setValue(ws); }
  void setToWorkspace(const Strings& ws) { myToWorkspace.setValue(ws); }

  // ---  File IO  ---
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  bool completeAfterParse();

  virtual int printSolverEntry(FILE* fp);

  // ---  Copy  ---
  virtual bool clone(FmBase* obj, int depth);

protected:

  // ---  Destructor  ---
  virtual ~FmExternalCtrlSys();

  // ---  File IO  ---
  virtual bool cloneLocal(FmBase* obj, int depth);

private:

  // --- Parse Simulink file ---
  bool readSimulinkFile();

public:
  FFaField<std::string> myFilePath;

private:
  FFaField<Strings> myFromWorkspace;
  FFaField<Strings> myToWorkspace;

  FFaReferenceList<FmEngine>      myEngines;
  FFaField<FFaReferenceListBase*> myEnginesField;

  std::string fileError;
  bool inputFileHasChanged;
};

#endif
