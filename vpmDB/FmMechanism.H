// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MECHANISM_H
#define FM_MECHANISM_H

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmResultStatusData.H"
#include "FFaLib/FFaAlgebra/FFaUnitCalculator.H"
#include "FFaLib/FFaAlgebra/FFaVec3.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmMechanism : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmMechanism();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual void initAfterResolve();

  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual int printSolverEntry(FILE* fp);

  FmResultStatusData* getResultStatusData(bool current = true);

  virtual const char* getUITypeName() const { return "Mechanism"; }
  virtual bool isListable() const { return false; }

  const std::string& getModelFileName() const    { return myModelFileName; }
  const std::string& getAbsModelFilePath() const { return myAbsModelFilePath; }
  const std::string& getAbsModelRDBPath() const  { return myAbsModelRDBPath; }

  bool isUntitled() const;

  std::string getRelativePath(const std::string& fullName) const;
  std::string getModelName(bool keepExt = false) const;
  std::string getAbsModelLRDBPath(bool createDir = false) const;
  std::string getPropertyLibPath(bool createDir = true) const;
  std::string getAirFoilLibPath() const;
  std::string getAbsBladeFolderPath() const;

  void syncPath(const std::string& name, bool updateRSD = false);
  bool setGravity(const FaVec3& vec);

  bool isEnabled(const std::string& resultFileName) const;
  bool getDisabledResultFiles(Strings& fileNames, bool absPath = true) const;
  bool hasDisabledFiles() const;
  bool disableResultFile(const std::string& fileName);
  bool enableResultFile(const std::string& fileName);
  bool enableAllResultFiles();

  FFaField<std::string>       propertyRepository;
  FFaField<std::string>       modelLinkRepository;
  FFaField<FFaUnitCalculator> modelDatabaseUnits;

  FFaField<std::string> cadModelFileName;
  FFaField<std::string> cadConfigurationName;

  FFaField<std::string> activeFunctionPlugin;
  FFaField<std::string> activeElementPlugin;

  FFaField<double> positionTolerance;
  FFaField<FaVec3> gravity;
  FFaField<FaVec3> initVel;

protected:
  virtual ~FmMechanism();

  FmMechanism(const FmMechanism&) = delete;
  FmMechanism& operator=(const FmMechanism&) = delete;

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FmResultStatusData* myInitialRSD;

  std::string myAbsModelFilePath;
  std::string myAbsModelRDBPath;
  std::string myModelFileName;

  FFaField<FmResultStatusData> myResultStatusData;
  FFaField<StringSet>          myDisabledResults;

  // Moved to FmAnalysis in R5.1
  FFaObsoleteField<int> maxConcurrentProcesses;
};

#endif
