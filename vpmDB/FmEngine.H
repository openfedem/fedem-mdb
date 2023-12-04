// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ENGINE_H
#define FM_ENGINE_H

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmThreshold.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


template<> inline
bool FFaField<FmThreshold>::isPrintable() const
{
  return myData.isDefined();
}


class FmEngine : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmEngine(bool useTimeSensor = true);

  virtual const char* getUITypeName() const { return "Function"; }
  virtual const char** getListViewPixmap() const;
  virtual std::string getInfoString() const;

  virtual bool disconnect();

  virtual FmBase* duplicate() const;

  //! \brief Returns all objects that are using the output from this engine.
  bool getUsers(std::vector<FmModelMemberBase*>& toFill,
                bool recursive = false) const;

  FmMathFuncBase* getFunction() const;
  FmSensorBase*   getSensor(size_t i = 0) const;
  int             getSensorId(size_t i = 0) const;

  const std::string& getEntityName(size_t i = 0) const;
  int                getEntity(size_t i = 0) const;
  int                getDof(size_t i = 0) const;

  size_t getNoArgs() const;

  FmSensorBase* getUniqueSensor() const;
  bool initGetValue() const;
  bool getValue(double x, double& y) const;

  void setFunction(FmMathFuncBase* func);
  void setSensor(FmSensorBase* sensor, size_t i = 0);

  void setEntityName(const std::string& name, size_t i = 0);
  void setEntity(int ent, size_t i = 0);
  void setDof(int dof, size_t i = 0);

  bool isActive() const;
  bool isControlInEngine() const;
  bool isControlOutEngine() const;
  bool isDriveFile() const;
  bool isExternalFunc() const;
  virtual bool isListable() const;

  bool      isFunctionLinked() const { return !myFunctionOwner.isNull(); }
  FmEngine* getEngineToLinkFunctionFrom() const { return myFunctionOwner.getPointer(); }
  void      setEngineToLinkFunctionFrom(FmEngine* engine);

  void translateJointSensorEntity();

  static void updateFunctionLinkedFromStuff();
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fd);

  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

protected:
  virtual ~FmEngine();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReference<FmMathFuncBase> myFunction;
  FFaField<FFaReferenceBase*>  myFunctionField;

  FFaReference<FmEngine>       myFunctionOwner;
  FFaField<FFaReferenceBase*>  myFunctionOwnerField;

  FFaReferenceList<FmSensorBase>  mySensor;
  FFaField<FFaReferenceListBase*> mySensorField;

  FFaField<Strings> myEntityNames;
  FFaField<IntVec>  myEntities;
  FFaField<IntVec>  myDofs;

public:
  FFaField<bool> myOutput;
  FFaField<FmThreshold> myThreshold;

  static std::set<int> betaFeatureEngines;
};

#endif
