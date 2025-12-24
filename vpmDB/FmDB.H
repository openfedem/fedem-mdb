// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_DB_H
#define FM_DB_H

#include <map>
#include <vector>
#include <string>
#include <iostream>

#include "FFaLib/FFaContainers/FFaField.H"
#include "FFaLib/FFaDefinitions/FFaVersionNumber.H"
#include "FFaLib/FFaDynCalls/FFaDynCB.H"

class FmBase;
class FmModelMemberBase;
class FmMathFuncBase;
class FmfMultiVarBase;
class FmfDeviceFunction;
class FmfSpline;
class FmCtrlInputElementBase;
class FmCtrlLine;
class FmcInput;
class FmcOutput;
class FmExternalCtrlSys;
class FmJointBase;
class FmRigidJoint;
class FmRevJoint;
class FmBallJoint;
class FmFreeJoint;
class FmPrismJoint;
class FmCylJoint;
class FmCamJoint;
class FmHPBase;
class FmGear;
class FmRackPinion;
class FmJointSpring;
class FmJointDamper;
class FmAxialSpring;
class FmAxialDamper;
class FmSpringChar;
class FmSensorBase;
class FmEngine;
class FmRefPlane;
class FmSticker;
class FmMechanism;
class FmLink;
class FmPart;
class FmBeam;
class FmTriad;
class FmLoad;
class FmSeaState;
class FmAirState;
class FmVesselMotion;
class FmSimulationEvent;
class FmSubAssembly;
class FmFuncTree;
class FmTurbine;
class FmBladeDesign;

class FmQuery;
class FmRingStart;
class FmAnalysis;
class FmModesOptions;
class FmGageOptions;
class FmFppOptions;
class FmModelExpOptions;
class FmGlobalViewSettings;
class FaMat34;
class FaVec3;


namespace FmDB
{
  using FmHeadMap = std::map<int,FmRingStart*>;

  void init();
  void removeInstances();

  void initHeadMap(FmHeadMap& headMap, FmFuncTree*& funcTree);
  void sortHeadMap(const FmHeadMap& headMap, FmHeadMap& sortedHeadMap,
                   bool reverse = false);

  FmModelMemberBase* createObject(int classTypeId);

  FmMechanism* newMechanism();

  // Head object return
  FmRingStart* getHead(int classTypeID);
  FmRingStart* getHead(int classTypeID, const std::vector<int>& assemblyID,
                       const FmHeadMap* root = NULL);

  // Search for ID
  bool findIDrange(const FmBase* obj, int& fromID, int& toID);
  FmBase* findID(int type, int IDnr, const std::vector<int>& assemblyID = {});
  FmBase* findID(const std::string& type, int IDnr,
                 const std::vector<int>& assemblyID = {});

  // Collects all pathnames in the model
  void getAllPaths(std::vector<FFaField<std::string>*>& allPathNames,
                   const FmSubAssembly* subAss = NULL);
  // Corrects all relative pathnames to external input files in the model
  // when moving it to a different directory (Save As...)
  void translateRelativePaths(const std::string& oldModelPath,
                              const std::string& newModelPath,
                              const FmSubAssembly* subAss = NULL);

  // Compresses the model by removing all inactive joint dof objects
  bool purgeJointComponents();

  bool updateModelVersionOnSave(bool warnOnNewVersion);

  bool reportAll(std::ostream& os = std::cout, bool writeMetaData = true,
                 const FmHeadMap* headMap = NULL, const char* metaData = NULL);
  void reportMembers(std::ostream& os, const FmHeadMap* headMap);

  void displayAll(const FmHeadMap* headMap = NULL);
  bool eraseAll(bool showProgress = false);

  void forAllInDB(FFaDynCB2<bool&,FmBase*>& toBeCalled4EachHead,
                  FFaDynCB1<FmBase*>& toBeCalled4Each,
                  const FmHeadMap* root = NULL);

  int getObjectCount(int type, const FmHeadMap* root = NULL);

  void getQuery(std::vector<FmModelMemberBase*>& toBeFilled, FmQuery* query);

  bool getAllOfType(std::vector<FmModelMemberBase*>& toBeFilled, int typeID,
                    const FmSubAssembly* subAss = NULL, const char* tag = NULL);

  bool hasObjectsOfType(int typeID, const FmHeadMap* root = NULL);

  void getFEParts(std::vector<FmPart*>& toFill, bool reverseOrder = false);
  void getUnsavedParts(std::vector<FmPart*>& toFill);
  void getAllParts(std::vector<FmPart*>& toFill,
                   const FmSubAssembly* subAss = NULL,
                   bool thisLevelOnly = false);
  void getAllBeams(std::vector<FmBeam*>& toFill,
                   const FmSubAssembly* subAss = NULL,
                   bool thisLevelOnly = false);
  void getAllLinks(std::vector<FmLink*>& toFill,
                   const FmSubAssembly* subAss = NULL,
                   bool thisLevelOnly = false);
  void getAllTriads(std::vector<FmTriad*>& toFill,
                    const FmSubAssembly* subAss = NULL,
                    bool thisLevelOnly = false);
  void getAllFunctions(std::vector<FmMathFuncBase*>& toFill,
                       const FmSubAssembly* subAss = NULL,
                       bool thisLevelOnly = false);

  void getAllSplines(std::vector<FmfSpline*>& toFill);
  void getAllMultiVarFuncs(std::vector<FmfMultiVarBase*>& toFill);
  void getAllDeviceFunctions(std::vector<FmfDeviceFunction*>& toFill);
  void getAllJoints(std::vector<FmJointBase*>& toFill);
  void getAllHPs(std::vector<FmHPBase*>& toFill);

  void getAllGears(std::vector<FmGear*>& toFill);
  void getAllRackPinions(std::vector<FmRackPinion*>& toFill);
  void getAllCylJoints(std::vector<FmCylJoint*>& toFill);
  void getAllCamJoints(std::vector<FmCamJoint*>& toFill);

  void getAllRevJoints(std::vector<FmRevJoint*>& toFill);
  void getAllBallJoints(std::vector<FmBallJoint*>& toFill);
  void getAllFreeJoints(std::vector<FmFreeJoint*>& toFill);
  void getAllRigidJoints(std::vector<FmRigidJoint*>& toFill);
  void getAllPrismJoints(std::vector<FmPrismJoint*>& toFill);
  void getAllControlLines(std::vector<FmCtrlLine*>& toFill);
  void getAllLoads(std::vector<FmLoad*>& toFill);

  void getAllAxialSprings(std::vector<FmAxialSpring*>& toFill);
  void getAllJointSprings(std::vector<FmJointSpring*>& toFill);
  void getAllSpringChars(std::vector<FmSpringChar*>& toFill);
  void getAllAxialDampers(std::vector<FmAxialDamper*>& toFill);
  void getAllJointDampers(std::vector<FmJointDamper*>& toFill);

  void getAllSensors(std::vector<FmSensorBase*>& toFill);
  void getAllEngines(std::vector<FmEngine*>& engines);
  void getAllExternalCtrlSys(std::vector<FmExternalCtrlSys*>& toFill);
  void getAllRefPlanes(std::vector<FmRefPlane*>& toFill);
  void getAllStickers(std::vector<FmSticker*>& toFill);

  void getAllControlOutput(std::vector<FmcOutput*>& toFill);
  void getAllControlInput(std::vector<FmcInput*>& toFill);
  void getAllControlElements(std::vector<FmCtrlInputElementBase*>& toFill);

  void getAllBladeDesigns(std::vector<FmBladeDesign*>& toFill);
  void getAllSimulationEvents(std::vector<FmSimulationEvent*>& toFill,
                              bool reverseOrder = false);

  bool readAll(const std::string& name, char ignoreFileVersion = 0);
  int  readFMF(std::istream& is);

  void unknownKeyword(const char* keyWord, const FmBase* obj);

  FmGlobalViewSettings* getActiveViewSettings(bool createIfNone = true);
  FmAnalysis* getActiveAnalysis(bool createIfNone = true);
  FmModesOptions* getModesOptions(bool createIfNone = true);
  FmGageOptions* getGageOptions(bool createIfNone = true);
  FmFppOptions* getFppOptions(bool createIfNone = true);
  FmModelExpOptions* getModelExportOptions(bool createIfNone = true);
  FmVesselMotion* getActiveRAO();

  double getPositionTolerance();
  double getParallelTolerance();
  void setParallelTolerance(double eps);
  const FaVec3& getGrav();
  void drawGVector();

  FaMat34 getSeaCS();
  bool    useSeaCS();
  void    drawSea();

  FmLink* getEarthLink();
  FmSensorBase* getTimeSensor(bool createIfNone = true);
  FmSeaState* getSeaStateObject(bool createIfNone = true);
  FmAirState* getAirStateObject(bool createIfNone = true);
  FmMechanism* getMechanismObject(bool createIfNone = true);
  FmTurbine* getTurbineObject(int ID = 0);

  void emergencyExitSave();

  void eraseAllStickers();
  void eraseAllControlObjects();

  bool hasObjects(int typeID, const FmHeadMap* root = NULL);

  const FFaVersionNumber& getModelFileVer();

  const FmHeadMap* getHeadMap(const FmSubAssembly* subAss = NULL);

  FmSubAssembly* getSubAssembly(const std::vector<int>& assemblyID);

  // Finds the model member based on baseID
  FmModelMemberBase* findObject(int baseID);
  bool insertInBaseIDMap(FmModelMemberBase* pt);
  void removeFromBaseIDMap(FmModelMemberBase* pt);

  // Returns next free baseID
  int getFreeBaseID();

  // Convenience methods for resolving
  void resolveObject(FmBase* obj);
  void initAfterResolveObject(FmBase* obj);
}

#endif
