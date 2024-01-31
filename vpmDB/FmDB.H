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
#ifdef FT_HAS_EXTCTRL
class FmExternalCtrlSys;
#endif
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
#ifdef FT_HAS_NCODE
class FmDutyCycleOptions;
#endif
class FmModelExpOptions;
class FmGlobalViewSettings;
class FaMat34;
class FaVec3;


class FmDB
{
  typedef std::map<int,FmRingStart*> FmHeadMap;

public:
  static void init();
  static void removeInstances();

  static void initHeadMap(FmHeadMap& headMap, FmFuncTree*& funcTree);
  static void sortHeadMap(const FmHeadMap& headMap, FmHeadMap& sortedHeadMap,
                          bool reverse = false);

  static FmModelMemberBase* createObject(int classTypeId);

  static FmMechanism* newMechanism();

  // Head object return
  static FmRingStart* getHead(int classTypeID);
  static FmRingStart* getHead(int classTypeID, const std::vector<int>& assemblyID,
                              const FmHeadMap* root = NULL);
  static FmRingStart* getHead(int classTypeID, const FmHeadMap* headMap);

  // Search for ID
  static bool findIDrange(const FmBase* obj, int& fromID, int& toID);
  static FmBase* findID(int type, int IDnr,
			const std::vector<int>& assemblyID = std::vector<int>());
  static FmBase* findID(const std::string& type, int IDnr,
			const std::vector<int>& assemblyID = std::vector<int>());

  // Collects all pathnames in the model
  static void getAllPaths(std::vector<FFaField<std::string>*>& allPathNames,
                          const FmSubAssembly* subAss = NULL);
  // Corrects all relative pathnames to external input files in the model
  // when moving it to a different directory (Save As...)
  static void translateRelativePaths(const std::string& oldModelPath,
				     const std::string& newModelPath,
				     const FmSubAssembly* subAss = NULL);

  // Compresses the model by removing all inactive joint dof objects
  static bool purgeJointComponents();

  static bool updateModelVersionOnSave(bool warnOnNewVersion);

  static bool reportAll(std::ostream& os = std::cout, bool writeMetaData = true,
                        const FmHeadMap& headMap = ourHeadMap,
                        const char* addMetaData = NULL);
  static void reportMembers(std::ostream& os, const FmHeadMap& headMap);

  static void displayAll(const FmHeadMap& headMap = ourHeadMap);
  static bool eraseAll(bool showProgress = false);

  static void forAllInDB(FFaDynCB2<bool&,FmBase*>& toBeCalled4EachHead,
                         FFaDynCB1<FmBase*>& toBeCalled4Each,
                         const FmHeadMap* root = &ourHeadMap);

  static int getObjectCount(int type, const FmHeadMap* root = &ourHeadMap);

  // Vector to fill up, map with TypeID and a bool saying
  // if we want/don't want objects of type TypeID
  static void getTypeQuery(std::vector<FmModelMemberBase*>& toBeFilled,
                           const std::map<int,bool>& query);

  static void getQuery(std::vector<FmModelMemberBase*>& toBeFilled, FmQuery* query);

  static bool getAllOfType(std::vector<FmModelMemberBase*>& toBeFilled, int typeID,
                           const FmSubAssembly* subAss = NULL, const char* tag = NULL);

  static bool hasObjectsOfType(int typeID, const FmHeadMap* root = &ourHeadMap);

  static void getFEParts(std::vector<FmPart*>& toFill, bool reverseOrder = false);
  static void getUnsavedParts(std::vector<FmPart*>& toFill);
  static void getAllParts(std::vector<FmPart*>& toFill,
			  const FmSubAssembly* subAss = NULL,
			  bool thisLevelOnly = false);
  static void getAllBeams(std::vector<FmBeam*>& toFill,
			  const FmSubAssembly* subAss = NULL,
			  bool thisLevelOnly = false);
  static void getAllLinks(std::vector<FmLink*>& toFill,
			  const FmSubAssembly* subAss = NULL,
			  bool thisLevelOnly = false);
  static void getAllTriads(std::vector<FmTriad*>& toFill,
			   const FmSubAssembly* subAss = NULL,
			   bool thisLevelOnly = false);
  static void getAllFunctions(std::vector<FmMathFuncBase*>& toFill,
			      const FmSubAssembly* subAss = NULL,
			      bool thisLevelOnly = false);

  static void getAllSplines(std::vector<FmfSpline*>& toFill);
  static void getAllMultiVarFuncs(std::vector<FmfMultiVarBase*>& toFill);
  static void getAllDeviceFunctions(std::vector<FmfDeviceFunction*>& toFill);
  static void getAllJoints(std::vector<FmJointBase*>& toFill);
  static void getAllHPs(std::vector<FmHPBase*>& toFill);

  static void getAllGears(std::vector<FmGear*>& toFill);
  static void getAllRackPinions(std::vector<FmRackPinion*>& toFill);
  static void getAllCylJoints(std::vector<FmCylJoint*>& toFill);
  static void getAllCamJoints(std::vector<FmCamJoint*>& toFill);

  static void getAllRevJoints(std::vector<FmRevJoint*>& toFill);
  static void getAllBallJoints(std::vector<FmBallJoint*>& toFill);
  static void getAllFreeJoints(std::vector<FmFreeJoint*>& toFill);
  static void getAllRigidJoints(std::vector<FmRigidJoint*>& toFill);
  static void getAllPrismJoints(std::vector<FmPrismJoint*>& toFill);
  static void getAllControlLines(std::vector<FmCtrlLine*>& toFill);
  static void getAllLoads(std::vector<FmLoad*>& toFill);

  static void getAllAxialSprings(std::vector<FmAxialSpring*>& toFill);
  static void getAllJointSprings(std::vector<FmJointSpring*>& toFill);
  static void getAllSpringChars(std::vector<FmSpringChar*>& toFill);
  static void getAllAxialDampers(std::vector<FmAxialDamper*>& toFill);
  static void getAllJointDampers(std::vector<FmJointDamper*>& toFill);

  static void getAllSensors(std::vector<FmSensorBase*>& toFill);
  static void getAllEngines(std::vector<FmEngine*>& engines);
#ifdef FT_HAS_EXTCTRL
  static void getAllExternalCtrlSys(std::vector<FmExternalCtrlSys*>& toFill);
#endif
  static void getAllRefPlanes(std::vector<FmRefPlane*>& toFill);
  static void getAllStickers(std::vector<FmSticker*>& toFill);

  static void getAllControlOutput(std::vector<FmcOutput*>& toFill);
  static void getAllControlInput(std::vector<FmcInput*>& toFill);
  static void getAllControlElements(std::vector<FmCtrlInputElementBase*>& toFill);

  static void getAllBladeDesigns(std::vector<FmBladeDesign*>& toFill);
  static void getAllSimulationEvents(std::vector<FmSimulationEvent*>& toFill,
                                     bool reverseOrder = false);

  static bool readAll(const std::string& name, char ignoreFileVersion = 0);
  static int  readFMF(std::istream& is);

  static FmGlobalViewSettings* getActiveViewSettings(bool createIfNone = true);
  static FmAnalysis* getActiveAnalysis(bool createIfNone = true);
  static FmModesOptions* getModesOptions(bool createIfNone = true);
  static FmGageOptions* getGageOptions(bool createIfNone = true);
  static FmFppOptions* getFppOptions(bool createIfNone = true);
#ifdef FT_HAS_NCODE
  static FmDutyCycleOptions* getDutyCycleOptions(bool createIfNone = true);
#endif
  static FmModelExpOptions* getModelExportOptions(bool createIfNone = true);
  static FmVesselMotion* getActiveRAO();

  static double getPositionTolerance();
  static const FaVec3& getGrav();
  static void drawGVector();

  static FaMat34 getSeaCS();
  static bool    useSeaCS();
  static void    drawSea();

  static FmLink* getEarthLink() { return itsEarthLink; }
  static FmSensorBase* getTimeSensor(bool createIfNone = true);
  static FmSeaState* getSeaStateObject(bool createIfNone = true);
  static FmAirState* getAirStateObject(bool createIfNone = true);
  static FmMechanism* getMechanismObject(bool createIfNone = true);
  static FmTurbine* getTurbineObject(int ID = 0);

  static void emergencyExitSave();

  static void eraseAllStickers();
  static void eraseAllControlObjects();

  static bool hasObjects(int typeID, const FmHeadMap* root = &ourHeadMap);

  static const FFaVersionNumber& getModelFileVer() { return ourModelFileVersion; }

  static const FmHeadMap* getHeadMap(const FmSubAssembly* subAss = NULL);
  static const FmHeadMap* getHeadMap(const std::vector<int>& assemblyID,
                                     const FmHeadMap* root = &ourHeadMap);

  static FmSubAssembly* getSubAssembly(const std::vector<int>& assemblyID);

  // Finds the model member based on baseID
  static FmModelMemberBase* findObject(int baseID);
  static bool insertInBaseIDMap(FmModelMemberBase* pt);
  static void removeFromBaseIDMap(FmModelMemberBase* pt);

  // Returns next free baseID
  static int getFreeBaseID();

  // Convenience methods for resolving
  static void resolveObject(FmBase* obj);
  static void initAfterResolveObject(FmBase* obj);

private:
  static FmHeadMap                        ourHeadMap;
  static std::map<int,FmModelMemberBase*> ourBaseIDMap;

  static FmFuncTree* itsFuncTree;
  static FmLink*     itsEarthLink;

  static FFaVersionNumber ourCurrentFedemVersion;
  static FFaVersionNumber ourModelFileVersion;
  static int              ourSaveNr;

public:
  static std::map<std::string,int> unknownKeywords;

private:
  static bool appendAllOfType(std::vector<FmModelMemberBase*>& toBeFilled, int typeID,
                              const std::vector<int>& butNotOfTypes = std::vector<int>(),
                              const std::string& tagged = "");

  static bool appendAllOfType(std::vector<FmModelMemberBase*>& toBeFilled, int typeID,
                              const std::vector<int>& butNotOfTypes,
                              const std::string& tagged, const FmHeadMap* root);

  static const FmHeadMap* getHeadMap(const FmHeadMap* root,
				     std::vector<int>::const_iterator it,
				     std::vector<int>::const_iterator end,
				     int parentAssemblyID = 0);

  static void displayMembers(int typeID, const FmHeadMap* root);
};

#endif
