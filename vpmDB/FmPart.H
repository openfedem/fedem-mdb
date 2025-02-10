// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PART_H
#define FM_PART_H

#include "vpmDB/FmLink.H"
#include "vpmDB/FmResultStatusData.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmMaterialProperty.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"
#include "FFaLib/FFaAlgebra/FFaCheckSum.H"
#include "FFaLib/FFaAlgebra/FFaTensor3.H"


template<>
inline bool FFaField<FmResultStatusData>::isPrintable() const
{
  return !this->isDefault();
}


class FmElementGroupProxy;
class FFlConnectorItems;
class FFaCompoundGeometry;
#ifdef FF_NAMESPACE
namespace FF_NAMESPACE {
#endif
class FFlLinkHandler;
class FFlNode;
#ifdef FF_NAMESPACE
}
using FFlLinkHandler = FF_NAMESPACE::FFlLinkHandler;
using FFlNode        = FF_NAMESPACE::FFlNode;
#endif
using FaVec3Vec = std::vector<FaVec3>;


class FmPart : public FmLink
{
  Fmd_DB_HEADER_INIT();

public:
  FmPart(const FaVec3& globalPos = FaVec3());
  FmPart(const char* EarthName);

protected:
  virtual ~FmPart();

  FmPart(const FmPart&) = delete;
  FmPart operator=(const FmPart&) = delete;

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool isSilent);
  virtual std::pair<FmTriad*,bool> getExistingTriad(const FmTriad* triad);
  virtual bool isAttachable() const;

public:

  // Utility methods

  virtual bool interactiveErase();
  virtual bool isDisabled() const;
  virtual bool isSuppressed() const { return suppressInSolver.getValue(); }
  virtual bool isGenericPart() const { return useGenericProperties.getValue(); }
  virtual bool isFEPart(bool loadedOnly = false) const;

  FmTriad* getTriadAtNode(int nodeNo) const;
  FFlNode* getNodeAtPoint(const FaVec3& point, double tolerance,
                          FFlConnectorItems* addItems = NULL);
  int getNodeIDAtPoint(const FaVec3& point, double tolerance);
  FFlNode* getNode(int nodeNo) const;
  int getNodePos(int nodeNo,
                 double* x = NULL, double* y = NULL, double* z = NULL) const;
  FFlNode* getClosestNode(const FaVec3& point) const;

  int  getCompModesFlags(IntVec& BCcodes) const;
  bool getCompModesAlpha(DoubleVec& alpha, int type) const;

  bool hasLoads() const;
  void printSolverLoads(FILE* fp) const;

  bool mergeGenericParts(FmPart* that);

  virtual bool getBBox(FaVec3& max, FaVec3& min) const;

  bool updateTriadTopologyRefs(bool checkUnloaded = true, char useOutput = 3);

  virtual const char* getUITypeName() const { return "Part"; }

  std::string getTaskName(const char* fmt = "%d_") const;

  // Modelling DOF filter
  enum DofFilter { ALL_DOFS = 0, THREE_DOFS = 3, SIX_DOFS = 6 };

  // DB Topology

  int getNumberOfTriads() const;
  FmTriad* findTriad(int baseID) const;
  void getTriads(std::vector<FmTriad*>& toFill, bool sortOnId = false) const;

  void getElementGroups(std::vector<FmElementGroupProxy*>& groups) const;
  bool setVisDetail(const std::vector<FmElementGroupProxy*>& groups, int dType);

  void getFreeJointEnds(std::vector<FmTriad*>& toFill) const;

  virtual void getStickers(std::vector<FmSticker*>& toFill) const;

  void setCGPosRef(FmIsPositionedBase* ref);
  void setCGRotRef(FmIsPositionedBase* ref);

  FmIsPositionedBase* getCGPosRef() const { return myCGPosRef; }
  FmIsPositionedBase* getCGRotRef() const { return myCGRotRef; }
  const FFa3DLocation& getLocationCG() const { return myCG.getValue(); }
  void setLocationCG(const FFa3DLocation& cg);

  virtual double getMass() const { return mass.getValue(); }
  virtual FaMat34 getPositionCG(bool globalCS = true) const;
  void setPositionCG(const FaVec3& CoG, bool edited = false);
  void setLocationCG(const FaVec3& CoG, const FaVec3& Iaxes);
  void setOrientationCG(const FaVec3& Xaxis, const FaVec3& XYplane);

  virtual void setLocalCS(const FaMat34& localCS);

  virtual bool isRotatable() const { return this->isTranslatable(); }
  virtual bool isTranslatable() const;

  // FE mesh generation
  bool isMeshable() const;
  bool createFEData(bool parabolic);
  bool getMeshParams(int* nnod, int* nel = NULL,
                     bool* parabolic = NULL, int* ndof = NULL,
                     std::string* elmTypeCount = NULL) const;

  // File handling
  bool openFEData();
  bool saveFEData(bool forceSave = false);
  bool setVisualizationFile(const std::string& fileName, bool updateViz = true);
  virtual bool writeToVTF(VTFAFile& file, IntVec* outputOrder, IntVec* fstOrdNodes);
  bool hasOP2files() const;
  bool convertOP2files(const std::string& filePath);
  bool copyExternalFiles(const std::string& from, const std::string& to);
  static std::string locateFEfile(const std::string& fileName);
  bool importPart(const std::string& fileName, const FFaUnitCalculator* conv,
                  bool storeRelativePath = false,
                  bool autoRefNodeTriads = false);
  bool exportPart(const std::string& fileName, bool extNodeInfo = true,
                  bool withCheckSum = false, bool noMetaData = false) const;
  bool isSaved(bool checkFEpartsOnly = true) const;
  bool isFELoaded(bool fullOnly = false) const;
  bool useFEModelAsVisualization() const;

  FFlLinkHandler* getLinkHandler() const { return myFEData; }
  void setLinkHandler(FFlLinkHandler* data, bool updateNnodes = true);

  double getConnectorTolerance() const;
  char isTriadConnectable(FmTriad* triad) const;
#ifdef FT_USE_CONNECTORS
  void updateConnectorVisualization();
  bool createConnector(const IntVec& nodes, const FaVec3& refNodePos,
                       FmTriad* triad = NULL, int spiderType = 0);
  bool createConnector(const FFaCompoundGeometry& geometry,
                       const FaVec3& refNodePos, const FaMat33& refNodeCS,
                       FmTriad* triad = NULL, int spiderType = 0);
  bool createCylinderConnector(const FaVec3Vec& cylPoints, bool useArcOnly,
                               const FaVec3* refNodePos = NULL,
                               bool projectRefNodeToAxis = false,
                               FmTriad* triad = NULL, int spiderType = 2);
  bool createPlaneConnector(const FaVec3Vec& planePoints,
                            const FaVec3* refNodePos = NULL,
                            bool projectRefNodeToPlane = false,
                            FmTriad* triad = NULL, int spiderType = 2);
  bool createLineConnector(const FaVec3Vec& linePoints,
                           const FaVec3* refNodePos = NULL,
                           bool projectRefNodeToAxis = false,
                           FmTriad* triad = NULL, int spiderType = 2);
#endif

  void updateCachedCheckSum();
  void delayedCheckSumUpdate() { needsCSupdate.setValue(true); }
  void getCheckSum(FFaCheckSum& cs);
  void forceSave() { savedCS.setValue(0); }
  bool enforceStrainRosetteRecovery();
  bool hasChangedFEdata() const;
  bool hasStrainRosettes() const;
  bool hasStrainCoat() const;
  bool hasResults() const;

  void readyForUpdate(bool useExistingFmx = false);
  void clearSupelFiles(bool includeFTL = true);
  void clearElemGroupProxies(bool doErase = false);
  void createElemGroupProxies();
  void updateElemGroupProxies();
  void updateMassProperties();
  void updateLoadCases();

  //! \brief Syncronizes the FE part RSD with data found on disk.
  bool syncRSD(bool askForMissingFiles = false);

  enum ReposType { INTERNAL_REP = 0, EXTERNAL_REP, LINK_SPECIFIC };

  ReposType usesRepository() const;
  std::string getAbsFilePath(bool createDir = false) const;
  std::string getBaseFTLFile(bool createDir = false) const;
  std::string getBaseFTLName(bool includExt = false) const;

  const std::string& setValidBaseFTLFile(unsigned int myCS = 0);

  static int checkParts();

  // Basic DB methods

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual FmBase* duplicate() const;

private:
  bool renewFEmodel();

  static bool locateOriginalFEfile(std::string& fileName, bool& useUnitCalc);

public:

  // Files present in RDB

  FFaField<std::string> BMatFile;
  FFaField<std::string> EMatFile;
  FFaField<std::string> GMatFile;
  FFaField<std::string> MMatFile;
  FFaField<std::string> SMatFile;
  FFaField<std::string> LMatFile;
  FFaField<std::string> DMatFile;
  FFaField<std::string> FMatFile;
  FFaField<std::string> SAMdataFile;
  FFaField<std::string> reducedFTLFile;
  FFaField<FmResultStatusData> myRSD;

  FFaField<std::string>       originalFEFile;  //!< Full name of imported file
  FFaField<FFaUnitCalculator> importConverter; //!< Converter to modeling units

  FFaField<bool>        externalSource;
  FFaField<std::string> myRepository;
  FFaField<std::string> baseFTLFile; //!< The part file basename actually used

  // Options

  enum RamLevel {
    NOTHING = 0,
    REDUCED_VIZ,
    SURFACE_FE,
    FULL_FE
  };

  FFaEnumMapping(RamLevel) {
    FFaEnumEntry(NOTHING,    "NOTHING");
    FFaEnumEntry(REDUCED_VIZ,"REDUCED_VIZ");
    FFaEnumEntry(SURFACE_FE, "SURFACE_FE");
    FFaEnumEntry(FULL_FE,    "FULL_FE");
    FFaEnumEntryEnd;
  };

  FFaField<RamLevelEnum> ramUsageLevel;

  enum LockLevel {
    FM_ALLOW_MODIFICATIONS,
    FM_ALLOW_LINK_EXT_NODE_MOD,
    FM_DENY_ALL_LINK_MOD,
    FM_DENY_LINK_USAGE
  };

  FFaNoPrintField<int>    lockLevel;
  FFaField<bool>          suppressInSolver;     //!< ON-OFF toggle

  // Mesh control parameters

  FFaField<int>    minSize;
  FFaField<double> quality;

  FFaReference<FmMaterialProperty> material;
  FFaField<FFaReferenceBase*>      materialField;

  // Generic part properties (alternative to FE data)

  FFaField<bool>          useGenericProperties; //!< ON-OFF toggle
  FFaField<bool>          condenseOutCoG;       //!< Should the CoG DOFs be condensed out?
  FFaField<double>        kt;                   //!< Translational stiffness at each triad
  FFaField<double>        kr;                   //!< Rotational stiffness at each triad
  FFaField<double>        mass;                 //!< Total mass
  FFaField<FFaTensor3>    inertia;              //!< Mass inertia about the centre of gravity

  enum MassCalculation {
    EXPLICIT,
    FROM_GEOMETRY,
    FROM_FEM
  };

  FFaEnumMapping(MassCalculation) {
    FFaEnumEntry(EXPLICIT     ,"EXPLICIT");
    FFaEnumEntry(FROM_GEOMETRY,"FROM_GEOMETRY");
    FFaEnumEntry(FROM_FEM     ,"FROM_FEM");
    FFaEnumEntryEnd;
  };

  FFaField<MassCalculationEnum> myCalculateMass;

  enum InertiaRef {
    POS_CG_ROT_CS,
    POS_CG_ROT_CG
  };

  FFaEnumMapping(InertiaRef) {
    FFaEnumEntry(POS_CG_ROT_CS,"POS_CG_ROT_CS");
    FFaEnumEntry(POS_CG_ROT_CG,"POS_CG_ROT_CG");
    FFaEnumEntryEnd;
  };

  FFaField<InertiaRefEnum> myInertiaRef;

  enum GenericPartStiffType {
    DEFAULT_RIGID,
    NODE_STIFFNESS
  };

  FFaEnumMapping(GenericPartStiffType) {
    FFaEnumEntry(DEFAULT_RIGID, "DEFAULT_RIGID");
    FFaEnumEntry(NODE_STIFFNESS,"NODE_STIFFNESS");
    FFaEnumEntryEnd;
  };

  FFaField<GenericPartStiffTypeEnum> myGenericPartStiffType;

  // Reducer options:

  // Ignore checksum for reducer and solver - only check file presence
  FFaField<bool>   overrideChecksum;

  FFaField<int>    nGenModes;
  FFaField<int>    nEigvalsCalc;
  FFaField<bool>   expandModeShapes;

  FFaField<bool>   useConsistentMassMatrix;
  FFaField<bool>   factorizeMassMxEigSol;

  enum FloatPrecision {
    SINGLE_PRECISION = 1,
    DOUBLE_PRECISION = 2
  };

  FFaEnumMapping(FloatPrecision) {
    FFaEnumEntry(SINGLE_PRECISION,"SINGLE_PRECISION");
    FFaEnumEntry(DOUBLE_PRECISION,"DOUBLE_PRECISION");
    FFaEnumEntryEnd;
  };

  FFaField<FloatPrecisionEnum> recoveryMatrixSavePrecision;

  FFaField<double> tolFactorize;
  FFaField<double> tolEigenval;

  // Parameters related to centripital force correction

  enum CentripOption {
    MODEL_DEFAULT_CENTRIP_CORRECTION,
    USE_CENTRIP_CORRECTION,
    NO_CENTRIP_CORRECTION
  };

  FFaEnumMapping(CentripOption) {
    FFaEnumEntry(MODEL_DEFAULT_CENTRIP_CORRECTION,"MODEL_DEFAULT");
    FFaEnumEntry(USE_CENTRIP_CORRECTION,          "USE_CENTRIP_CORRECTION");
    FFaEnumEntry(NO_CENTRIP_CORRECTION,           "NO_CENTRIP_CORRECTION");
    FFaEnumEntryEnd;
  };

  FFaField<CentripOptionEnum> myCentripOption;

  // Parameters related to distributed loads on FE model

  FFaReferenceList<FmEngine>      myLoadEngines;
  FFaField<FFaReferenceListBase*> myLoadEnginesField;
  FFaField<DoubleVec>             myLoadDelays;
  FFaField<DoubleVec>             myLoadFactors;
  FFaField<IntVec>                myLoadCases;

  // Parameters related to stress recovery

  FFaField<int>         recoveryDuringSolve;
  FFaField<bool>        useExternalResFile;
  FFaField<std::string> externalResFileName;

  // Parameters related to nonlinear behaviour

  static bool           enableNonlinearFE;
  FFaField<bool>        useNonlinearReduction;
  FFaField<int>         numberOfNonlinearSolutions;
  FFaField<int>         nonLinStates;
  FFaField<std::string> nonlinearDataFileName;

  // Hydrodynamic properties

  FFaField<bool> hasBuoyancy;

private:
  static long int nFENodesTotal; //!< Total model size (summed over all parts)

  FFlLinkHandler*               myFEData; //!< The actual FE model data
  FFaNoPrintField<bool>    needsCSupdate; //!< To delay checksum calculation
  FFaField<FFaCheckSum>   cachedChecksum; //!< Used when FE model is unloaded
  FFaNoPrintField<unsigned long> savedCS; //!< The last saved checksum value

  FFaField<FFa3DLocation>          myCG;
  FFaReference<FmIsPositionedBase> myCGPosRef;
  FFaField<FFaReferenceBase*>      myCGPosRefField;
  FFaReference<FmIsPositionedBase> myCGRotRef;
  FFaField<FFaReferenceBase*>      myCGRotRefField;

  bool isCGedited; //!< true, when the CG has been edited manually
  int fileVersion; //!< Version number of the saved FTL-file
};

#endif
