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


//! \cond TEMPLATE_SPECIALIZATION
template<>
inline bool FFaField<FmResultStatusData>::isPrintable() const
{
  return !this->isDefault();
}
//! \endcond


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

using FaVec3Vec = std::vector<FaVec3>; //!< Convenience type alias


class FmPart : public FmLink
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
  //! \brief Default constructor.
  FmPart(const FaVec3& globalPos = FaVec3());
  //! \brief Constructor used for creating the earth link only.
  FmPart(const char* EarthName);

protected:
  //! \brief The destructor disconnects the part from the model database.
  virtual ~FmPart();

  //! \brief Deleted copy constructor.
  FmPart(const FmPart&) = delete;
  //! \brief Deleted assignment operator.
  FmPart operator=(const FmPart&) = delete;

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

  //! \brief Attaching a triad when having an old triad in the same place.
  bool attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool isSilent) final;
  //! \brief Returns a triad co-located with another triad, if any.
  std::pair<FmTriad*,bool> getExistingTriad(const FmTriad* triad) final;
  //! \brief Returns \e true if external node status changes are allowed.
  bool isAttachable() const final;

public:

  // Utility methods

  // From FmModelMemberBase
  bool interactiveErase() override;

  // From FmLink
  bool isDisabled() const final;
  bool isSuppressed() const final { return suppressInSolver.getValue(); }
  bool isGenericPart() const final { return useGenericProperties.getValue(); }
  //! \brief Returns \e true if this is a used (and loaded) FE part.
  bool isFEPart(bool loadedOnly = false) const final;

  FaMat34 getTransform() const;

  //! \brief Returns the triad associated with given FE node, or NULL if none.
  FmTriad* getTriadAtNode(int nodeNo) const;
  //! \brief Returns the closest FE node to a given point, or NULL if not found.
  FFlNode* getNodeAtPoint(const FaVec3& point, double tolerance,
                          FFlConnectorItems* addItems = NULL);
  int getNodeIDAtPoint(const FaVec3& point, double tolerance);
  FFlNode* getNode(int nodeNo) const;
  int getNodePos(int nodeNo,
                 double* x = NULL, double* y = NULL, double* z = NULL) const;
  bool getNodeConnectivity(int nodeNo, Strings& elements) const;
  FFlNode* getClosestNode(const FaVec3& point) const;

  int  getCompModesFlags(IntVec& BCcodes) const;
  bool getCompModesAlpha(DoubleVec& alpha, int type) const;

  //! \brief Checks if this part has any distributed loads.
  bool hasLoads() const;
  //! \brief Writes distributed load data to the solver input file.
  void printSolverLoads(FILE* fp) const;

  bool mergeGenericParts(FmPart* that);

  // From FmLink
  bool getBBox(FaVec3& max, FaVec3& min) const final;

  //! \brief Updates triad to FE node connection after (re)loading the FE model.
  bool updateTriadTopologyRefs(bool checkUnloaded = true, char useOutput = 3);

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

  // From FmIsPositionedBase
  void getStickers(std::vector<FmSticker*>& toFill) const override;

  bool setCGPosRef(FmIsPositionedBase* ref);
  bool setCGRotRef(FmIsPositionedBase* ref);

  FmIsPositionedBase* getCGPosRef() const { return myCGPosRef; }
  FmIsPositionedBase* getCGRotRef() const { return myCGRotRef; }
  const FFa3DLocation& getLocationCG() const { return myCG.getValue(); }
  void setLocationCG(const FFa3DLocation& cg);

  // From FmLink
  double getMass() const final { return mass.getValue(); }

  //! \brief Returns the Center of Gravity position for this part.
  //! \param[in] globalCS If \e false, use part coordinates, otherwise global
  FaMat34 getPositionCG(bool globalCS = true) const override;
  void setPositionCG(const FaVec3& CoG, bool edited = false);
  void setLocationCG(const FaVec3& CoG, const FaVec3& Iaxes);
  void setOrientationCG(const FaVec3& Xaxis, const FaVec3& XYplane);

  // From FmIsPositionedBase
  void setLocalCS(const FaMat34& localCS) override;
  char isRotatable() const override { return this->isTranslatable(); }
  bool isTranslatable() const override;

  // FE mesh generation
  bool isMeshable() const;
  //! \brief Generates a tetrahedron mesh from CAD data, if any.
  bool createFEData(bool parabolic);
  bool getMeshParams(int* nnod, int* nel = NULL,
                     bool* parabolic = NULL, int* ndof = NULL,
                     std::string* elmTypeCount = NULL) const;

  // File handling

  //! \brief Reads the FE data from a file already in the part DB.
  bool openFEData();
  bool saveFEData(bool forceSave = false);
  bool setVisualizationFile(const std::string& fileName, bool updateViz = true);
  //! \brief Writes the FE data of this part to the specified \a vtfFile.
  bool writeToVTF(VTFAFile& vtfFile,
                  IntVec* outputOrder, IntVec* fstOrdNodes) override;
  bool hasOP2files() const;
  bool convertOP2files(const std::string& filePath);
  bool copyExternalFiles(const std::string& from, const std::string& to);
  static std::string locateFEfile(const std::string& fileName);
  //! \brief Imports FE part data from an external source.
  bool importPart(const std::string& fileName,
                  const FFaUnitCalculator* conv = NULL,
                  bool storeRelativePath = true,
                  bool autoRefNodeTriads = false,
                  bool isReloading = false);
  //! \brief Writes the FE data of this part to the specified \a ftlFile
  bool exportPart(const std::string& ftlFile, bool extNodeInfo = true,
                  bool withCheckSum = false, bool noMetaData = false) const;
  //! \brief Returns \e true if the part is saved in its associated repository.
  bool isSaved(bool checkFEpartsOnly = true) const;
  //! \brief Returns \e true if the part's FE model currently is loaded.
  bool isFELoaded(bool fullOnly = false) const;
  //! \brief Returns \e true if the part's FE model is used for visualization.
  bool useFEModelAsVisualization() const;

  //! \brief Returns the current FE data model for this part.
  FFlLinkHandler* getLinkHandler() const { return myFEData; }
  //! \brief Assigns a new FE data model to the part.
  void setLinkHandler(FFlLinkHandler* data, bool eraseGroupProxies = false);

#ifdef FT_USE_CONNECTORS
  //! \brief Returns the geometric tolerance used when creating connectors.
  double getConnectorTolerance() const;
  //! \brief Checks if \a triad can be attached and used as a connector triad.
  char isTriadConnectable(FmTriad* triad) const;
  //! \brief Updates the visualization of the surface connectors in this part.
  void updateConnectorVisualization();
  //! \brief Creates a connector to the given \a nodes at specified location.
  bool createConnector(const IntVec& nodes, const FaVec3& refNodePos,
                       FmTriad* triad = NULL, int spiderType = 0);
  //! \brief Creates a connector at the specified location.
  bool createConnector(const FFaCompoundGeometry& geometry,
                       const FaVec3& refNodePos, const FaMat33& refNodeCS,
                       FmTriad* triad = NULL, int spiderType = 0);
  //! \brief Creates a circular/cylindric connector at the specified location.
  bool createCylinderConnector(const FaVec3Vec& cylPoints, bool useArcOnly,
                               const FaVec3* refNodePos = NULL,
                               bool projectRefNodeToAxis = false,
                               FmTriad* triad = NULL, int spiderType = 2);
  //! \brief Creates a plane connector at the specified location.
  bool createPlaneConnector(const FaVec3Vec& planePoints,
                            const FaVec3* refNodePos = NULL,
                            bool projectRefNodeToPlane = false,
                            FmTriad* triad = NULL, int spiderType = 2);
  //! \brief Creates a line connector at the specified location.
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

  //! \brief Clears data related to the FE model before importing a new one.
  void readyForUpdate(bool useExistingFmx = false);
  //! \brief Clears all file paths derived from #originalFEFile.
  void clearSupelFiles(bool includeFTL = true);
  //! \brief Creates proxy element groups based on the groups in the FE model.
  void createElemGroupProxies();
  //! \brief Updates element groups to be in sync with the FE model of the part.
  void updateElemGroupProxies();
  //! \brief Recalculates the mass properties of the part from its FE model.
  void updateMassProperties();
  //! \brief Updates the load case data from the FE part data object.
  void updateLoadCases();

  //! \brief Syncronizes the FE part RSD with data found on disk.
  bool syncRSD(bool askForMissingFiles = false);

  enum ReposType { INTERNAL_REP = 0, EXTERNAL_REP, LINK_SPECIFIC };

  //! \brief Checks if the part data files are saved in an external repository.
  ReposType usesRepository() const;
  //! \brief Returns the path to the directory in which the part will be saved.
  std::string getAbsFilePath(bool createDir = false) const override;
  //! \brief Returns the full path of the saved FE data file of this part.
  std::string getBaseFTLFile(bool createDir = false) const;
  //! \brief Returns the base ftl-file name of this part.
  std::string getBaseFTLName(bool includExt = false) const;

  //! \brief Assigns a valid filename for storing the FE data of this part.
  const std::string& setValidBaseFTLFile(unsigned int cs = 0);

  //! \brief Checks the consistency of the part topology.
  static int checkParts();

  // Basic DB methods

  //! \brief Creates a new part object by reading data from a stream.
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

  // From FmBase
  std::ostream& writeFMF(std::ostream& os = std::cout) override;
  bool clone(FmBase* obj, int depth) override;
  const char* getUITypeName() const override { return "Part"; }

  // From FmSimulationModelBase
  void initAfterResolve() override;

  // From FmModelMemberBase
  FmBase* duplicate() const override;

private:
  //! \brief Allocates a new FFlLinkHandler object for the storage of FE data.
  bool renewFEmodel();

  //! \brief Private method searching for the original FE file of a part.
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

  FFaField<bool>        externalSource; //!< Externally reduced FE data?
  FFaField<std::string> myRepository; //!< Part-specific model data repository
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

  //! Ignore checksum for reducer and solver - only check file presence
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

  // Parameters related to centripetal force correction

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
  FFaField<bool>        ignoreInRecovery;
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
  FFlLinkHandler*               myFEData; //!< The actual FE model data
  FFaNoPrintField<bool>    needsCSupdate; //!< To delay checksum calculation
  FFaField<FFaCheckSum>   cachedChecksum; //!< Used when FE model is unloaded
  FFaNoPrintField<unsigned long> savedCS; //!< The last saved checksum value

  FFaField<FFa3DLocation>          myCG;
  FFaReference<FmIsPositionedBase> myCGPosRef;
  FFaField<FFaReferenceBase*>      myCGPosRefField;
  FFaReference<FmIsPositionedBase> myCGRotRef;
  FFaField<FFaReferenceBase*>      myCGRotRefField;

  bool isCGedited; //!< If \e true, the CG has been edited manually
  int fileVersion; //!< Version number of the saved FTL-file
};

#endif
