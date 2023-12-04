// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ANALYSIS_H
#define FM_ANALYSIS_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmSolverConvergence.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmVTFType.H"
#include "FFaLib/FFaAlgebra/FFaVec3.H"
#include "FFaLib/FFaContainers/FFaReference.H"


typedef std::pair<double,double> FmPair; //!< For stress ranges, etc.


class FmAnalysis : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmAnalysis();

  virtual const char* getUITypeName() const { return "Analysis"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual void initAfterResolve();

  void setGravity(const FaVec3& vec);
  void setWaveDir(const FaVec3& vec);
  void setMeanSeaLevel(double z0);

  bool setQuasistaticUpToTime(double var);
  bool setStartTime(double var);
  bool setEndTime(double var);
  bool setTimeIncrement(double var);
  bool setMinTimeIncrement(double var);
  double getMinTimeIncrement() const;

  void setEngine(FmEngine* engine) { myTimeIncEngine.setRef(engine); }
  FmEngine* getEngine() const { return myTimeIncEngine.getPointer(); }

  bool setNewmarkDamping(double alpha_f, double alpha_m = 0.0);
  bool setForceNumIt(int var);
  bool setMaxNumIt(int var);
  bool setMinNumIt(int var);
  bool setMinMatrixUpdates(int var);
  bool setMaxSequentialNoMatrixUpdates(int var);
  bool setTolMatrixUpdateFactor(double var);

  bool setRelativePropTolerance(double var);
  bool setTolDisplacementNorm(double var, int toggle = -1);
  bool setTolDisplacementTra(double var, int toggle = -1);
  bool setTolDisplacementRot(double var, int toggle = -1);
  bool setTolVelocityNorm(double var, int toggle = -1);
  bool setTolResidualNorm(double var, int toggle = -1);
  bool setTolResidualTra(double var, int toggle = -1);
  bool setTolResidualRot(double var, int toggle = -1);
  bool setTolEnergyMax(double var, int toggle = -1);
  bool setTolEnergySum(double var, int toggle = -1);

  bool setSolveEigenvalueFlag(bool var);
  bool setRequestedEigenmodes(int val);
  bool setEigenvalueSolutionInterval(int var);
  bool setEigenvalueSolutionTimeInterval(double var);

  bool setStaticEqulTol(double var);
  bool setIterationStepReductionFactor(double var);

  bool setRampSteps(int var);
  bool setRampPause(double var);
  bool setRampShape(double Vmax, double T);

  int  getNumberOfStressTimeSteps() const;
  void getEigenvalueSamples(DoubleVec& sampleArray) const;
  void getEigenvalueList(IntVec& modesList) const;

  bool needMassMatrix() const;

  std::string getProcessPath() const;

protected:
  virtual ~FmAnalysis();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmAnalysis* obj);

private:
  // Model preferences (moved to FmMechanism in R5.1)
  FFaObsoleteField<double> positionTolerance;
  FFaObsoleteField<FaVec3> gravity;
  FFaObsoleteField<FaVec3> initVel;

public:

  // Time stepping options

  FFaField<bool> dynamicsEnable;
  FFaField<bool> stopTimeEnable;
  FFaField<bool> quasistaticEnable;
  FFaField<bool> quasistaticMode;
  FFaField<double> quasistaticUpToTime;

  FFaField<double> startTime;
  FFaField<double> stopTime;
  FFaField<double> timeIncr;
  FFaField<double> minTimeIncr;

  FFaReference<FmEngine>      myTimeIncEngine;
  FFaField<FFaReferenceBase*> myTimeIncEngineField;

  FFaField<bool>   doCutback;
  FFaField<double> cutbackFactor;
  FFaField<int>    cutbackSteps;

  FFaField<bool>   doRestart;
  FFaField<double> restartTime;

  // Time integration options

  enum NewmarkDamping {
    NONE,
    HHT_ALPHA,
    GENERALIZED_ALPHA
  };

  FFaEnumMapping(NewmarkDamping) {
    FFaEnumEntry(NONE, "NONE");
    FFaEnumEntry(HHT_ALPHA, "HHT_ALPHA");
    FFaEnumEntry(GENERALIZED_ALPHA, "GENERALIZED_ALPHA");
    FFaEnumEntryEnd;
  };

  FFaField<NewmarkDampingEnum> newmarkDamping;
  FFaField<FmPair>             newmarkFactors;

  FFaField<bool>   ignoreTolerance;
  FFaField<int>    fixedNumIt;
  FFaField<int>    minNumIt;
  FFaField<int>    maxNumIt;

  FFaField<bool>   useFixedMatrixUpdates;
  FFaField<int>    minMatrixUpdates;
  FFaField<int>    maxSequentialNoMatrixUpdates;
  FFaField<double> tolMatrixUpdateFactor;

  FFaField<bool>   useDynStressStiffening;
  FFaField<bool>   useMassCorrection;

  FFaField<int>    defaultShadowPosAlg;
  FFaField<double> shadowPosTol;

  // Convergence tolerance options

  FFaField<double>              tolVelProp;
  FFaField<FmSolverConvergence> tolDisplacementNorm;
  FFaField<FmSolverConvergence> tolDisplacementTra;
  FFaField<FmSolverConvergence> tolDisplacementRot;
  FFaField<FmSolverConvergence> tolVelocityNorm;
  FFaField<FmSolverConvergence> tolResidualNorm;
  FFaField<FmSolverConvergence> tolResidualTra;
  FFaField<FmSolverConvergence> tolResidualRot;
  FFaField<FmSolverConvergence> tolEnergyMax;
  FFaField<FmSolverConvergence> tolEnergySum;

  // Eigenmode options

  FFaField<bool>   solveEigenvalues;
  FFaField<int>    numEigenmodes;
  FFaField<double> eigenSolveTimeInterval;
  FFaField<double> eigenvalueShiftFactor;
  FFaField<bool>   useBCsOnEigenvalues;
  FFaField<bool>   dampedEigenvalues;
  FFaField<bool>   useEigStressStiffening;

  // Frequency domain options

  FFaField<bool>   solveFrequencyDomain;

  // Initial equilibrium options

  FFaField<bool>   solveInitEquil;
  FFaField<double> staticEqlTol;
  FFaField<double> iterStepLimit;
  FFaField<bool>   useEquStressStiffening;

  FFaField<bool>   smoothRamp;
  FFaField<bool>   rampGrav;
  FFaField<int>    rampSteps;
  FFaField<double> rampVmax;
  FFaField<double> rampLength;
  FFaField<double> rampPause;

  // Auto-export options

  FFaField<bool>        autoCurveExportSwitch;
  FFaField<std::string> autoCurveExportFileName;

  enum ExportFileFormatType {
    RPC_LITTLE_ENDIAN,
    RPC_BIG_ENDIAN,
    ASCII_MULTI_COLUMN
  };

  FFaEnumMapping(ExportFileFormatType) {
    FFaEnumEntry(RPC_LITTLE_ENDIAN, "RPC_PC_FORMAT");
    FFaEnumEntry(RPC_BIG_ENDIAN,    "RPC_UNIX_FORMAT");
    FFaEnumEntry(ASCII_MULTI_COLUMN,"ASCII_MULTI_COLUMN");
    FFaEnumEntryEnd;
  };

  FFaField<ExportFileFormatTypeEnum> autoCurveExportFileFormat;

  FFaField<bool>            autoSolverVTFExport;
  FFaField<std::string>     solverVTFname;
  FFaField<VTFFileTypeEnum> solverVTFtype;

  FFaField<bool>   autoAnimateSwitch;

  FFaField<bool>   overwriteResults;
  FFaField<bool>   overwriteFEParts;

  // Stress recovery options

  FFaField<double> stressStartTime;
  FFaField<double> stressStopTime;
  FFaField<double> stressTimeIncr;

  FFaField<bool> stressAllTimeSteps;
  FFaField<bool> stressDeformation;
  FFaField<bool> stressStrainTensor;
  FFaField<bool> stressStressTensor;
  FFaField<bool> stressSRTensor;
  FFaField<bool> stressVMstrain;
  FFaField<bool> stressVMstress;
  FFaField<bool> stressMaxPstrain;
  FFaField<bool> stressMaxPstress;
  FFaField<bool> stressMinPstrain;
  FFaField<bool> stressMinPstress;
  FFaField<bool> stressMaxSstrain;
  FFaField<bool> stressMaxSstress;

  FFaField<bool>            autoStressVTFExport;
  FFaField<std::string>     stressVTFname;
  FFaField<VTFFileTypeEnum> stressVTFtype;
  FFaField<FmPair>          stressVTFrange;

  // Additional solver options

  FFaField<std::string> reducerAddOpts;
  FFaField<std::string> solverAddOpts;
  FFaField<std::string> stressAddOpts;

  FFaField<bool>   useRamSizeGSF;
  FFaField<bool>   autoRamSizeGSF;
  FFaField<int>    ramSizeGSF;

  FFaField<bool>   useRamSizeBmat;
  FFaField<bool>   autoRamSizeBmat;
  FFaField<int>    ramSizeBmat;

  FFaField<int>    maxConcurrentProcesses;

  FFaField<bool>   useProcessPrefix;
  FFaField<bool>   useProcessPath;
  FFaField<std::string> processPrefix;
  FFaField<std::string> processPath;

  FFaField<std::string> cloudAppId;

  FFaField<bool>        useExternalFuncFile;
  FFaField<std::string> externalFuncFileName;
};

#endif
