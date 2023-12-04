// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmModesOptions.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmPart.H"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include <numeric>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcANALYSIS, FmAnalysis, FmSimulationModelBase);


FmAnalysis::FmAnalysis()
{
  Fmd_CONSTRUCTOR_INIT(FmAnalysis);

  // Add the base ID to the list of fields to be saved in the model file, only
  // to avoid warning because it is present in the template file (default.fmm).
  FFA_FIELD_INIT(myBaseID, -1, "BASE_ID");

  // Default model preferences

  FFA_FIELD_INIT(positionTolerance,1.0e-4,"POSITION_TOLERANCE");
  FFA_FIELD_INIT(gravity,FaVec3(0.0,0.0,-9.81),"GRAVITY");
  FFA_FIELD_DEFAULT_INIT(initVel,"GLOBAL_INITIAL_VELOCITY");

  // Default analysis options

  FFA_FIELD_INIT(dynamicsEnable, true, "DYNAMICS_SOLVER_ENABLE");
  FFA_FIELD_INIT(stopTimeEnable, true, "STOP_TIME_ENABLE");
  FFA_FIELD_INIT(quasistaticEnable, false, "QUASISTATIC_ENABLE");
  FFA_FIELD_INIT(quasistaticMode, false, "QUASISTATIC_MODE");
  FFA_FIELD_INIT(quasistaticUpToTime, 0.0, "QUASISTATIC_UPTOTIME");

  FFA_FIELD_INIT(startTime,  0.0,  "START_TIME");
  FFA_FIELD_INIT(stopTime,   1.0,  "END_TIME");
  FFA_FIELD_INIT(timeIncr,   0.01, "TIME_INCR");
  FFA_FIELD_INIT(minTimeIncr,0.001,"MIN_TIME_INCREMENT");
  FFA_FIELD_INIT(doCutback,  false,"CUTBACK");
  FFA_FIELD_INIT(cutbackFactor,0.5,"CUTBACK_FACTOR");
  FFA_FIELD_INIT(cutbackSteps, 1,  "CUTBACK_STEPS");
  FFA_FIELD_INIT(doRestart,  false,"RESTART");
  FFA_FIELD_INIT(restartTime,0.0,  "RESTART_TIME");

  FFA_REFERENCE_FIELD_INIT(myTimeIncEngineField,myTimeIncEngine,"TIME_INCR_ENGINE");
  myTimeIncEngine.setPrintIfZero(false);

  FFA_FIELD_INIT(newmarkDamping,HHT_ALPHA,      "NEWMARK_DAMPING");
  FFA_FIELD_INIT(newmarkFactors,FmPair(0.1,0.0),"NEWMARK_ALPHA_FACTORS");

  FFA_FIELD_INIT(ignoreTolerance,false,"IGNORE_TOLERANCE");
  FFA_FIELD_INIT(fixedNumIt,      5,   "FORCE_NUM_IT");
  FFA_FIELD_INIT(minNumIt,        1,   "MINIMUM_NUM_IT");
  FFA_FIELD_INIT(maxNumIt,       15,   "MAXIMUM_NUM_IT");

  FFA_FIELD_INIT(useFixedMatrixUpdates,      true,"USE_FIXED_MATRIX_UPDATES");
  FFA_FIELD_INIT(minMatrixUpdates,              5,"MIN_MATRIX_UPDATES");
  FFA_FIELD_INIT(maxSequentialNoMatrixUpdates,100,"MAX_SEQUENTIAL_NO_MATRIX_UPDATES");
  FFA_FIELD_INIT(tolMatrixUpdateFactor,    1000.0,"TOL_MATRIX_UPDATE_FACTOR");

  FFA_FIELD_INIT(useDynStressStiffening,false,"USE_DYN_STRESS_STIFFENING");
  FFA_FIELD_INIT(useMassCorrection,     true, "USE_MASS_CORRECTION");

  FFA_FIELD_INIT(defaultShadowPosAlg,1,"COROTATIONAL_ALGORITHM");
  FFA_FIELD_INIT(shadowPosTol,0.05,"COROTATIONAL_OFFSET_TOLERANCE");

  FFA_FIELD_INIT(tolVelProp,0.0,"VELOCITY_PROP_TOLERANCE");

  FFA_FIELD_DEFAULT_INIT(tolDisplacementNorm,"TOL_DISPLACEMENT_NORM");
  FFA_FIELD_DEFAULT_INIT(tolDisplacementTra, "TOL_DISPLACEMENT_TRA");
  FFA_FIELD_DEFAULT_INIT(tolDisplacementRot, "TOL_DISPLACEMENT_ROT");
  FFA_FIELD_DEFAULT_INIT(tolVelocityNorm,    "TOL_VELOCITY_NORM");
  FFA_FIELD_DEFAULT_INIT(tolResidualNorm,    "TOL_RESIDUAL_NORM");
  FFA_FIELD_DEFAULT_INIT(tolResidualTra,     "TOL_RESIDUAL_TRA");
  FFA_FIELD_DEFAULT_INIT(tolResidualRot,     "TOL_RESIDUAL_ROT");
  FFA_FIELD_DEFAULT_INIT(tolEnergyMax,       "TOL_ENERGY_MAX");
  FFA_FIELD_DEFAULT_INIT(tolEnergySum,       "TOL_ENERGY_SUM");

  FFA_FIELD_INIT(solveEigenvalues,      false,"SOLVE_EIGENVALUES");
  FFA_FIELD_INIT(numEigenmodes,         0,    "NUM_EIGENMODES");
  FFA_FIELD_INIT(eigenSolveTimeInterval,0.1,  "EIGENVALUE_SOLUTION_TIME_INTERVAL");
  FFA_FIELD_INIT(eigenvalueShiftFactor, 0.0,  "EIGENVALUE_SHIFT");
  FFA_FIELD_INIT(useBCsOnEigenvalues,   false,"USE_BC_ON_EIGENVALUES");
  FFA_FIELD_INIT(dampedEigenvalues,     false,"CALCULATE_DAMPED_EIGENVALUES");
  FFA_FIELD_INIT(useEigStressStiffening,false,"USE_EIG_STRESS_STIFFENING");
  FFA_FIELD_INIT(solveFrequencyDomain,  false,"SOLVE_FREQUENCYDOMAIN");

  FFA_FIELD_INIT(solveInitEquil,        false,"INITIAL_EQL_ITERATIONS");
  FFA_FIELD_INIT(staticEqlTol,          0.001,"STATIC_EQL_TOLERANCE");
  FFA_FIELD_INIT(iterStepLimit,         1.0,  "STEP_REDUCTION_FACTOR");
  FFA_FIELD_INIT(useEquStressStiffening,false,"USE_EQU_STRESS_STIFFENING");

  FFA_FIELD_INIT(smoothRamp, false, "USE_DYNAMIC_RAMP");
  FFA_FIELD_INIT(rampGrav,   false, "RAMP_GRAVITY");
  FFA_FIELD_INIT(rampSteps,      0, "RAMP_STEPS");
  FFA_FIELD_INIT(rampVmax,     1.0, "RAMP_MAX_SPEED");
  FFA_FIELD_INIT(rampLength,   2.0, "RAMP_LENGTH");
  FFA_FIELD_INIT(rampPause,    0.0, "RAMP_DELAY");

  FFA_FIELD_INIT(autoCurveExportSwitch, false,                  "AUTO_CURVE_EXPORT");
  FFA_FIELD_INIT(autoCurveExportFileName, "exported_curves.asc","AUTO_CURVE_EXPORT_FILE");
  FFA_FIELD_INIT(autoCurveExportFileFormat, ASCII_MULTI_COLUMN, "AUTO_CURVE_EXPORT_FORMAT");

  FFA_FIELD_INIT(autoSolverVTFExport,false, "AUTO_VTF_SOLVER_EXPORT");
  FFA_FIELD_INIT(solverVTFname,"solver.vtf","AUTO_VTF_SOLVER_FILE");
  FFA_FIELD_INIT(solverVTFtype,VTF_EXPRESS, "AUTO_VTF_SOLVER_TYPE");

  FFA_FIELD_INIT(autoAnimateSwitch,false,"AUTO_RBM_ANIMATION");
  FFA_FIELD_INIT(overwriteResults,false,"OVERWRITE_RESULTS");
  FFA_FIELD_INIT(overwriteFEParts,false,"OVERWRITE_FE_PARTS");

  // Default stress recovery options

  FFA_FIELD_INIT(stressStartTime,   0.0,  "STRESS_START_TIME");
  FFA_FIELD_INIT(stressStopTime,    1.0,  "STRESS_STOP_TIME");
  FFA_FIELD_INIT(stressTimeIncr,    0.1,  "STRESS_TIME_INCR");
  FFA_FIELD_INIT(stressAllTimeSteps,false,"USE_ALL_TIME_STEPS");

  FFA_FIELD_INIT(stressDeformation, true, "STRESS_DEFORMATION_OUTPUT");
  FFA_FIELD_INIT(stressStrainTensor,false,"STRESS_STRAIN_OUTPUT");
  FFA_FIELD_INIT(stressStressTensor,false,"STRESS_STRESS_OUTPUT");
  FFA_FIELD_INIT(stressSRTensor,    false,"STRESS_STRESSRES_OUTPUT");
  FFA_FIELD_INIT(stressVMstrain,    false,"STRESS_VMSTRAIN_OUTPUT");
  FFA_FIELD_INIT(stressVMstress,    true, "STRESS_VMSTRESS_OUTPUT");
  FFA_FIELD_INIT(stressMaxPstrain,  false,"STRESS_MAX_PSTRAIN_OUTPUT");
  FFA_FIELD_INIT(stressMaxPstress,  false,"STRESS_MAX_PSTRESS_OUTPUT");
  FFA_FIELD_INIT(stressMinPstrain,  false,"STRESS_MIN_PSTRAIN_OUTPUT");
  FFA_FIELD_INIT(stressMinPstress,  false,"STRESS_MIN_PSTRESS_OUTPUT");
  FFA_FIELD_INIT(stressMaxSstrain,  false,"STRESS_MAX_SHSTRAIN_OUTPUT");
  FFA_FIELD_INIT(stressMaxSstress,  false,"STRESS_MAX_SHSTRESS_OUTPUT");

  FFA_FIELD_INIT(autoStressVTFExport,false, "AUTO_VTF_STRESS_EXPORT");
  FFA_FIELD_INIT(stressVTFname,"stress.vtf","AUTO_VTF_STRESS_FILE");
  FFA_FIELD_INIT(stressVTFtype,VTF_EXPRESS, "AUTO_VTF_STRESS_TYPE");
  FFA_FIELD_INIT(stressVTFrange,FmPair(0,1),"AUTO_VTF_STRESS_RANGE");

  // Default additional solver options

  FFA_FIELD_DEFAULT_INIT(reducerAddOpts,"REDUCER_ADD_OPTIONS");
  FFA_FIELD_DEFAULT_INIT(solverAddOpts, "SOLVER_ADD_OPTIONS");
  FFA_FIELD_DEFAULT_INIT(stressAddOpts, "STRESS_ADD_OPTIONS");

  FFA_FIELD_INIT(useRamSizeGSF, true,"USE_RAMSIZE_GSF_SOLVER");
  FFA_FIELD_INIT(autoRamSizeGSF,true,"AUTO_RAMSIZE_GSF_SOLVER");
  FFA_FIELD_INIT(ramSizeGSF,       0,"RAMSIZE_GSF_SOLVER");

  FFA_FIELD_INIT(useRamSizeBmat, true,"USE_RAMSIZE_RECOVERY_MATRIX");
  FFA_FIELD_INIT(autoRamSizeBmat,true,"AUTO_RAMSIZE_RECOVERY_MATRIX");
  FFA_FIELD_INIT(ramSizeBmat,       0,"RAMSIZE_RECOVERY_MATRIX");

  FFA_FIELD_INIT(maxConcurrentProcesses, 1, "MAX_CONCURRENT_PROCESSES");

  FFA_FIELD_INIT(useProcessPrefix,false,"USE_SOLVE_PROCESS_PREFIX");
  FFA_FIELD_INIT(useProcessPath,  false,"USE_SOLVE_PROCESS_MODEL_FILE_PATH");
  FFA_FIELD_DEFAULT_INIT(processPrefix, "SOLVE_PROCESS_PREFIX");
  FFA_FIELD_DEFAULT_INIT(processPath,   "SOLVE_PROCESS_MODEL_FILE_PATH");

  FFA_FIELD_DEFAULT_INIT(cloudAppId, "CLOUD_APP_ID");

  FFA_FIELD_INIT(useExternalFuncFile,false,"USE_EXTERNAL_FUNCTION_FILE");
  FFA_FIELD_DEFAULT_INIT(externalFuncFileName, "EXTERNAL_FUNCTION_FILE");
}


FmAnalysis::~FmAnalysis()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmAnalysis::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmAnalysis::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmAnalysis::getClassTypeID());
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmAnalysis::writeFMF(std::ostream& os)
{
  os <<"ANALYSIS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


static int seaEngine = 0;

bool FmAnalysis::readAndConnect(std::istream& is, std::ostream&)
{
  FmAnalysis* old = FmDB::getActiveAnalysis(false);
  FmAnalysis* obj = new FmAnalysis();

  FFaObsoleteField<int> seaEngId, shadowPosAlg;
  FFaObsoleteField<bool> useNewmark;
  FFaObsoleteField<double> newmarkFactor;
  FFA_OBSOLETE_FIELD_INIT(seaEngId,0, "SEA_LEVEL_ENGINE", obj);
  FFA_OBSOLETE_FIELD_INIT(shadowPosAlg,1, "SHADOW_POS_ALGORITHM", obj);
  FFA_OBSOLETE_FIELD_INIT(useNewmark,true, "USE_NEWMARK_DAMPING", obj);
  FFA_OBSOLETE_FIELD_INIT(newmarkFactor,0.1, "NEWMARK_DAMPING_FACTOR", obj);
  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("SEA_LEVEL_ENGINE", obj);
  if (seaEngId.wasOnFile() && seaEngine == 0)
    seaEngine = seaEngId.getValue();

  FFA_OBSOLETE_FIELD_REMOVE("SHADOW_POS_ALGORITHM", obj);
  if (shadowPosAlg.wasOnFile() && obj->defaultShadowPosAlg.getValue() == 1)
  {
    if (shadowPosAlg.getValue() == 1)
      obj->defaultShadowPosAlg.setValue(3); // To be consistent with R7.2.1 and earlier
    else if (shadowPosAlg.getValue() == 0)
    {
      obj->defaultShadowPosAlg.setValue(3);
      FFaMsg::dialog("This model used the depreciated \"Max triangle, with unit offset\" option in the\n"
		     "\"Default positioning algorithm for the co-rotated reference coordinate system\"\n"
		     "menu. This is now changed to \"Max triangle, with scaled offset\" instead.\n\n"
		     "This change may affect the simulation, if the model contains Parts that\n"
		     "rely on the default positioning algorithm.",FFaMsg::WARNING);
    }
    else
      obj->defaultShadowPosAlg.setValue(shadowPosAlg.getValue());
  }

  FFA_OBSOLETE_FIELD_REMOVE("USE_NEWMARK_DAMPING", obj);
  if (useNewmark.wasOnFile())
    obj->newmarkDamping.setValue(useNewmark.getValue() ? HHT_ALPHA : NONE);

  FFA_OBSOLETE_FIELD_REMOVE("NEWMARK_DAMPING_FACTOR", obj);
  if (newmarkFactor.wasOnFile())
    obj->setNewmarkDamping(newmarkFactor.getValue());

  // If an old EIGENMODE record was read, the number of eigenmodes
  // has already been stored in the FmAnalysis object
  if (old && obj->numEigenmodes.getValue() == 0)
    obj->numEigenmodes.setValue(old->numEigenmodes.getValue());

  return obj->cloneOrConnect();
}


bool FmAnalysis::localParse(const char* keyWord, std::istream& activeStatement,
			    FmAnalysis* obj)
{
  enum { MAX_EIGENMODES = 1,
         ABSOLUTE_INTEGRATION_TOLERANCE,
         USE_STRESS_STIFFENING,
         EIGENVALUE_SOLUTION_INTERVAL,
         MIN_TIME_INCREMNT,
         MAX_NUM_IT,
         MATRIX_UPDATE_NO,
         STRESS_DEFORMATIONS_ONLY,
         WATER_DENSITY,
         MEAN_SEA_LEVEL,
         SEA_WATER_LEVEL,
         WAVE_DIRECTION,
         WAVE_FUNCTION,
         CURR_FUNCTION,
         CURR_DIRECTION,
         CURR_SCALE };

  static const char* key_words[] = {"MAX_EIGENMODES",
				    "ABSOLUTE_INTEGRATION_TOLERANCE",
				    "USE_STRESS_STIFFENING",
				    "EIGENVALUE_SOLUTION_INTERVAL",
				    "MIN_TIME_INCREMNT",
				    "MAX_NUM_IT",
				    "MATRIX_UPDATE_NO",
				    "STRESS_DEFORMATIONS_ONLY",
				    "WATER_DENSITY",
				    "MEAN_SEA_LEVEL",
				    "SEA_WATER_LEVEL",
				    "WAVE_DIRECTION",
				    "WAVE_FUNCTION",
				    "CURR_FUNCTION",
				    "CURR_DIRECTION",
				    "CURR_SCALE",
				    NULL};

  int tmpInt;
  double tmp;
  switch (FaParse::findIndex(key_words, keyWord))
    {
    case MAX_EIGENMODES:
      return parentParse("NUM_EIGENMODES", activeStatement, obj);

    case ABSOLUTE_INTEGRATION_TOLERANCE:
      activeStatement >> obj->tolVelocityNorm.getValue().value;
      obj->tolVelocityNorm.getValue().policy = FmSolverConvergence::CONV_ALL_OF;
      break;

    case USE_STRESS_STIFFENING:
      activeStatement >> tmpInt;
      obj->useDynStressStiffening.setValue(tmpInt);
      obj->useEquStressStiffening.setValue(tmpInt);
      obj->useEigStressStiffening.setValue(tmpInt);
      break;

    case EIGENVALUE_SOLUTION_INTERVAL:
      activeStatement >> tmpInt;
      obj->eigenSolveTimeInterval.setValue(tmpInt*obj->timeIncr.getValue());
      break;

    case MIN_TIME_INCREMNT:
      return parentParse("MIN_TIME_INCREMENT", activeStatement, obj);

    case MAX_NUM_IT:
      return parentParse("MAXIMUM_NUM_IT", activeStatement, obj);

    case MATRIX_UPDATE_NO:
      if (obj->useFixedMatrixUpdates.getValue())
        return parentParse("MIN_MATRIX_UPDATES", activeStatement, obj);
      break;

    case STRESS_DEFORMATIONS_ONLY:
      bool defsonly;
      activeStatement >> defsonly;
      obj->stressDeformation.setValue(true);
      obj->stressStressTensor.setValue(!defsonly);
      obj->stressStrainTensor.setValue(!defsonly);
      obj->stressSRTensor.setValue(!defsonly);
      break;

    case WATER_DENSITY:
      activeStatement >> tmp;
      if (tmp != 0.0) // Do not create sea state object if default R5.0 value
        FmDB::getSeaStateObject()->waterDensity.setValue(tmp);
      break;

    case MEAN_SEA_LEVEL:
    case SEA_WATER_LEVEL:
      activeStatement >> tmp;
      if (tmp != 0.0) // Do not create sea state object if default R5.0 value
        FmDB::getSeaStateObject()->meanSeaLevel.setValue(tmp);
      break;

    case WAVE_DIRECTION:
    case WAVE_FUNCTION:
    case CURR_FUNCTION:
    case CURR_DIRECTION:
    case CURR_SCALE:
      FmDB::getSeaStateObject()->readField(keyWord,activeStatement);
      break;

    default:
      return parentParse(keyWord, activeStatement, obj);
    }

  return false;
}


void FmAnalysis::initAfterResolve()
{
  this->FmSimulationModelBase::initAfterResolve();

  // Update from old model file

  if (FmDB::getModelFileVer() <= FFaVersionNumber(7,4,3) && shadowPosTol.isDefault() &&
      FmDB::getObjectCount(FmPart::getClassTypeID()) > 0)
  {
    shadowPosTol.setValue(0.5);
    if (defaultShadowPosAlg.getValue() == 1 || defaultShadowPosAlg.getValue() == 3)
      FFaMsg::dialog("This model uses the \"Max triangle, with scaled offset\" option in the menu\n"
                     "\"Default positioning algorithm for the co-rotated reference coordinate systems\".\n"
                     "\nThe offset tolerance used for the third point is reset to the loose value of 0.5\n"
                     "to be compliant with older versions of Fedem. Consider changing this in the\n"
                     "model file by editing the COROTATIONAL_OFFSET_TOLERANCE field.",FFaMsg::WARNING);
  }

  if (positionTolerance.wasOnFile())
    FmDB::getMechanismObject()->positionTolerance.setValue(positionTolerance.getValue());

  if (gravity.wasOnFile())
    FmDB::getMechanismObject()->gravity.setValue(gravity.getValue());

  if (initVel.wasOnFile())
    FmDB::getMechanismObject()->initVel.setValue(initVel.getValue());

  if (seaEngine == 0) return;

  FmMathFuncBase* sfunc = NULL;
  FmBase* found = FmDB::findID(FmEngine::getClassTypeID(),seaEngine);
  if (found) sfunc = static_cast<FmEngine*>(found)->getFunction();
  if (sfunc)
  {
    sfunc->setUserDescription(found->getUserDescription());
    sfunc->setFunctionUse(FmMathFuncBase::WAVE_FUNCTION);
    FmDB::getSeaStateObject()->waveFunction.setRef(sfunc);
    FmModelMemberBase* user = NULL;
    if (!found->hasReferringObjs(user))
    {
      static_cast<FmEngine*>(found)->setFunction(NULL);
      found->erase();
    }
  }

  seaEngine = 0;
}


bool FmAnalysis::setQuasistaticUpToTime(double var)
{
  quasistaticUpToTime.setValue(var);
  return true;
}

bool FmAnalysis::setStartTime(double var)
{
  startTime.setValue(var);

  FmModesOptions* modes = FmDB::getModesOptions(false);
  if (modes && solveEigenvalues.getValue())
    modes->setMinTime(startTime.getValue());

  return true;
}

bool FmAnalysis::setEndTime(double var)
{
  stopTime.setValue(var < startTime.getValue() ? startTime.getValue() : var);

  FmModesOptions* modes = FmDB::getModesOptions(false);
  if (modes && solveEigenvalues.getValue())
    modes->setMaxTime(stopTime.getValue());

  return true;
}

bool FmAnalysis::setTimeIncrement(double var)
{
  if (var <= 0.0) return false;

  timeIncr.setValue(var);
  return true;
}

bool FmAnalysis::setMinTimeIncrement(double var)
{
  if (var <= 0.0) return false;

  minTimeIncr.setValue(var);
  return true;
}

double FmAnalysis::getMinTimeIncrement() const
{
  if (!myTimeIncEngine.isNull() || doCutback.getValue())
    return minTimeIncr.getValue();
  else
    return timeIncr.getValue();
}


bool FmAnalysis::setNewmarkDamping(double alpha_f, double alpha_m)
{
  if (newmarkDamping.getValue() == GENERALIZED_ALPHA)
  {
    if (alpha_f < -1.0 || alpha_f > 0.5)     return false;
    if (alpha_m < -1.0 || alpha_m > alpha_f) return false;
    if (alpha_m < 3.0*alpha_f-1.0)           return false;
    newmarkFactors.setValue({alpha_f,alpha_m});
  }
  else if (newmarkDamping.getValue() == HHT_ALPHA)
  {
    if (alpha_f < 0.0 || 3.0*alpha_f > 1.0)  return false;
    newmarkFactors.getValue().first = alpha_f;
  }

  return true;
}

bool FmAnalysis::setForceNumIt(int var)
{
  // Allow negative values also, now used to specify a Function
  // defining a model/time-dependent fixed number of iterations
  if (var == 0) return false;

  fixedNumIt.setValue(var);
  return true;
}

bool FmAnalysis::setMaxNumIt(int var)
{
  if (var <= 0) return false;

  maxNumIt.setValue(var);
  return true;
}

bool FmAnalysis::setMinNumIt(int var)
{
  if (var <= 0) return false;

  minNumIt.setValue(var);
  return true;
}

bool FmAnalysis::setMinMatrixUpdates(int var)
{
  if (var <= 0) return false;

  minMatrixUpdates.setValue(var);
  return true;
}

bool FmAnalysis::setMaxSequentialNoMatrixUpdates(int var)
{
  if (var < 0) return false;

  maxSequentialNoMatrixUpdates.setValue(var);
  return true;
}

bool FmAnalysis::setTolMatrixUpdateFactor(double var)
{
  if (var < 0.0) return false;

  tolMatrixUpdateFactor.setValue(var);
  return true;
}


bool FmAnalysis::setRelativePropTolerance(double var)
{
  if (var < 0.0) return false;

  tolVelProp.setValue(var);
  return true;
}


static bool setTolerance(FFaField<FmSolverConvergence>& tolField,
                         double var, int toggle)
{
  if (var < 0.0 && (toggle < 0 || toggle >= 10))
    return false;

  if (toggle < 0)
    tolField.getValue().value = var;
  else if (var < 0.0)
    tolField.getValue().policy = FmSolverConvergence::CONV_IGNORE;
  else
    tolField.setValue(FmSolverConvergence(var,toggle%10));

  return true;
}


bool FmAnalysis::setTolDisplacementNorm(double var, int toggle)
{
  return setTolerance(tolDisplacementNorm,var,toggle);
}

bool FmAnalysis::setTolDisplacementTra(double var, int toggle)
{
  return setTolerance(tolDisplacementTra,var,toggle);
}

bool FmAnalysis::setTolDisplacementRot(double var, int toggle)
{
  return setTolerance(tolDisplacementRot,var,toggle);
}

bool FmAnalysis::setTolVelocityNorm(double var, int toggle)
{
  return setTolerance(tolVelocityNorm,var,toggle);
}

bool FmAnalysis::setTolResidualNorm(double var, int toggle)
{
  return setTolerance(tolResidualNorm,var,toggle);
}

bool FmAnalysis::setTolResidualTra(double var, int toggle)
{
  return setTolerance(tolResidualTra,var,toggle);
}

bool FmAnalysis::setTolResidualRot(double var, int toggle)
{
  return setTolerance(tolResidualRot,var,toggle);
}

bool FmAnalysis::setTolEnergyMax(double var, int toggle)
{
  return setTolerance(tolEnergyMax,var,toggle);
}

bool FmAnalysis::setTolEnergySum(double var, int toggle)
{
  return setTolerance(tolEnergySum,var,toggle);
}


bool FmAnalysis::setSolveEigenvalueFlag(bool var)
{
  solveEigenvalues.setValue(var);

  FmModesOptions* modes = FmDB::getModesOptions(false);
  if (modes && solveEigenvalues.getValue())
  {
    modes->setMinTime(startTime.getValue());
    modes->setMaxTime(stopTime.getValue());
  }

  return true;
}


bool FmAnalysis::setRequestedEigenmodes(int val)
{
  if (val < 0) return false;

  numEigenmodes.setValue(val);

  FmModesOptions* modes = FmDB::getModesOptions(false);
  if (modes && solveEigenvalues.getValue())
    modes->setMaxEigenmode(val);

  return true;
}


bool FmAnalysis::setEigenvalueSolutionInterval(int var)
{
  eigenSolveTimeInterval.setValue(var*timeIncr.getValue());

  return true;
}

bool FmAnalysis::setEigenvalueSolutionTimeInterval(double var)
{
  if (var < timeIncr.getValue())
    eigenSolveTimeInterval.setValue(timeIncr.getValue());
  else
    eigenSolveTimeInterval.setValue(var);

  return true;
}


bool FmAnalysis::setStaticEqulTol(double var)
{
  // Allow non-positive values also, now used to flag linear static analysis
  staticEqlTol.setValue(var);
  return true;
}

bool FmAnalysis::setIterationStepReductionFactor(double var)
{
  if (var <= 0.0) return false;

  iterStepLimit.setValue(var);
  return true;
}


bool FmAnalysis::setRampSteps(int var)
{
  if (var < 0) return false;

  rampSteps.setValue(var);
  return true;
}

bool FmAnalysis::setRampPause(double var)
{
  if (var < 0.0) return false;

  rampPause.setValue(var);
  return true;
}

bool FmAnalysis::setRampShape(double Vmax, double T)
{
  if (Vmax <= 0.0 || T <= 0.0) return false;

  if (Vmax*T <= 1.0 || Vmax*T > 2.0)
  {
    FFaMsg::dialog("The product of the Maximum gradient and the Total ramp length\n"
                   "has to be within the range  < 1.0, 2.0 ].",FFaMsg::DISMISS_ERROR);
    return false;
  }

  rampVmax.setValue(Vmax);
  rampLength.setValue(T);
  return true;
}


int FmAnalysis::getNumberOfStressTimeSteps() const
{
  const double tolTime = 1.0e-12;
  double timeSpan = stressStopTime.getValue() - stressStartTime.getValue();
  if (timeSpan < -tolTime) return 0;
  if (timeSpan <= tolTime) return 1;

  double timeInc = stressAllTimeSteps.getValue() ?
    this->getMinTimeIncrement() : stressTimeIncr.getValue();

  if (timeInc >= tolTime)
    return (int)floor((timeSpan+tolTime)/timeInc + 1.0);
  else
    return 0;
}


void FmAnalysis::getEigenvalueSamples(DoubleVec& sampleArray) const
{
  const double tolTime = 1.0e-12;
  double timeSpan = stopTime.getValue() - startTime.getValue();

  int count = 0;
  double eigInc = eigenSolveTimeInterval.getValue();
  if (timeSpan <= tolTime)
    count = 1;
  else if (eigInc >= tolTime)
    count = (int)floor((timeSpan+tolTime)/eigInc + 1.0);

  sampleArray.resize(count);
  for (int i = 0; i < count; i++)
    sampleArray[i] = startTime.getValue() + i*eigInc;
}


void FmAnalysis::getEigenvalueList(IntVec& modesList) const
{
  modesList.resize(numEigenmodes.getValue());
  std::iota(modesList.begin(),modesList.end(),1);
}


std::string FmAnalysis::getProcessPath() const
{
  if (this->useProcessPath.getValue())
    return this->processPath.getValue();
  else
    return std::string("");
}


bool FmAnalysis::needMassMatrix() const
{
  if (this->solveEigenvalues.getValue())
    return true; // We are doing eigenvalue analysis

  else if (!this->dynamicsEnable.getValue())
    return false; // No time history response analysis

  else if (!this->quasistaticEnable.getValue())
    return true; // Pure dynamics simulation

  else if (!this->quasistaticMode.getValue())
    return false; // Pure quasi-static simulation

  return this->quasistaticUpToTime.getValue() < this->stopTime.getValue();
}
