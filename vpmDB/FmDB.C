// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <fstream>
#include <ctime>

#include "vpmDB/FmDB.H"
#include "vpmDB/FmQuery.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmModesOptions.H"
#include "vpmDB/FmGageOptions.H"
#include "vpmDB/FmFppOptions.H"
#ifdef FT_HAS_NCODE
#include "vpmDB/FmDutyCycleOptions.H"
#endif
#include "vpmDB/FmGlobalViewSettings.H"

#include "vpmDB/FmAnimation.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmEngine.H"
#ifdef FT_HAS_EXTCTRL
#include "vpmDB/FmExternalCtrlSys.H"
#endif
#include "vpmDB/FmFileReference.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmGear.H"
#include "vpmDB/FmGenericDBObject.H"
#include "vpmDB/FmGraph.H"
#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmJointMotion.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmRackPinion.H"
#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmRelativeSensor.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmRoad.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmSpringChar.H"
#include "vpmDB/FmSticker.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmTimeSensor.H"
#include "vpmDB/FmTire.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmDofMotion.H"
#include "vpmDB/FmDofLoad.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmAllControlHeaders.H"
#include "vpmDB/FmAllFunctionHeaders.H"
#include "vpmDB/FmBearingFriction.H"
#include "vpmDB/FmPrismaticFriction.H"
#include "vpmDB/FmCamFriction.H"
#include "vpmDB/FmStraightMaster.H"
#include "vpmDB/FmPipeSurface.H"
#include "vpmDB/FmPipeStringDataExporter.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmAirState.H"
#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmFuncTree.H"
#include "vpmDB/FmStructAssembly.H"
#include "vpmDB/FmRiser.H"
#include "vpmDB/FmSoilPile.H"
#include "vpmDB/FmJacket.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmMaterialProperty.H"
#include "vpmDB/FmUserDefinedElement.H"
#include "vpmDB/FmModelExpOptions.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#include "vpmDB/FmModelMemberConnector.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdDB.H"
#endif
#ifdef FT_USE_CMDLINEARG
#include "FFaLib/FFaCmdLineArg/FFaCmdLineArg.H"
#endif
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "Admin/FedemAdmin.H"


/**********************************************************************
 *
 * STATIC VARIABLES
 *
 **********************************************************************/

FmDB::FmHeadMap                  FmDB::ourHeadMap;
std::map<int,FmModelMemberBase*> FmDB::ourBaseIDMap;
std::map<std::string,int>        FmDB::unknownKeywords;

FmFuncTree* FmDB::itsFuncTree  = NULL;
FmLink*     FmDB::itsEarthLink = NULL;

FFaVersionNumber FmDB::ourCurrentFedemVersion;
FFaVersionNumber FmDB::ourModelFileVersion;
int              FmDB::ourSaveNr = 0;


/*******************************************************************************
 *
 * STATIC METHODS
 *
 ******************************************************************************/

/*!
  Initialization method for \a headMap containing all entities in a model.
  The initialization creates one FmRingStart object for each type of object,
  and sets up a parent-child relationship if neccesary for use in the GUI.

  The order of creation of the FmRingStart objects determines the "natural
  order" of DB objects, mainly used for the output order on the model file.

  It is strongly advisable to add new objects at the end to keep diff'ing
  new and old model files as clean as possible.
*/

void FmDB::initHeadMap(FmHeadMap& headMap, FmFuncTree*& funcTree)
{
  headMap[FmGlobalViewSettings::getClassTypeID()] = new FmRingStart("View settings");

  headMap[FmMechanism::getClassTypeID()] = new FmRingStart("Mechanisms");
  headMap[FmSeaState::getClassTypeID()] = new FmRingStart("Sea states", sea_xpm);
  headMap[FmAirState::getClassTypeID()] = new FmRingStart("Air states");
  headMap[FmAnalysis::getClassTypeID()] = new FmRingStart("Analyses");
  headMap[FmModesOptions::getClassTypeID()] = new FmRingStart("Modes Options");
  headMap[FmGageOptions::getClassTypeID()] = new FmRingStart("Gage Options");
  headMap[FmFppOptions::getClassTypeID()] = new FmRingStart("Fpp Options");
#ifdef FT_HAS_NCODE
  headMap[FmDutyCycleOptions::getClassTypeID()] = new FmRingStart("Duty Cycle Options");
#endif
  headMap[FmModelExpOptions::getClassTypeID()] = new FmRingStart("Model Export Options");

  headMap[FmRefPlane::getClassTypeID()] = new FmRingStart("Reference planes", referencePlane_xpm);
  headMap[FmPart::getClassTypeID()] = new FmRingStart("Parts", FELink_xpm, true);
  headMap[FmBeam::getClassTypeID()] = new FmRingStart("Beams", beam_xpm, true);
  headMap[FmElementGroupProxy::getClassTypeID()] = new FmRingStart("Element groups");
  headMap[FmTriad::getClassTypeID()] = new FmRingStart("Triads", triad_xpm, true);

  headMap[FmJointBase::getClassTypeID()] = new FmRingStart("Joints", revJoint_xpm, true); // meta
  headMap[FmRevJoint::getClassTypeID()] = new FmRingStart("Revolute joints", revJoint_xpm);
  headMap[FmBallJoint::getClassTypeID()] = new FmRingStart("Ball joints", ballJoint_xpm);
  headMap[FmRigidJoint::getClassTypeID()] = new FmRingStart("Rigid joints", rigidJoint_xpm);
  headMap[FmFreeJoint::getClassTypeID()] = new FmRingStart("Free joints", freeJoint_xpm);
  headMap[FmPrismJoint::getClassTypeID()] = new FmRingStart("Prismatic joints", prismJoint_xpm);
  headMap[FmCylJoint::getClassTypeID()] = new FmRingStart("Cylindric joints", cylJoint_xpm);
  headMap[FmCamJoint::getClassTypeID()] = new FmRingStart("Cam joints", camJoint_xpm);

  headMap[Fm1DMaster::getClassTypeID()] = new FmRingStart("Multi-masters", NULL, true); // meta
  headMap[FmStraightMaster::getClassTypeID()] = new FmRingStart("Straight masters");
  headMap[FmArcSegmentMaster::getClassTypeID()] = new FmRingStart("Curved masters");
  headMap[FmPipeSurface::getClassTypeID()] = new FmRingStart("Pipe surfaces");

  headMap[FmHPBase::getClassTypeID()] = new FmRingStart("Gears", gear_xpm, true); // meta
  headMap[FmGear::getClassTypeID()] = new FmRingStart("Gears", gear_xpm);
  headMap[FmRackPinion::getClassTypeID()] = new FmRingStart("Rack-and-pinions", rackPinon_xpm);

  headMap[FmSticker::getClassTypeID()] = new FmRingStart("Stickers", sticker_xpm, true);

  headMap[FmMathFuncBase::getClassTypeID()] = new FmRingStart("Function definitions", function_xpm, true); // meta
  headMap[FmEngine::getClassTypeID()] = new FmRingStart("Functions", function_xpm, true);
  headMap[FmfLinVelVar::getClassTypeID()] = new FmRingStart("Linear derivative functions");
  headMap[FmfLinVar::getClassTypeID()] = new FmRingStart("Poly lines");
  headMap[FmfSpline::getClassTypeID()] = new FmRingStart("Splines");
  headMap[FmfConstant::getClassTypeID()] = new FmRingStart("Constants");
  headMap[FmfMathExpr::getClassTypeID()] = new FmRingStart("Math expressions");
  headMap[FmfDeviceFunction::getClassTypeID()] = new FmRingStart("Poly lines from file");
  headMap[FmfExternalFunction::getClassTypeID()] = new FmRingStart("External function");
  headMap[FmfScale::getClassTypeID()] = new FmRingStart("Linear functions");
  headMap[FmfSinusoidal::getClassTypeID()] = new FmRingStart("Sines");
  headMap[FmfComplSinus::getClassTypeID()] = new FmRingStart("Combined sines");
  headMap[FmfDelayedComplSinus::getClassTypeID()] = new FmRingStart("Delayed combined sines");
  headMap[FmfWaveSinus::getClassTypeID()] = new FmRingStart("Wave sines");
  headMap[FmfWaveSpectrum::getClassTypeID()] = new FmRingStart("Wave spectrums");
  headMap[FmfRamp::getClassTypeID()] = new FmRingStart("Ramps");
  headMap[FmfStep::getClassTypeID()] = new FmRingStart("Steps");
  headMap[FmfSquarePuls::getClassTypeID()] = new FmRingStart("Periodic square pulses");
  headMap[FmfDiracPuls::getClassTypeID()] = new FmRingStart("Dirac pulses");
  headMap[FmfLimRamp::getClassTypeID()] = new FmRingStart("Limited ramps");
  headMap[FmfSmoothTraj::getClassTypeID()] = new FmRingStart("Smooth trajectories");
  headMap[FmfUserDefined::getClassTypeID()] = new FmRingStart("User-defined functions");

  headMap[FmFrictionBase::getClassTypeID()] = new FmRingStart("Frictions", friction_xpm, true); // meta
  headMap[FmRotFriction::getClassTypeID()] = new FmRingStart("Rotational frictions", gearFriction_xpm);
  headMap[FmTransFriction::getClassTypeID()] = new FmRingStart("Translational frictions", camJointFriction_xpm);
  headMap[FmBearingFriction::getClassTypeID()] = new FmRingStart("Bearing frictions", revJointFriction_xpm);
  headMap[FmPrismaticFriction::getClassTypeID()] = new FmRingStart("Prismatic frictions", prismJointFriction_xpm);
  headMap[FmCamFriction::getClassTypeID()] = new FmRingStart("Cam frictions", camJointFriction_xpm);

  headMap[FmSpringChar::getClassTypeID()] = new FmRingStart("Advanced spring characteristics", spring_xpm, true);

  headMap[FmLoad::getClassTypeID()] = new FmRingStart("Loads", loadSmall_xpm, true);
  headMap[FmAxialDamper::getClassTypeID()] = new FmRingStart("Axial dampers", damper_xpm, true);
  headMap[FmAxialSpring::getClassTypeID()] = new FmRingStart("Axial springs", spring_xpm, true);

  headMap[FmJointDamper::getClassTypeID()] = new FmRingStart("Joint dampers", NULL, true);
  headMap[FmJointSpring::getClassTypeID()] = new FmRingStart("Joint springs", NULL, true);
  headMap[FmJointMotion::getClassTypeID()] = new FmRingStart("Joint motions", NULL, true);
  headMap[FmDofMotion::getClassTypeID()] = new FmRingStart("Motions", NULL, true);
  headMap[FmDofLoad::getClassTypeID()] = new FmRingStart("Loads", NULL, true);

  headMap[FmVesselMotion::getClassTypeID()] = new FmRingStart("Vessel motions", vesselMotion_xpm, true);

  headMap[FmTire::getClassTypeID()] = new FmRingStart("Tires", createTire_xpm, true);
  headMap[FmRoad::getClassTypeID()] = new FmRingStart("Roads", createRoad_xpm, true);

  headMap[FmSensorBase::getClassTypeID()] = new FmRingStart("Sensors", makeSimpleSensor_xpm, true); // meta
  headMap[FmTimeSensor::getClassTypeID()] = new FmRingStart("Time sensors", makeSimpleSensor_xpm);
  headMap[FmSimpleSensor::getClassTypeID()] = new FmRingStart("Simple sensors", makeSimpleSensor_xpm);
  headMap[FmRelativeSensor::getClassTypeID()] = new FmRingStart("Relative sensors", makeRelativeSensor_xpm);

  headMap[FmCtrlElementBase::getClassTypeID()] = new FmRingStart("Control elements", control_xpm, true); // meta
  headMap[FmcInput::getClassTypeID()] = new FmRingStart("Inputs", ctrlElemIn_xpm);
  headMap[FmcOutput::getClassTypeID()] = new FmRingStart("Outputs", ctrlElemOut_xpm);
  headMap[FmcAmplifier::getClassTypeID()] = new FmRingStart("Amplifiers", ctrlAmplifier_xpm);
  headMap[FmcPower::getClassTypeID()] = new FmRingStart("Power elements", ctrlPower_xpm);
  headMap[FmcComparator::getClassTypeID()] = new FmRingStart("Comparators", ctrlComparator_xpm);
  headMap[FmcAdder::getClassTypeID()] = new FmRingStart("Adders", ctrlAdder_xpm);
  headMap[FmcDeadZone::getClassTypeID()] = new FmRingStart("Dead zones", ctrlDeadZone_xpm);
  headMap[FmcHysteresis::getClassTypeID()] = new FmRingStart("Hysteresis elements", ctrlHysteresis_xpm);
  headMap[FmcIntegrator::getClassTypeID()] = new FmRingStart("Integrators", ctrlIntegrator_xpm);
  headMap[FmcLimDerivator::getClassTypeID()] = new FmRingStart("Limited derivators", ctrlLimDerivator_xpm);
  headMap[FmcLimitation::getClassTypeID()] = new FmRingStart("Limitation elements", ctrlLimitation_xpm);
  headMap[FmcLogicalSwitch::getClassTypeID()] = new FmRingStart("Logical switches", ctrlLogicalSwitch_xpm);
  headMap[FmcMultiplier::getClassTypeID()] = new FmRingStart("Multipliers", ctrlMultiplier_xpm);
  headMap[FmcPi::getClassTypeID()] = new FmRingStart("PI controllers", ctrlPi_xpm);
  headMap[FmcSampleHold::getClassTypeID()] = new FmRingStart("Sample and hold elements", ctrlSampleHold_xpm);
  headMap[FmcTimeDelay::getClassTypeID()] = new FmRingStart("Time delays", ctrlTimeDelay_xpm);
  headMap[Fmc1ordTF::getClassTypeID()] = new FmRingStart("1st Order transfer functions", ctrl1ordTF_xpm);
  headMap[Fmc2ordTF::getClassTypeID()] = new FmRingStart("2nd Order transfer functions", ctrl2ordTF_xpm);
  headMap[FmcCompConjPole::getClassTypeID()] = new FmRingStart("Complex conjugate poles", ctrlCompConjPole_xpm);
  headMap[FmcPIlimD::getClassTypeID()] = new FmRingStart("PI+lim D controllers", ctrlPIlimD_xpm);
  headMap[FmcPd::getClassTypeID()] = new FmRingStart("PD Controllers", ctrlPd_xpm);
  headMap[FmcPid::getClassTypeID()] = new FmRingStart("PID controllers", ctrlPid_xpm);
  headMap[FmcPlimD::getClassTypeID()] = new FmRingStart("P+lim D controllers", ctrlPlimD_xpm);
  headMap[FmcPlimI::getClassTypeID()] = new FmRingStart("P+lim I controllers", ctrlPlimI_xpm);
  headMap[FmcPlimIlimD::getClassTypeID()] = new FmRingStart("P+lim I+lim D controllers", ctrlPlimIlimD_xpm);
  headMap[FmcRealPole::getClassTypeID()] = new FmRingStart("Real poles", ctrlRealPole_xpm);
  headMap[FmCtrlLine::getClassTypeID()] = new FmRingStart("Control Lines");
#ifdef FT_HAS_EXTCTRL
  headMap[FmExternalCtrlSys::getClassTypeID()] = new FmRingStart("External control systems", control_xpm);
#endif
  headMap[FmUserDefinedElement::getClassTypeID()] = new FmRingStart("User-defined elements", NULL, true);

  headMap[FmAnimation::getClassTypeID()] = new FmRingStart("Animations", NULL, true);
  headMap[FmGraph::getClassTypeID()] = new FmRingStart("Graphs", NULL, true);
  headMap[FmCurveSet::getClassTypeID()] = new FmRingStart("Curves", NULL, true);

  headMap[FmFileReference::getClassTypeID()] = new FmRingStart("File references", fileref_xpm, true);
  headMap[FmStrainRosette::getClassTypeID()] = new FmRingStart("Strain rosettes", makeStrainRosette_xpm, true);
  headMap[FmGenericDBObject::getClassTypeID()] = new FmRingStart("Generic objects", generic_xpm, true);

  headMap[FmPipeStringDataExporter::getClassTypeID()] = new FmRingStart("Pipe string exporters", NULL, true);

  headMap[FmBladeProperty::getClassTypeID()] = new FmRingStart("Blade properties", windBladeProp_xpm, true);
  headMap[FmBeamProperty::getClassTypeID()] = new FmRingStart("Beam cross sections", beamProp_xpm, true);
  headMap[FmMaterialProperty::getClassTypeID()] = new FmRingStart("Materials", material_xpm, true);

  headMap[FmSubAssembly::getClassTypeID()] = new FmRingStart("Assemblies", NULL, true);

  // Bugfix #308: The simulation event object must always be listed at the end.
  // Please, do not add new objects after this points, if so it will
  // not be possible to event-modify those objects.
  headMap[FmSimulationEvent::getClassTypeID()] = new FmRingStart("Simulation events", events_xpm, true);

  // Set up ring start topology.

  funcTree = new FmFuncTree(headMap[FmMathFuncBase::getClassTypeID()]);

  // Joints
  headMap[FmRevJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmBallJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmRigidJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmFreeJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmCamJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmCylJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);
  headMap[FmPrismJoint::getClassTypeID()]->setParent(headMap[FmJointBase::getClassTypeID()]);

  // Multi-masters
  headMap[FmStraightMaster::getClassTypeID()]->setParent(headMap[Fm1DMaster::getClassTypeID()]);
  headMap[FmArcSegmentMaster::getClassTypeID()]->setParent(headMap[Fm1DMaster::getClassTypeID()]);
  headMap[FmPipeSurface::getClassTypeID()]->setParent(headMap[Fm1DMaster::getClassTypeID()]);

  // Functions
  headMap[FmEngine::getClassTypeID()]->setParent(headMap[FmMathFuncBase::getClassTypeID()]);

  headMap[FmfConstant::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfDelayedComplSinus::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfDiracPuls::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfMathExpr::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfDeviceFunction::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfExternalFunction::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfLimRamp::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfLinVelVar::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfLinVar::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfScale::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfSpline::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfComplSinus::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfRamp::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfSinusoidal::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfWaveSinus::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfWaveSpectrum::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfSquarePuls::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfStep::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfSmoothTraj::getClassTypeID()]->setParent(funcTree->myHead);
  headMap[FmfUserDefined::getClassTypeID()]->setParent(funcTree->myHead);

  // Control elements
  headMap[FmcAdder::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcAmplifier::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcComparator::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcCompConjPole::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmCtrlLine::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcDeadZone::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcInput::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcHysteresis::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcIntegrator::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcLimDerivator::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcLimitation::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcLogicalSwitch::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcMultiplier::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcOutput::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPi::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPIlimD::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPd::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPid::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPlimD::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPlimI::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPlimIlimD::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcPower::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcSampleHold::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcTimeDelay::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[FmcRealPole::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[Fmc1ordTF::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);
  headMap[Fmc2ordTF::getClassTypeID()]->setParent(headMap[FmCtrlElementBase::getClassTypeID()]);

  // Sensors
  // The FmTimeSensor ring is not assigned a parent ring to avoid writing the
  // Sensors heading to the model file when it does not contain other sensors.
  // The TimeSensor itself is not saved but generated automatically when needed.
  headMap[FmSimpleSensor::getClassTypeID()]->setParent(headMap[FmSensorBase::getClassTypeID()]);
  headMap[FmRelativeSensor::getClassTypeID()]->setParent(headMap[FmSensorBase::getClassTypeID()]);

  // Gears
  headMap[FmGear::getClassTypeID()]->setParent(headMap[FmHPBase::getClassTypeID()]);
  headMap[FmRackPinion::getClassTypeID()]->setParent(headMap[FmHPBase::getClassTypeID()]);

  // Frictions
  headMap[FmRotFriction::getClassTypeID()]->setParent(headMap[FmFrictionBase::getClassTypeID()]);
  headMap[FmTransFriction::getClassTypeID()]->setParent(headMap[FmFrictionBase::getClassTypeID()]);
  headMap[FmBearingFriction::getClassTypeID()]->setParent(headMap[FmFrictionBase::getClassTypeID()]);
  headMap[FmPrismaticFriction::getClassTypeID()]->setParent(headMap[FmFrictionBase::getClassTypeID()]);
  headMap[FmCamFriction::getClassTypeID()]->setParent(headMap[FmFrictionBase::getClassTypeID()]);

  // Update ring starts to contain type data
  for (FmHeadMap::value_type& head : headMap)
    head.second->setRingMemberType(head.first);
}


void FmDB::sortHeadMap(const FmHeadMap& headMap,
                       FmHeadMap& sortedHeadMap, bool reverse)
{
  // Sort rings in output order
  for (const FmHeadMap::value_type& head : headMap)
    if (reverse)
      sortedHeadMap[-head.second->getSortNumber()] = head.second;
    else
      sortedHeadMap[head.second->getSortNumber()] = head.second;
}


void FmDB::init()
{
  // Initialize the top-level head map
  FmDB::initHeadMap(ourHeadMap,itsFuncTree);

  // Initalize the earth link
  itsEarthLink = new FmPart("Earth");

  // Initialize the version number of current executable
  ourCurrentFedemVersion.parseLine(FedemAdmin::getVersion());
}


/*!
  This method cleans up the heap-allocated singelton objects
  in FmDB which are not related to a mechanism model as such.
  Used mainly in test programs to verify no memory leaks, etc.
*/

void FmDB::removeInstances()
{
  itsEarthLink->erase();
  itsEarthLink = NULL;

  delete itsFuncTree;
  itsFuncTree = NULL;

  for (FmHeadMap::value_type& head : ourHeadMap)
    delete head.second;
  ourHeadMap.clear();

  FFaFieldContainer::removeDictInstance();
  FmSignalConnector::removeInstance();
  FFaSwitchBoard::removeInstance();
}


FmModelMemberBase* FmDB::createObject(int classTypeId)
{
  if (classTypeId == FmMechanism::getClassTypeID())
    return FmDB::getMechanismObject(); // There should only be one
  else if (classTypeId == FmAirState::getClassTypeID())
    return FmDB::getAirStateObject(); // There should only be one...
  else if (classTypeId == FmSeaState::getClassTypeID())
    return FmDB::getSeaStateObject(); // There should only be one...
  else if (classTypeId == FmAnalysis::getClassTypeID())
    return FmDB::getActiveAnalysis(); // There should only be one...

  else if (classTypeId == FmTurbine::getClassTypeID())
    return new FmTurbine('T');
  else if (classTypeId == FmBladeDesign::getClassTypeID())
    return new FmBladeDesign();
  else if (classTypeId == FmBladeProperty::getClassTypeID())
    return new FmBladeProperty();

  else if (classTypeId == FmBeamProperty::getClassTypeID())
    return new FmBeamProperty();
  else if (classTypeId == FmMaterialProperty::getClassTypeID())
    return new FmMaterialProperty();

  return NULL;
}


FmMechanism* FmDB::newMechanism()
{
  FmDB::eraseAll();

  ourSaveNr = 0;

  FmMechanism* mech = new FmMechanism();
  mech->connect();

  itsEarthLink->setLocalCS(FaMat34());

  FmRefPlane* refPlane = new FmRefPlane();
  refPlane->connect();
  refPlane->draw();

  FmDB::drawGVector();

  return mech;
}


int FmDB::getObjectCount(int typeID, const FmHeadMap* root)
{
  FmRingStart* head = FmDB::getHead(typeID,root);
  int count = head ? head->countRingMembers() : 0;
  if (!head || (count == 0 && !head->getChildren().empty()))
    for (const FmHeadMap::value_type& child : *root)
      if (child.second->getNext()->isOfType(std::abs(typeID)))
        count += child.second->countRingMembers();

  head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (!head) return count;

  for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
  {
    FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
    if (subAss) count += FmDB::getObjectCount(typeID,subAss->getHeadMap());
  }

  return count;
}


/*!
  This method fills a vector of all objects in the DB of requested type.
  The method returns false if no objects of the type queried for are found.
*/

bool FmDB::getAllOfType(std::vector<FmModelMemberBase*>& toBeFilled,
                        int classTypeID, const FmSubAssembly* subAss,
                        const char* tag)
{
  toBeFilled.clear();
  return FmDB::appendAllOfType(toBeFilled,classTypeID,std::vector<int>(),
                               tag ? tag : "", FmDB::getHeadMap(subAss));

}

bool FmDB::appendAllOfType(std::vector<FmModelMemberBase*>& toBeFilled,
                           int classTypeID, const std::vector<int>& except,
                           const std::string& tagged)
{
  return FmDB::appendAllOfType(toBeFilled,classTypeID,except,tagged,&ourHeadMap);
}

bool FmDB::appendAllOfType(std::vector<FmModelMemberBase*>& toBeFilled,
                           int classTypeID, const std::vector<int>& except,
                           const std::string& tagged, const FmHeadMap* root)
{
  size_t oldSize = toBeFilled.size();
  size_t nExclude = except.size();
  for (const FmHeadMap::value_type& head : *root)
  {
    FmBase* runner = head.second->getNext();
    bool okToUse = runner->isOfType(std::abs(classTypeID));
    for (size_t i = 0; i < nExclude && okToUse; i++)
      if (runner->isOfType(except[i])) okToUse = false;

    if (okToUse)
      while (runner && runner != head.second)
      {
        FmModelMemberBase* obj = static_cast<FmModelMemberBase*>(runner);
        if (tagged.empty() || obj->isTagged(tagged))
          toBeFilled.push_back(obj);
        runner = runner->getNext();
      }
  }

  FmBase* head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (head && classTypeID >= 0)
    for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
    {
      FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
      if (subAss) FmDB::appendAllOfType(toBeFilled,classTypeID,except,
                                        tagged,subAss->getHeadMap());
    }

  return toBeFilled.size() > oldSize;
}


void FmDB::getTypeQuery(std::vector<FmModelMemberBase*>& toBeFilled,
			const std::map<int,bool>& query)
{
  toBeFilled.clear();
  if (query.empty()) return;

  std::vector<int> dontWantTypes;
  for (const std::pair<const int,bool>& qp : query)
    if (!qp.second)
      dontWantTypes.push_back(qp.first);

  for (const std::pair<const int,bool>& qp : query)
    if (qp.second)
      FmDB::appendAllOfType(toBeFilled, qp.first, dontWantTypes);
}


void FmDB::getQuery(std::vector<FmModelMemberBase*>& toBeFilled, FmQuery* query)
{
  toBeFilled.clear();
  if (!query) return;

  if (query->verifyCB.empty())
    FmDB::getTypeQuery(toBeFilled,query->typesToFind);
  else {
    bool isOK = false;
    std::vector<FmModelMemberBase*> tmp;
    FmDB::getTypeQuery(tmp,query->typesToFind);
    for (FmModelMemberBase* obj : tmp) {
      query->verifyCB.invoke(isOK,obj);
      if (isOK)
        toBeFilled.push_back(obj);
    }
  }
}


template<class T> static void FmdFill_Vec(std::vector<T*>& queName,
					  const std::map<int,FmRingStart*>* root,
					  int classTypeId = -1,
					  bool thisLevelOnly = false)
{
  if (classTypeId < 0) classTypeId = T::getClassTypeID();
  FmBase* head = FmDB::getHead(classTypeId,root);
  if (head)
    for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
      queName.push_back((T*)pt);

  if (thisLevelOnly) return;

  head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (head)
    for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
    {
      FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
      if (subAss) FmdFill_Vec(queName,subAss->getHeadMap(),classTypeId);
    }
}


void FmDB::getAllTriads(std::vector<FmTriad*>& triads,
			const FmSubAssembly* subAss,
			bool thisLevelOnly)
{
  triads.clear();
  FmdFill_Vec(triads,FmDB::getHeadMap(subAss),-1,thisLevelOnly);
}


void FmDB::getAllLinks(std::vector<FmLink*>& links,
		       const FmSubAssembly* subAss,
		       bool thisLevelOnly)
{
  links.clear();
  const FmHeadMap* headMap = FmDB::getHeadMap(subAss);
  FmdFill_Vec(links, headMap, FmPart::getClassTypeID(), thisLevelOnly);
  FmdFill_Vec(links, headMap, FmBeam::getClassTypeID(), thisLevelOnly);
  FmdFill_Vec(links, headMap, FmUserDefinedElement::getClassTypeID(), thisLevelOnly);
}


void FmDB::getAllBeams(std::vector<FmBeam*>& beams,
		       const FmSubAssembly* subAss,
		       bool thisLevelOnly)
{
  beams.clear();
  FmdFill_Vec(beams,FmDB::getHeadMap(subAss),-1,thisLevelOnly);
}


void FmDB::getAllParts(std::vector<FmPart*>& parts,
		       const FmSubAssembly* subAss,
		       bool thisLevelOnly)
{
  parts.clear();
  FmdFill_Vec(parts,FmDB::getHeadMap(subAss),-1,thisLevelOnly);
}


void FmDB::getUnsavedParts(std::vector<FmPart*>& parts)
{
  FmDB::getAllParts(parts);
  for (std::vector<FmPart*>::iterator it = parts.begin(); it != parts.end();)
    if ((*it)->isSaved())
      it = parts.erase(it);
    else
      ++it;
}


void FmDB::getFEParts(std::vector<FmPart*>& parts, bool reverseOrder)
{
  FmDB::getAllParts(parts);
  for (std::vector<FmPart*>::iterator it = parts.begin(); it != parts.end();)
    if ((*it)->isGenericPart() || (*it)->isSuppressed())
      it = parts.erase(it);
    else
      ++it;

  if (reverseOrder && parts.size() > 1)
    std::reverse(parts.begin(),parts.end());
}


FmModelMemberBase* FmDB::findObject(int baseID)
{
  std::map<int,FmModelMemberBase*>::const_iterator it = ourBaseIDMap.find(baseID);
  if (it != ourBaseIDMap.end())
    return it->second;
  else
    return NULL;
}


bool FmDB::insertInBaseIDMap(FmModelMemberBase* pt)
{
  if (!pt) return false;

  bool status = ourBaseIDMap.insert(std::make_pair(pt->getBaseID(),pt)).second;
#ifdef FM_DEBUG
  std::cout <<"FmDB::insertInBaseIDMap() "<< pt->getTypeIDName()
	    <<" "<< pt->getID() <<" ("<< pt->getBaseID()
	    <<") "<< std::boolalpha << status << std::endl;
#endif
  return status;
}


void FmDB::removeFromBaseIDMap(FmModelMemberBase* pt)
{
#ifdef FM_DEBUG
  std::cout <<"FmDB::removeFromBaseIDMap() "<< pt->getTypeIDName()
	    <<" "<< pt->getID() <<" ("<< pt->getBaseID() <<")"<< std::endl;
#endif
  ourBaseIDMap.erase(pt->getBaseID());
}


int FmDB::getFreeBaseID()
{
  if (ourBaseIDMap.empty())
    return 1;
  else
    return ourBaseIDMap.rbegin()->first + 1;
}


void FmDB::getAllGears(std::vector<FmGear*>& g)
{
  g.clear();
  FmdFill_Vec(g,&ourHeadMap);
}


void FmDB::getAllRackPinions(std::vector<FmRackPinion*>& rp)
{
  rp.clear();
  FmdFill_Vec(rp,&ourHeadMap);
}


void FmDB::getAllStickers(std::vector<FmSticker*>& s)
{
  s.clear();
  FmdFill_Vec(s,&ourHeadMap);
}


void FmDB::eraseAllStickers()
{
  std::vector<FmSticker*> stickers;
  FmDB::getAllStickers(stickers);
  for (FmSticker* s : stickers) s->erase();
}


void FmDB::getAllControlOutput(std::vector<FmcOutput*>& o)
{
  o.clear();
  FmdFill_Vec(o,&ourHeadMap);
}


void FmDB::getAllControlInput(std::vector<FmcInput*>& i)
{
  i.clear();
  FmdFill_Vec(i,&ourHeadMap);
}


void FmDB::getAllControlElements(std::vector<FmCtrlInputElementBase*>& ctrl)
{
  ctrl.clear();
  FmdFill_Vec(ctrl,&ourHeadMap,FmcAmplifier::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcAdder::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPower::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcComparator::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcDeadZone::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcHysteresis::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcIntegrator::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcLimDerivator::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcLogicalSwitch::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcLimitation::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPi::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcMultiplier::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcSampleHold::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcTimeDelay::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,Fmc1ordTF::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,Fmc2ordTF::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcCompConjPole::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPIlimD::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPd::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPid::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPlimD::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPlimI::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcPlimIlimD::getClassTypeID());
  FmdFill_Vec(ctrl,&ourHeadMap,FmcRealPole::getClassTypeID());
}


void FmDB::eraseAllControlObjects()
{
  std::vector<FmCtrlInputElementBase*> ctrl;
  std::vector<FmcOutput*> out;
  std::vector<FmcInput*> in;

  FmDB::getAllControlElements(ctrl);
  FmDB::getAllControlOutput(out);
  FmDB::getAllControlInput(in);

  for (FmCtrlInputElementBase* ce : ctrl) ce->erase();
  for (FmcOutput* co : out) co->erase();
  for (FmcInput* ci : in) ci->erase();
}


bool FmDB::hasObjects(int typeID, const FmHeadMap* root)
{
  FmBase* head = FmDB::getHead(typeID,root);
  if (!head) return false;

  if (head->getNext() != head) return true;

  head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (!head) return false;

  for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
  {
    FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
    if (subAss && FmDB::hasObjects(typeID,subAss->getHeadMap()))
      return true;
  }

  return false;
}


bool FmDB::hasObjectsOfType(int classTypeID, const FmHeadMap* root)
{
  for (const FmHeadMap::value_type& head : *root)
  {
    FmBase* pt = head.second->getNext();
    if (pt && pt != head.second && pt->isOfType(classTypeID))
      return true;
  }

  FmBase* head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (head)
    for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
    {
      FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
      if (subAss && FmDB::hasObjectsOfType(classTypeID,subAss->getHeadMap()))
	return true;
    }

  return false;
}


void FmDB::getAllHPs(std::vector<FmHPBase*>& hps)
{
  hps.clear();
  FmdFill_Vec(hps,&ourHeadMap,FmGear::getClassTypeID());
  FmdFill_Vec(hps,&ourHeadMap,FmRackPinion::getClassTypeID());
}


void FmDB::getAllCylJoints(std::vector<FmCylJoint*>& cyl)
{
  cyl.clear();
  FmdFill_Vec(cyl,&ourHeadMap);
}


void FmDB::getAllCamJoints(std::vector<FmCamJoint*>& cams)
{
  cams.clear();
  FmdFill_Vec(cams,&ourHeadMap);
}


void FmDB::getAllRevJoints(std::vector<FmRevJoint*>& rev)
{
  rev.clear();
  FmdFill_Vec(rev,&ourHeadMap);
}


void FmDB::getAllBallJoints(std::vector<FmBallJoint*>& ball)
{
  ball.clear();
  FmdFill_Vec(ball,&ourHeadMap);
}


void FmDB::getAllFreeJoints(std::vector<FmFreeJoint*>& fj)
{
  fj.clear();
  FmdFill_Vec(fj,&ourHeadMap);
}


void FmDB::getAllRigidJoints(std::vector<FmRigidJoint*>& rgd)
{
  rgd.clear();
  FmdFill_Vec(rgd,&ourHeadMap);
}


void FmDB::getAllPrismJoints(std::vector<FmPrismJoint*>& prism)
{
  prism.clear();
  FmdFill_Vec(prism,&ourHeadMap);
}


void FmDB::getAllControlLines(std::vector<FmCtrlLine*>& lines)
{
  lines.clear();
  FmdFill_Vec(lines,&ourHeadMap);
}


void FmDB::getAllLoads(std::vector<FmLoad*>& loads)
{
  loads.clear();
  FmdFill_Vec(loads,&ourHeadMap);
}


void FmDB::getAllRefPlanes(std::vector<FmRefPlane*>& rp)
{
  rp.clear();
  FmdFill_Vec(rp,&ourHeadMap);
}


void FmDB::getAllAxialSprings(std::vector<FmAxialSpring*>& springs)
{
  springs.clear();
  FmdFill_Vec(springs,&ourHeadMap);
}


void FmDB::getAllSpringChars(std::vector<FmSpringChar*>& springChars)
{
  springChars.clear();
  FmdFill_Vec(springChars,&ourHeadMap);
}


void FmDB::getAllAxialDampers(std::vector<FmAxialDamper*>& dampers)
{
  dampers.clear();
  FmdFill_Vec(dampers,&ourHeadMap);
}


void FmDB::getAllJointDampers(std::vector<FmJointDamper*>& dampers)
{
  dampers.clear();
  std::vector<FmJointBase*> jnts;
  FmDB::getAllJoints(jnts);
  for (FmJointBase* joint : jnts)
    for (int dof = 0; dof < FmJointBase::MAX_DOF; dof++)
    {
      FmJointDamper* dmp = joint->getDamperAtDOF(dof);
      if (dmp) dampers.push_back(dmp);
    }
}


void FmDB::getAllJointSprings(std::vector<FmJointSpring*>& springs)
{
  springs.clear();
  std::vector<FmJointBase*> jnts;
  FmDB::getAllJoints(jnts);
  for (FmJointBase* joint : jnts)
    for (int dof = 0; dof < FmJointBase::MAX_DOF; dof++)
    {
      FmJointSpring* spr = joint->getSpringAtDOF(dof);
      if (spr) springs.push_back(spr);
    }
}


void FmDB::getAllFunctions(std::vector<FmMathFuncBase*>& toFill,
			   const FmSubAssembly* subAss,
			   bool thisLevelOnly)
{
  const FmHeadMap* headMap = FmDB::getHeadMap(subAss);

  toFill.clear();
  FmdFill_Vec(toFill,headMap,FmfConstant::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfScale::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfLinVelVar::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfSinusoidal::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfComplSinus::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfDelayedComplSinus::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfWaveSinus::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfWaveSpectrum::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfRamp::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfStep::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfSquarePuls::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfDiracPuls::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfLimRamp::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfSmoothTraj::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfLinVar::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfMathExpr::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfDeviceFunction::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfExternalFunction::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfSpline::getClassTypeID(),thisLevelOnly);
  FmdFill_Vec(toFill,headMap,FmfUserDefined::getClassTypeID(),thisLevelOnly);
}


void FmDB::getAllMultiVarFuncs(std::vector<FmfMultiVarBase*>& f)
{
  f.clear();
  FmdFill_Vec(f,&ourHeadMap,FmfLinVelVar::getClassTypeID());
  FmdFill_Vec(f,&ourHeadMap,FmfLinVar::getClassTypeID());
  FmdFill_Vec(f,&ourHeadMap,FmfSpline::getClassTypeID());
}


void FmDB::getAllSensors(std::vector<FmSensorBase*>& sens)
{
  sens.clear();
  FmdFill_Vec(sens,&ourHeadMap,FmTimeSensor::getClassTypeID());
  FmdFill_Vec(sens,&ourHeadMap,FmSimpleSensor::getClassTypeID());
  FmdFill_Vec(sens,&ourHeadMap,FmRelativeSensor::getClassTypeID());
}


#ifdef FT_HAS_EXTCTRL
void FmDB::getAllExternalCtrlSys(std::vector<FmExternalCtrlSys*>& ext)
{
  ext.clear();
  FmdFill_Vec(ext,&ourHeadMap);
}
#endif


void FmDB::getAllEngines(std::vector<FmEngine*>& engines)
{
  engines.clear();
  FmdFill_Vec(engines,&ourHeadMap);
}


void FmDB::getAllSplines(std::vector<FmfSpline*>& sp)
{
  sp.clear();
  FmdFill_Vec(sp,&ourHeadMap);
}


void FmDB::getAllDeviceFunctions(std::vector<FmfDeviceFunction*>& f)
{
  f.clear();
  FmdFill_Vec(f,&ourHeadMap);
}


void FmDB::getAllJoints(std::vector<FmJointBase*>& jnts)
{
  jnts.clear();
  FmdFill_Vec(jnts,&ourHeadMap,FmRevJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmRigidJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmBallJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmFreeJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmPrismJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmCylJoint::getClassTypeID());
  FmdFill_Vec(jnts,&ourHeadMap,FmCamJoint::getClassTypeID());
}


void FmDB::getAllSimulationEvents(std::vector<FmSimulationEvent*>& events,
                                  bool reverseOrder)
{
  events.clear();
  FmdFill_Vec(events,&ourHeadMap);
  if (reverseOrder && events.size() > 1)
    std::reverse(events.begin(),events.end());
}


void FmDB::getAllBladeDesigns(std::vector<FmBladeDesign*>& blades)
{
  std::vector<FmSubAssembly*> subass;
  FmdFill_Vec(subass,&ourHeadMap);

  FmBladeDesign* bs;
  blades.clear();
  for (FmSubAssembly* obj : subass)
    if ((bs = dynamic_cast<FmBladeDesign*>(obj)))
      blades.push_back(bs);
}


double FmDB::getPositionTolerance()
{
  return FmDB::getMechanismObject()->positionTolerance.getValue();
}


const FaVec3& FmDB::getGrav()
{
  return FmDB::getMechanismObject()->gravity.getValue();
}


void FmDB::drawGVector()
{
#ifdef USE_INVENTOR
  FdDB::updateGDirection(FmDB::getGrav());
#endif
}


FmVesselMotion* FmDB::getActiveRAO()
{
  FmSeaState* seastate = FmDB::getSeaStateObject(false);
  if (!seastate) return NULL;

  FmVesselMotion* raom = NULL;
  FmMathFuncBase* func = seastate->waveFunction.getPointer();
  if (func && func->hasReferringObjs(raom,"waveFunction"))
    return raom;
  else
    return NULL;
}


FaMat34 FmDB::getSeaCS()
{
  FmSeaState* seastate = FmDB::getSeaStateObject(false);
  if (!seastate) return FaMat34();

  // Define the sea coordinate system based on the vessel configuration
  FmMechanism*    mech = FmDB::getMechanismObject();
  FmVesselMotion* raom = FmDB::getActiveRAO();
  if (raom)
    return raom->getWaveCS(mech->gravity.getValue(),
			   seastate->waveDir.getValue(),
			   seastate->meanSeaLevel.getValue());

  // No vessel system is provided. Define the Z-axis to be in opposite
  // direction of the gravitation vector, and the X-axis to be projection
  // of the provided wave direction vector onto the XY-plane
  FaVec3 eZ(-mech->gravity.getValue());
  FaVec3 eY = eZ.normalize() ^ seastate->waveDir.getValue();
  FaVec3 eX = eY.normalize() ^ eZ;
  FaVec3 O(seastate->getX(),seastate->getY(),seastate->meanSeaLevel.getValue());
  FaMat33 mat(eX,eY,eZ);
  return FaMat34(mat,mat*O);
}


bool FmDB::useSeaCS()
{
  // Return whether the sea coordinate system is in use or not
  return FmDB::getSeaStateObject(false) ? true : false;
}


void FmDB::drawSea()
{
  FmSeaState* seaState = FmDB::getSeaStateObject(false);
  if (seaState)
    seaState->draw();
}


template<class T> static T* FmdGet_Object(T* object, bool createIfNone)
{
  FmBase* head = FmDB::getHead(T::getClassTypeID());
  if (!head) return NULL; // Logic error, should never happen

  if (head->getNext() != head)
    object = static_cast<T*>(head->getNext());
  else if (createIfNone)
  {
    object = new T();
    object->connect();
  }
  else
    object = NULL;

  return object;
}


FmSensorBase* FmDB::getTimeSensor(bool createIfNone)
{
  FmTimeSensor* sensor = NULL;
  return FmdGet_Object(sensor,createIfNone);
}


FmSeaState* FmDB::getSeaStateObject(bool createIfNone)
{
  FmSeaState* seastate = NULL;
  return FmdGet_Object(seastate,createIfNone);
}


FmAirState* FmDB::getAirStateObject(bool createIfNone)
{
  FmAirState* airstate = NULL;
  return FmdGet_Object(airstate,createIfNone);
}


FmGlobalViewSettings* FmDB::getActiveViewSettings(bool createIfNone)
{
  FmGlobalViewSettings* viewSettings = NULL;
  return FmdGet_Object(viewSettings,createIfNone);
}


FmAnalysis* FmDB::getActiveAnalysis(bool createIfNone)
{
  FmAnalysis* analysis = NULL;
  return FmdGet_Object(analysis,createIfNone);
}


FmModesOptions* FmDB::getModesOptions(bool createIfNone)
{
  FmModesOptions* modes = NULL;
  return FmdGet_Object(modes,createIfNone);
}


FmGageOptions* FmDB::getGageOptions(bool createIfNone)
{
  FmGageOptions* gage = NULL;
  return FmdGet_Object(gage,createIfNone);
}


FmFppOptions* FmDB::getFppOptions(bool createIfNone)
{
  FmFppOptions* fpp = NULL;
  return FmdGet_Object(fpp,createIfNone);
}


#ifdef FT_HAS_NCODE
FmDutyCycleOptions* FmDB::getDutyCycleOptions(bool createIfNone)
{
  FmDutyCycleOptions* dc = NULL;
  return FmdGet_Object(dc,createIfNone);
}
#endif


FmModelExpOptions* FmDB::getModelExportOptions(bool createIfNone)
{
  FmModelExpOptions* exp = NULL;
  return FmdGet_Object(exp, createIfNone);
}


FmMechanism* FmDB::getMechanismObject(bool createIfNone)
{
  FmMechanism* mech = NULL;
  return FmdGet_Object(mech,createIfNone);
}


FmTurbine* FmDB::getTurbineObject(int ID)
{
  FmTurbine* turbine = NULL;
  std::vector<FmModelMemberBase*> allAss;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID());
  for (FmModelMemberBase* obj : allAss)
    if (ID < 1 || obj->getID() == ID)
      if ((turbine = dynamic_cast<FmTurbine*>(obj)))
        break;

  if (!turbine && ID < 0)
  {
    // No tower assembly if ID < -1
    turbine = new FmTurbine(ID < -1 ? 'N' : 'T');
    turbine->connect();
  }

  return turbine;
}


/*!
  Utility method to collect all fields in the model containing a file path.
*/

void FmDB::getAllPaths(std::vector<FFaField<std::string>*>& allPathNames,
                       const FmSubAssembly* subAss)
{
  // Model link repository
  if (!subAss)
    allPathNames.push_back(&(FmDB::getMechanismObject()->modelLinkRepository));

  std::vector<FmPart*> allParts;
  FmDB::getAllParts(allParts,subAss);

  // Imported part data files and part-specific repositories
  for (FmPart* part : allParts)
  {
    allPathNames.push_back(&(part->visDataFile));
    allPathNames.push_back(&(part->originalFEFile));
    allPathNames.push_back(&(part->myRepository));
  }

  // Tire property files
  std::vector<FmModelMemberBase*> allObjs;
  FmDB::getAllOfType(allObjs,FmTire::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmTire*>(obj)->tireDataFileName));

  // Road property files
  FmDB::getAllOfType(allObjs,FmRoad::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmRoad*>(obj)->roadDataFileName));

#ifdef FT_HAS_EXTCTRL
  // External control system files
  FmDB::getAllOfType(allObjs,FmExternalCtrlSys::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmExternalCtrlSys*>(obj)->myFilePath));
#endif

  // External function files
  FmDB::getAllOfType(allObjs,FmfDeviceFunction::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmfDeviceFunction*>(obj)->deviceName));

  // Reference curve files
  FmDB::getAllOfType(allObjs,FmCurveSet::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmCurveSet*>(obj)->myFilePath));

  // File references
  FmDB::getAllOfType(allObjs,FmFileReference::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmFileReference*>(obj)->fileName));

  // Vessel motion RAO files
  FmDB::getAllOfType(allObjs,FmVesselMotion::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    allPathNames.push_back(&(static_cast<FmVesselMotion*>(obj)->raoFile));

  // Turbulent wind file
  if (!subAss)
  {
    FmAirState* air = FmDB::getAirStateObject(false);
    if (air) allPathNames.push_back(&(air->windFile));
  }

  // Tower shadow file
  FmTurbine* turbine;
  FmDB::getAllOfType(allObjs,FmSubAssembly::getClassTypeID(),subAss);
  for (FmModelMemberBase* obj : allObjs)
    if ((turbine = dynamic_cast<FmTurbine*>(obj)))
      allPathNames.push_back(&(turbine->towerFile));

  // Remove the empty fields
  size_t i, n = 0;
  for (i = 0; i < allPathNames.size(); i++)
    if (!allPathNames[i]->getValue().empty())
      allPathNames[n++] = allPathNames[i];
  allPathNames.resize(n);
}


/*!
  Utility method to translate all relative paths in the model such that they
  are correct after saving, when changing the model file path of the model.
  Used by "Save As..."
*/

void FmDB::translateRelativePaths(const std::string& oldPath,
                                  const std::string& newPath,
                                  const FmSubAssembly* subAss)
{
  if (oldPath == newPath) return;

  // Find all pathnames in the model and store a pointer to each in a vector
  std::vector<FFaField<std::string>*> allPathNames;
  FmDB::getAllPaths(allPathNames,subAss);
  FmAnalysis* anal = FmDB::getActiveAnalysis(false);
  if (anal && !anal->externalFuncFileName.getValue().empty())
    allPathNames.push_back(&(anal->externalFuncFileName));

  // Now modify the relative path names to comply with the new model file path
  for (FFaField<std::string>* field : allPathNames)
  {
    std::string fName = field->getValue();
    if (!FFaFilePath::isRelativePath(fName)) continue; // skip absolute paths

    // Make fname into an absolute path
    fName = FFaFilePath::appendFileNameToPath(oldPath,fName);

    // Store updated relative path wrt. newPath
    field->setValue(FFaFilePath::getRelativeFilename(newPath,fName));
  }
}


/*!
  Erase all joint dof spring, dampers, loads and motions that are not active.
  Typically invoked before a "Save", to reduce the size of the model file.
*/

bool FmDB::purgeJointComponents()
{
  std::vector<FmModelMemberBase*> objs;

  int nErasedS = 0;
  FmDB::getAllOfType(objs,FmJointSpring::getClassTypeID());
  for (FmModelMemberBase* obj : objs)
    if (!static_cast<FmJointSpring*>(obj)->getActiveOwner())
      if (obj->erase()) nErasedS++;

  if (nErasedS > 0)
    ListUI <<" --> Purging "<< nErasedS <<" inactive joint springs\n";

  int nErasedD = 0;
  FmDB::getAllOfType(objs,FmJointDamper::getClassTypeID());
  for (FmModelMemberBase* obj : objs)
    if (!static_cast<FmJointDamper*>(obj)->getActiveOwner())
      if (obj->erase()) nErasedD++;

  if (nErasedD > 0)
    ListUI <<" --> Purging "<< nErasedD <<" inactive joint dampers\n";

  int nErasedL = 0;
  FmDB::getAllOfType(objs,FmDofLoad::getClassTypeID());
  for (FmModelMemberBase* obj : objs)
    if (!static_cast<FmDofLoad*>(obj)->getActiveOwner())
      if (obj->erase()) nErasedL++;

  if (nErasedL > 0)
    ListUI <<" --> Purging "<< nErasedL <<" inactive DOF loads\n";

  int nErasedM = 0;
  FmDB::getAllOfType(objs,FmDofMotion::getClassTypeID());
  for (FmModelMemberBase* obj : objs)
    if (!static_cast<FmDofMotion*>(obj)->getActiveOwner())
      if (obj->erase()) nErasedM++;

  if (nErasedM > 0)
    ListUI <<" --> Purging "<< nErasedM <<" inactive DOF motions\n";

  return nErasedS + nErasedD + nErasedL + nErasedM > 0;
}


bool FmDB::updateModelVersionOnSave(bool warnOnNewVersion)
{
  if (ourModelFileVersion == ourCurrentFedemVersion) return true;

  if (warnOnNewVersion)
  {
    // Warn the user when saving the model in a different Fedem version
    std::string msg("The current model was last saved in Fedem ");
    msg += ourModelFileVersion.getString() + ".\n";
    if (ourModelFileVersion > ourCurrentFedemVersion)
      // Current version is older than the model file version
      msg += "Are you sure you now want to save this model in Fedem "
	+ ourCurrentFedemVersion.getString() + " ?";
    else
      // Current version is newer than the model file version
      msg += "If you save this model now, it will no longer be readable in that version, and you\n"
	"have to manually edit the model file to obtain an equivalent model in that version.\n"
	"Please consult the latest Release Notes for issues that might arise.\n\nProceed ?";

    if (!FFaMsg::dialog(msg,FFaMsg::OK_CANCEL))
      return false;
  }

  // Update the current model file version to avoid warning on next save
  ourModelFileVersion = ourCurrentFedemVersion;
  return true;
}


bool FmDB::reportAll(std::ostream& os, bool writeMetaData,
		     const FmHeadMap& headMap, const char* addMetaData)
{
  if (!os) return false;

  // Writing the model file
  os <<"FEDEMMODELFILE {" << FedemAdmin::getVersion() <<" ASCII}\n";
  os <<"!Module version: "<< FedemAdmin::getVersion() <<" "<< FedemAdmin::getBuildDate() <<"\n";
  os <<"!Model file name: "<< FmDB::getMechanismObject()->getModelFileName() <<"\n";
  if (writeMetaData)
  {
    const time_t currentTime = time(NULL);
    os <<"!Last saved: #"<< ++ourSaveNr <<", "<< ctime(&currentTime);
  }
  if (addMetaData)
    os << addMetaData <<"\n";
  os <<"\n";

  int oldPrec = os.precision(12); // Output precision for real values
  FmDB::reportMembers(os,headMap);
  os.precision(oldPrec);

  os <<"END {FEDEMMODELFILE}\n";
  return os ? true : false;
}


void FmDB::reportMembers(std::ostream& os, const FmHeadMap& headMap)
{
  if (!os) return;

  // Sort rings in output order
  FmHeadMap sortedMap;
  FmDB::sortHeadMap(headMap,sortedMap);
  // Swap the order for Functions and Function Definitions, such that
  // the headings are printed in the correct place in the model file.
  // This is a consequence of the May 14 2014 bugfix where the sorted order
  // of these two was changed to avoid crash when erasing sub-assemblies.
  FmHeadMap::const_iterator itxE = headMap.find(FmEngine::getClassTypeID());
  FmHeadMap::const_iterator itxF = headMap.find(FmMathFuncBase::getClassTypeID());
  if (itxE != headMap.end() && itxF != headMap.end())
    std::swap(sortedMap[itxE->second->getSortNumber()],
              sortedMap[itxF->second->getSortNumber()]);

  // Print them
  for (const FmHeadMap::value_type& head : sortedMap)
  {
    if (head.second->printHeader() && head.second->hasRingMembers())
      os <<"\n!*** "<< head.second->getUITypeName() <<" ***\n\n";
    for (FmBase* pt = head.second->getNext(); pt != head.second; pt = pt->getNext())
      if (!pt->writeFMF(os)) return;
  }
}


void FmDB::emergencyExitSave()
{
  std::cerr <<"Trying to save model file..."<< std::endl;
  FmSubAssembly::mainFilePath = "";
  std::ofstream os("fedem_save.fmm",std::ios::out);
  if (FmDB::reportAll(os,false))
    std::cerr <<"Emergency save: Model file saved in [fedem_save.fmm] on your project directory."<< std::endl;
}


void FmDB::displayAll(const FmHeadMap& headMap)
{
  if (FFaAppInfo::isConsole()) return;

  std::vector<int> displayOrder;

  displayOrder.push_back(FmPart::getClassTypeID());
  displayOrder.push_back(FmTriad::getClassTypeID());
  displayOrder.push_back(FmRevJoint::getClassTypeID());
  displayOrder.push_back(FmBallJoint::getClassTypeID());
  displayOrder.push_back(FmRigidJoint::getClassTypeID());
  displayOrder.push_back(FmFreeJoint::getClassTypeID());
  displayOrder.push_back(FmPrismJoint::getClassTypeID());
  displayOrder.push_back(FmCylJoint::getClassTypeID());
  displayOrder.push_back(FmCamJoint::getClassTypeID());
  displayOrder.push_back(FmGear::getClassTypeID());
  displayOrder.push_back(FmRackPinion::getClassTypeID());

  // First display the objects that must be displayed in given order
  for (int classType : displayOrder)
    FmDB::displayMembers(classType,&headMap);

  // Then display the rest, taking care not to display any of
  // the already displayed ones
  for (const FmHeadMap::value_type& head : headMap)
    if (std::find(displayOrder.begin(),displayOrder.end(),head.first) == displayOrder.end())
      FmDB::displayMembers(head.first,&headMap);

  if (&headMap != &ourHeadMap) return;

  FmDB::drawGVector();
  FmDB::getActiveViewSettings()->sync();
}


void FmDB::displayMembers(int typeID, const FmHeadMap* root)
{
  if (!root) return;

  FmHeadMap::const_iterator it = root->find(typeID);
  if (it != root->end()) it->second->displayRingMembers();

  FmBase* head = FmDB::getHead(FmSubAssembly::getClassTypeID(),root);
  if (!head) return;

  for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
  {
    FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(pt);
    if (subAss) FmDB::displayMembers(typeID,subAss->getHeadMap());
  }
}


bool FmDB::eraseAll(bool showProgress)
{
  // Erase the rings in reverse order
  FmHeadMap sortedMap;
  FmDB::sortHeadMap(ourHeadMap,sortedMap,true);
  for (const FmHeadMap::value_type& head : sortedMap)
    head.second->eraseRingMembers(showProgress);

  ourHeadMap[FmElementGroupProxy::getClassTypeID()]->printHeader(false);

  itsEarthLink->setLocalCS(FaMat34());

  ourBaseIDMap.clear();
  return true;
}


/*!
  Traverses all the objects in the DB. For each group of objects
  toBeCalledForEachHead is called, and the CB is supposed to set the
  passed bool variable to false if the current group is not to be traversed.

  Groups that do not contain any objects are ignored.
*/

void FmDB::forAllInDB(FFaDynCB2<bool&,FmBase*>& toBeCalledForEachHead,
		      FFaDynCB1<FmBase*>& toBeCalledForEach,
		      const FmHeadMap* root)
{
  for (const FmHeadMap::value_type& head : *root)
  {
    bool okToTraverseGroup = true;
    FmBase* runner = head.second->getNext();
    if (runner != head.second)
      toBeCalledForEachHead.invoke(okToTraverseGroup,runner->getNext());

    if (okToTraverseGroup)
      while (runner && runner != head.second)
      {
	toBeCalledForEach.invoke(runner);
	if (head.first == FmSubAssembly::getClassTypeID())
	  FmDB::forAllInDB(toBeCalledForEachHead,toBeCalledForEach,
			   static_cast<FmSubAssembly*>(runner)->getHeadMap());
	runner = runner->getNext();
      }
  }
}


bool FmDB::findIDrange(const FmBase* obj, int& fromID, int& toID)
{
  if (!obj) return false;

  FmSubAssembly* parentAss = dynamic_cast<FmSubAssembly*>(obj->getParentAssembly());
  FmBase* hd = FmDB::getHead(obj->getTypeID(),FmDB::getHeadMap(parentAss));
  if (!hd) return false;

  fromID = hd->getNext()->getID();
  toID   = hd->getPrev()->getID();

  return true;
}


FmBase* FmDB::findID(int type, int IDnr,
		     const std::vector<int>& assemblyID)
{
  // First, check if this is a leaf type
  FmBase* hd = FmDB::getHead(type,assemblyID,FmSubAssembly::tmpHeadMap);
  if (hd)
    for (FmBase* pt = hd->getNext(); pt != hd; pt = pt->getNext())
      if (pt->getID() == IDnr)
	return pt;

  const FmHeadMap* headMap = FmDB::getHeadMap(assemblyID,FmSubAssembly::tmpHeadMap);
  if (!headMap) return NULL;

  // It probably wasn't, see if it is a parent class then
  for (const FmHeadMap::value_type& head : *headMap)
  {
    FmBase* runner = head.second->getNext();
    if (runner && runner->isOfType(type))
      for (; runner && runner != head.second; runner = runner->getNext())
        if (runner->isOfType(type) && runner->getID() == IDnr)
          return runner;
  }

  return NULL;
}

FmBase* FmDB::findID(const std::string& type, int IDnr,
		     const std::vector<int>& assemblyID)
{
  const FmHeadMap* headMap = FmDB::getHeadMap(assemblyID,FmSubAssembly::tmpHeadMap);
  if (!headMap) return NULL;

  for (const FmHeadMap::value_type& head : *headMap)
  {
    FmBase* runner = head.second->getNext();
    if (runner && runner->getUITypeName() == type)
      for (; runner && runner != head.second; runner = runner->getNext())
        if (runner->getID() == IDnr)
          return runner;
  }

  return NULL;
}


FmRingStart* FmDB::getHead(int classTypeID)
{
  return FmDB::getHead(classTypeID,&ourHeadMap);
}

FmRingStart* FmDB::getHead(int classTypeID,
                           const std::vector<int>& assemblyID,
                           const FmHeadMap* root)
{
  return FmDB::getHead(classTypeID,FmDB::getHeadMap(assemblyID,root));
}

FmRingStart* FmDB::getHead(int classTypeID, const FmHeadMap* headMap)
{
  if (!headMap) return NULL;

  FmHeadMap::const_iterator it = headMap->find(classTypeID);
  return it == headMap->end() ? NULL : it->second;
}


const FmDB::FmHeadMap* FmDB::getHeadMap(const FmSubAssembly* subAss)
{
  return subAss ? subAss->getHeadMap() : &ourHeadMap;
}

const FmDB::FmHeadMap* FmDB::getHeadMap(const std::vector<int>& assemblyID,
                                        const FmHeadMap* root)
{
  return FmDB::getHeadMap(root ? root : &ourHeadMap,
                          assemblyID.begin(),assemblyID.end());
}

const FmDB::FmHeadMap* FmDB::getHeadMap(const FmHeadMap* root,
                                        std::vector<int>::const_iterator it,
                                        std::vector<int>::const_iterator end,
                                        int parentAssemblyID)
{
  if (it == end || *it < 1) return root;

  // For each level in the assembly id path,
  // find the correct assembly, then return the headMap of that assembly
  FmHeadMap::const_iterator ait = root->find(FmSubAssembly::getClassTypeID());
  if (ait == root->end()) return NULL; // Logic error; no sub-assembly ring

  // Find sub-assembly with correct ID
  FmSubAssembly* subAss;
  FmBase* ringStart = ait->second;
  for (FmBase* pt = ringStart->getNext(); pt != ringStart; pt = pt->getNext())
    if (pt->getID() == *it)
    {
      if ((subAss = dynamic_cast<FmSubAssembly*>(pt)))
        // Invoke recursively for the next sub-assembly level
        return FmDB::getHeadMap(subAss->getHeadMap(),++it,end,subAss->getID());
      else
        return NULL; // Logic error; object found is of incorrect type
    }

  // The sub-assembly does not exist yet, so create it here
  subAss = new FmSubAssembly();
  subAss->setID(*it);
  subAss->setParentAssembly(parentAssemblyID);
  subAss->connect();

  // Invoke recursively for the next sub-assembly level
  return FmDB::getHeadMap(subAss->getHeadMap(),++it,end,subAss->getID());
}


FmSubAssembly* FmDB::getSubAssembly(const std::vector<int>& assemblyID)
{
  if (assemblyID.empty()) return NULL;

  int subAssID = assemblyID.back();
  std::vector<int> assID(assemblyID); assID.pop_back();
  FmBase* head = FmDB::getHead(FmSubAssembly::getClassTypeID(),assID);
  if (head)
    for (FmBase* pt = head->getNext(); pt != head; pt = pt->getNext())
      if (pt->getID() == subAssID)
	return dynamic_cast<FmSubAssembly*>(pt);

  std::cerr <<"ERROR: Invalid assembly ID:";
  for (int id : assemblyID) std::cerr <<" "<< id;
  return NULL;
}


static std::map<int,int> readLog;

enum  { FEDEMMODELFILE = 1,
	MECHANISM = 2,
	ANALYSIS = 3,
	TRIAD = 4,
	LINK = 5,
	GRAPH = 6,
	CURVE_SET = 7,
	GLOBAL_VIEW_SETTINGS = 8,
	STICKER = 9,
	REV_JOINT = 10,
	BALL_JOINT = 11,
	FREE_JOINT = 12,
	RIGID_JOINT = 13,
	PRISM_JOINT = 14,
	CYL_JOINT = 15,
	AXIAL_DAMPER = 16,
	AXIAL_SPRING = 17,
	JOINT_SPRING = 18,
	JOINT_DAMPER = 19,
	LOAD = 20,
	REF_PLANE = 21,
	FUNC_LIN_VEL_VAR = 22,
	FUNC_CONSTANT = 23,
	FUNC_SINUSOIDAL = 24,
	FUNC_COMPL_SINUS = 25,
	FUNC_DELAYED_COMPL_SINUS = 26,
	FUNC_RAMP = 27,
	FUNC_STEP = 28,
	FUNC_SQUARE_PULS = 29,
	FUNC_LIN_VAR = 30,
	FUNC_SPLINE = 31,
	GEAR = 32,
	RACK_PINION = 33,
	FUNC_DIRAC_PULS = 34,
	FUNC_LIM_RAMP = 35,
	FUNC_SMOOTH_TRAJ = 36,
	ENGINE = 37,
	SENSOR = 38,
	RELATIVE_SENSOR = 39,
	EIGENMODE = 40,
	AIR_STATE = 41,
	FUNC_REV_JNT_FRICTION = 42,
	FUNC_PRISM_JNT_FRICTION = 43,
	FUNC_CAM_JNT_FRICTION = 44,
	FUNC_SCALE = 45,
	CONTROL_LINE = 46,
	CONTROL_AMPLIFIER = 47,
	CONTROL_POWER = 48,
	CONTROL_ADDER = 49,
	CONTROL_COMPARATOR = 50,
	CONTROL_INPUT = 51,
	CONTROL_OUTPUT = 52,
	CONTROL_DEAD_ZONE = 53,
	CONTROL_HYSTERESIS = 54,
	CONTROL_INTEGRATOR = 55,
	CONTROL_LIM_DERIVATOR = 56,
	CONTROL_LIMITATION = 57,
	CONTROL_LOGICAL_SWITCH = 58,
	CONTROL_MULTIPLIER = 59,
	CONTROL_PI = 60,
	CONTROL_SAMPLE_HOLD = 61,
	CONTROL_TIME_DELAY = 62,
	CONTROL_FIRST_ORDTF = 63,
	CONTROL_SEC_ORDTF = 64,
	CONTROL_COMPCONJPOLE = 65,
	CONTROL_PILIMD = 66,
	CONTROL_PD = 67,
	CONTROL_PID = 68,
	CONTROL_PLIMD = 69,
	CONTROL_PLIMI = 70,
	CONTROL_PLIMILIMD = 71,
	CONTROL_REAL_POLE = 72,
	CAM_JOINT = 73,
	ELEMENT_GROUP = 74,
	FUNC_DEVICE_FUNCTION = 75,
	ANIMATION = 76,
	MODESOPTIONS = 77,
	GAGEOPTIONS = 78,
	FPPOPTIONS = 79,
	DUTYCYCLEOPTIONS = 80,
	GENERIC_DB_OBJECT = 81,
	EXTERNAL_CTRL_SYSTEM = 82,
	FILE_REFERENCE = 83,
	FUNC_MATH_EXPRESSION = 84,
	TIRE = 85,
	ROAD = 86,
	STRAIN_ROSETTE = 87,
	JOINT_MOTION = 88,
	TRIAD_MOTION = 89,
	JOINT_LOAD = 90,
	SPRING_CHAR = 91,
	PIPE_SURFACE = 92,
	PIPE_STRING_EXPORTER = 93,
	ROT_FRICTION = 94,
	TRANS_FRICTION = 95,
	BEARING_FRICTION = 96,
	PRISMATIC_FRICTION = 97,
	CAM_FRICTION = 98,
	FUNC_WAVE_SINUS = 99,
	FUNC_WAVE_SPECTRUM = 100,
	VESSEL_MOTION = 101,
	MASTER_LINE = 102,
	MASTER_ARC_SEGMENT = 103,
	SIMULATION_EVENT = 104,
	DOF_MOTION = 105,
	DOF_LOAD = 106,
	FUNC_USER_DEFINED = 107,
	SEA_STATE = 108,
	SUBASSEMBLY = 109,
	RISER = 110,
	SOIL_PILE = 111,
	JACKET = 112,
	TURBINE = 113,
	TOWER = 114,
	NACELLE = 115,
	GENERATOR = 116,
	GEARBOX = 117,
	SHAFT = 118,
	ROTOR = 119,
	BLADE = 120,
	TURBINE_BLADE_DESIGN = 121,
	TURBINE_BLADE_PROPERTY = 122,
	BEAM_PROPERTY = 123,
	BEAMMATERIAL_PROPERTY = 124,
	BEAM = 125,
	PART = 126,
	USER_DEFINED_ELEMENT = 127,
	STRUCT_ASSEMBLY = 128,
	MATERIAL_PROPERTY = 129,
	FUNC_EXTERNAL_FUNCTION = 130,
	MODEL_EXPORT_OPTIONS = 131,
	END = 132 };

  static const char* key_words[] = {
    "FEDEMMODELFILE",
    "MECHANISM",
    "ANALYSIS",
    "TRIAD",
    "LINK",
    "GRAPH",
    "CURVE_SET",
    "GLOBAL_VIEW_SETTINGS",
    "STICKER",
    "REV_JOINT",
    "BALL_JOINT",
    "FREE_JOINT",
    "RIGID_JOINT",
    "PRISM_JOINT",
    "CYL_JOINT",
    "AXIAL_DAMPER",
    "AXIAL_SPRING",
    "JOINT_SPRING",
    "JOINT_DAMPER",
    "LOAD",
    "REF_PLANE",
    "FUNC_LIN_VEL_VAR",
    "FUNC_CONSTANT",
    "FUNC_SINUSOIDAL",
    "FUNC_COMPL_SINUS",
    "FUNC_DELAYED_COMPL_SINUS",
    "FUNC_RAMP",
    "FUNC_STEP",
    "FUNC_SQUARE_PULS",
    "FUNC_LIN_VAR",
    "FUNC_SPLINE",
    "GEAR",
    "RACK_PINION",
    "FUNC_DIRAC_PULS",
    "FUNC_LIM_RAMP",
    "FUNC_SMOOTH_TRAJ",
    "ENGINE",
    "SENSOR",
    "RELATIVE_SENSOR",
    "EIGENMODE",
    "AIR_STATE",
    "FUNC_REV_JNT_FRICTION",
    "FUNC_PRISM_JNT_FRICTION",
    "FUNC_CAM_JNT_FRICTION",
    "FUNC_SCALE",
    "CONTROL_LINE",
    "CONTROL_AMPLIFIER",
    "CONTROL_POWER",
    "CONTROL_ADDER",
    "CONTROL_COMPARATOR",
    "CONTROL_INPUT",
    "CONTROL_OUTPUT",
    "CONTROL_DEAD_ZONE",
    "CONTROL_HYSTERESIS",
    "CONTROL_INTEGRATOR",
    "CONTROL_LIM_DERIVATOR",
    "CONTROL_LIMITATION",
    "CONTROL_LOGICAL_SWITCH",
    "CONTROL_MULTIPLIER",
    "CONTROL_PI",
    "CONTROL_SAMPLE_HOLD",
    "CONTROL_TIME_DELAY",
    "CONTROL_FIRST_ORDTF",
    "CONTROL_SEC_ORDTF",
    "CONTROL_COMPCONJPOLE",
    "CONTROL_PILIMD",
    "CONTROL_PD",
    "CONTROL_PID",
    "CONTROL_PLIMD",
    "CONTROL_PLIMI",
    "CONTROL_PLIMILIMD",
    "CONTROL_REAL_POLE",
    "CAM_JOINT",
    "ELEMENT_GROUP",
    "FUNC_DEVICE_FUNCTION",
    "ANIMATION",
    "MODESOPTIONS",
    "GAGEOPTIONS",
    "FPPOPTIONS",
    "DUTYCYCLEOPTIONS",
    "GENERIC_DB_OBJECT",
    "EXTERNAL_CTRL_SYSTEM",
    "FILE_REFERENCE",
    "FUNC_MATH_EXPRESSION",
    "TIRE",
    "ROAD",
    "STRAIN_ROSETTE",
    "JOINT_MOTION",
    "TRIAD_MOTION",
    "JOINT_LOAD",
    "SPRING_CHAR",
    "PIPE_SURFACE",
    "PIPE_STRING_DATA_EXPORTER",
    "ROT_FRICTION",
    "TRANS_FRICTION",
    "BEARING_FRICTION",
    "PRISMATIC_FRICTION",
    "CAM_FRICTION",
    "FUNC_WAVE_SINUS",
    "FUNC_WAVE_SPECTRUM",
    "VESSEL_MOTION",
    "MASTER_LINE",
    "MASTER_ARC_SEGMENT",
    "SIMULATION_EVENT",
    "DOF_MOTION",
    "DOF_LOAD",
    "FUNC_USER_DEFINED",
    "SEA_STATE",
    "SUBASSEMBLY",
    "RISER",
    "SOIL_PILE",
    "JACKET",
    "TURBINE",
    "TOWER",
    "NACELLE",
    "GENERATOR",
    "GEARBOX",
    "SHAFT",
    "ROTOR",
    "BLADE",
    "TURBINE_BLADE_DESIGN",
    "TURBINE_BLADE_PROPERTY",
    "BEAM_PROPERTY",
    "BEAMMATERIAL_PROPERTY",
    "BEAM",
    "PART",
    "USER_DEFINED_ELEMENT",
    "STRUCT_ASSEMBLY",
    "MATERIAL_PROPERTY",
    "FUNC_EXTERNAL_FUNCTION",
    "MODEL_EXPORT_OPTIONS",
    "END",
    NULL };


/*!
  Reads the model file named \a name into the database.

  First it does some checks on the file to find the version it was saved in.
  The version is kept in an FmDB-internal variable for further reference.

  The model file version number is parsed through the following scheme:
  * Files starting with "FEDEMMODELFILE" and something different from
    "{V.0.9b ASCII}" afterwards will have that text parsed into version number.
  * Files starting with "FEDEMMODELFILE {V.0.9b ASCII}" and with
    "Module version:" in the second line, will have the text following
    "Module version:" parsed into version number.
  * Files starting with "FEDEMMODELFILE {V.0.9b ASCII}", without
    "Module version:" in the second line but with "BASE_ID" within its first
    1000 lines, are assumed to be of version 2.5 - 2.5m1
  * Files starting with "FEDEMMODELFILE {V.0.9b ASCII}", without
    "Module version:" in the second line and no "BASE_ID" within its first
    1000 lines, are assumed to be of version 2.1.2
*/

bool FmDB::readAll(const std::string& name, char ignoreFileVersion)
{
#ifdef FM_DEBUG
  std::cout <<"FmDB::readAll() "<< name
            <<" "<< std::boolalpha << ignoreFileVersion << std::endl;
#endif

  std::ifstream fs(name.c_str(),std::ios::in);
  if (!fs) {
    FFaMsg::dialog("The file \"" + name + "\" could not be opened.\n"
		   "Please check that you have read permission on this file.",
		   FFaMsg::ERROR);
    return false;
  }

  // Find some version info from the first two lines of the file

  char line[256];
  fs.getline(line,80);
  std::string firstLine(line);
  if (firstLine.empty()) {
    FFaMsg::dialog("The file \"" + name + "\" is empty!",FFaMsg::ERROR);
    return false;
  }

  // Check the first line
  ourModelFileVersion.setVersion(0);
  if (firstLine.find("FEDEMMODELFILE") == std::string::npos)
    ListUI <<"===> WARNING: Opening a model file without proper header.\n"
	   <<"              This might cause problems.\n";
  else if (ignoreFileVersion && ignoreFileVersion != 'W')
    // We don't care about this model file version, set to current Fedem version
    ourModelFileVersion = ourCurrentFedemVersion;
  else if (firstLine.find("{V.0.9b ASCII}") == std::string::npos)
    // Try to parse version number from the first line
    ourModelFileVersion.parseLine(firstLine,'{');

  bool doRewind = false;
  if (ourModelFileVersion == 0)
  {
    // Check the second line
    fs.getline(line,128);
    std::string secondLine(line);
    if (secondLine.find("Module version:") != std::string::npos)
      ourModelFileVersion.parseLine(secondLine,':');
    else
      doRewind = true;
  }

  if (ourModelFileVersion == 0)
  {
    // Check for pre 2.5 file by trying to find the keyword BASE_ID in the file.
    // If it is not there we have a pre 2.5 file, most likely a 2.1.2 model.

    doRewind = true;
    bool is2_5 = false;
    const char* ident = "BASE_ID";
    for (int lCount = 0; fs.good() && lCount < 1000 && !is2_5; lCount++)
    {
      fs.getline(line,256);
      int firstLetterPos = 0;
      while (!isalpha(line[firstLetterPos]) && firstLetterPos < 128)
	firstLetterPos++;
      if (strncmp((const char*)&line[firstLetterPos],ident,strlen(ident)) == 0)
	is2_5 = true;
      else if (strncmp((const char*)&line[firstLetterPos],"END",3) == 0)
	break;
    }

    if (is2_5)
      ourModelFileVersion.setVersion(2,5,1);
    else
    {
      ListUI <<"===> WARNING: The model file "<< name <<" was last saved in Fedem 2.1.2\n"
	     <<"              or earlier. The file is converted, and will be written to\n"
	     <<"              disk in the current format at next save.\n";
      ourModelFileVersion.setVersion(2,1,2);
    }
  }

  // Ignore build number differences only
  FFaVersionNumber fedemVersion(ourCurrentFedemVersion);
  if (ourModelFileVersion > FFaVersionNumber(7,5))
    fedemVersion.set(4,ourModelFileVersion.get(4));

  if (ignoreFileVersion && ignoreFileVersion != 'W')
    ourSaveNr = 0;
  else if (ourModelFileVersion > fedemVersion && ignoreFileVersion != 'W')
  {
    FFaMsg::dialog("The file \"" + name + "\" was created in Fedem " +
                   ourModelFileVersion.getString() + ",\nwhich is a more "
                   "recent version than " + ourCurrentFedemVersion.getString() +
                   " that you are currently running.\n"
		   "Opening this model is prohibited to avoid model inconsistencies.\n\n"
		   "You have to upgrade to " + ourModelFileVersion.getString() +
		   " or later to be able to use this model.",FFaMsg::ERROR);
    return false;
  }
  else
  {
    ListUI <<"  -> Model file created by Fedem version : "
	   << ourModelFileVersion.getString() <<"  ["
	   << ourModelFileVersion.getInterpretedString() <<"]\n";

    ourSaveNr = 1; // Get the save number for this file, if any
    if (ourModelFileVersion > FFaVersionNumber(4,1,1))
      while (fs.getline(line,256) && line[0] == '!' && fs.good())
	if (strncmp(line,"!Last saved: #",14) == 0)
	{
	  ourSaveNr = atoi(line+14);
	  break;
	}

    if (ourSaveNr > 1)
      ListUI <<"  -> Save number : "<< ourSaveNr <<"\n";

    if (ourModelFileVersion > fedemVersion)
      FFaMsg::dialog("The file \"" + name + "\" was created in Fedem " +
                     ourModelFileVersion.getString() + ",\nwhich is a more "
                     "recent version than " + ourCurrentFedemVersion.getString() +
                     " of the current installation.\nBe aware that"
                     " opening this model may cause inconsistencies"
                     " due to recent changes in the model file format.",
                     FFaMsg::WARNING);
    else if (ourModelFileVersion < fedemVersion)
      if (!FFaMsg::dialog("The file \"" + name + "\" was created in Fedem " +
                          ourModelFileVersion.getString() + ",\nwhich is older "
                          "than the version you are currently running (" +
                          ourCurrentFedemVersion.getString() +").\n"
                          "If you continue and perform a \"Save\", the model"
                          " file will be updated to\nthe current version"
                          " and will no longer be usable in Fedem " +
                          ourModelFileVersion.getString(),FFaMsg::OK_CANCEL))
        return false;
  }

  // Try read the file nomatterwhat - see what happens

  readLog.clear();
  if (doRewind) fs.seekg(0,std::ios_base::beg);

  int dataIsRead = FmDB::readFMF(fs);

  if (!unknownKeywords.empty())
  {
    for (const std::pair<const std::string,int>& unknown : unknownKeywords)
      ListUI <<" ==> "<< unknown.first <<" ("<< unknown.second <<").\n";
    unknownKeywords.clear();
  }

  // Connect all (if any) multi-master objects that were created
  // due to parsing of old (R5.0 and older) model files.
  FmMMJointBase::connectTmpMasters();

  if (dataIsRead < 0) {
    FFaMsg::dialog("Parsing the file \"" + name + "\" aborted.\nIt has to "
		   "be manually corrected (see Output List for details).",
		   FFaMsg::ERROR);
    FmDB::newMechanism();
    return false;
  }
  else if (!dataIsRead)
    ListUI <<"===> WARNING: End-of-file reached before the END keyword.\n"
	   <<"              Possibly corrupted model file.\n";

  FFaMsg::setSubTask("Resolving topology");

  // Resolve the conflicting baseIDs, if any
  FmModelMemberBase::resolveBaseIDProblems();

  FFaDynCB2<bool&,FmBase*> headCB;
  FFaDynCB1<FmBase*> allCB;

  // Resolve references that are read through a field
  allCB = FFaDynCB1S(FmDB::resolveObject,FmBase*);
  FmDB::forAllInDB(headCB,allCB);

  // Erase curves without any owner graphs (to avoid crash later)
  std::vector<FmModelMemberBase*> allCurves;
  FmDB::getAllOfType(allCurves,FmCurveSet::getClassTypeID());
  for (FmModelMemberBase* curve : allCurves)
    if (!static_cast<FmCurveSet*>(curve)->getOwnerGraph())
    {
      ListUI <<" ==> "<< curve->getIdString(true)
	     <<" does not have an owner graph (erased).\n";
      curve->erase();
    }

  // Set up the other references and connections.
  // Make sure objects are initialized after resolving if necessary
  allCB = FFaDynCB1S(FmDB::initAfterResolveObject,FmBase*);
  FmDB::forAllInDB(headCB,allCB);

#ifdef FT_HAS_EXTCTRL
  // External ctrl systems need to read simulink file to give warnings
  // if the simulink file has changed or stuff
  std::vector<FmExternalCtrlSys*> allExtCtrlSys;
  FmDB::getAllExternalCtrlSys(allExtCtrlSys);
  for (FmExternalCtrlSys* ctrl : allExtCtrlSys)
    if (!ctrl->completeAfterParse())
      FFaMsg::dialog(ctrl->getErrorString());
#endif

  // Resolve functions
  FmMathFuncBase::resolveAfterRead();

  if (ourModelFileVersion < FFaVersionNumber(3,0,0,8))
  {
    // Convert non-linear dampers from pre-3.0i8.
    // InitDamp must be set to 1.0 to make the scale engine work properly
    std::vector<FmModelMemberBase*> allDampers;
    FmDB::getAllOfType(allDampers,FmDamperBase::getClassTypeID());
    for (FmModelMemberBase* damper : allDampers)
      if (static_cast<FmDamperBase*>(damper)->getDampEngine())
        static_cast<FmDamperBase*>(damper)->setInitDamp(1.0);
  }

  // Convert and sync engines and function references
  FmEngine::updateFunctionLinkedFromStuff();

  // Translate old obsolete info into new engine
  std::vector<FmEngine*> allEngines;
  FmDB::getAllEngines(allEngines);
  for (FmEngine* engine : allEngines)
    engine->translateJointSensorEntity();

  // Erase simple sensors not measuring anything. Replace it
  // (there should only be one) by the new TimeSensor in all Engines using it.
  // Most likely it measured the obsolete Time object in R4.1.1 and earlier.
  std::vector<FmModelMemberBase*> allSensors;
  FmDB::getAllOfType(allSensors,FmSimpleSensor::getClassTypeID());
  for (FmModelMemberBase* sensor : allSensors)
    if (!static_cast<FmSimpleSensor*>(sensor)->getMeasured()) {
      sensor->releaseReferencesToMe("mySensor",FmDB::getTimeSensor());
      sensor->erase();
    }

  // Make sure 3D location and coordinate systems are in sync
  std::vector<FmModelMemberBase*> allPosBases;
  FmDB::getAllOfType(allPosBases,FmIsPositionedBase::getClassTypeID());
  for (FmModelMemberBase* obj : allPosBases)
    static_cast<FmIsPositionedBase*>(obj)->updateLocation();
  FmDB::getAllOfType(allPosBases,FmSubAssembly::getClassTypeID());
  for (FmModelMemberBase* obj : allPosBases)
    static_cast<FmSubAssembly*>(obj)->updateLocation('T');

  FFaMsg::setSubTask("");

#ifdef FT_USE_CMDLINEARG
  int incID = 0;
  FFaCmdLineArg::instance()->getValue("ID_increment",incID);
  if (incID > 0) {
    ListUI <<"===> Incrementing all IDs with "<< incID <<"\n";
    for (const FmHeadMap::value_type& head : ourHeadMap)
      // Skip incrementing for the reference plane
      if (head.first != FmRefPlane::getClassTypeID())
        for (FmBase* p = head.second->getNext(); p != head.second; p = p->getNext())
          p->setID(p->getID() + incID);
  }
#endif

  // Conversion of old-style generic DB objects to cross sections and materials.
  // Before R7.0, we used a generic DB object that contained material, geometry
  // and hydrodynamic properties as a text-blob. Now separate cross section and
  // material objects are used instead. The following auto-upgrades the model.
  FmBeamProperty::convertFromGenericDBObjects();

  // End of file parsing: Write the log to FFaMsg::list
  if (FFaAppInfo::isConsole()) return true;

  FFaMsg::list("\n\nObject type:                   Count:\n"
	       "-------------------------------------\n");
  char tmpChar[256];
  for (const std::pair<const int,int>& log : readLog)
  {
    snprintf(tmpChar, 256, "%-26s%8i\n", key_words[log.first], log.second);
    FFaMsg::list(tmpChar);
  }

  FFaMsg::list("-------------------------------------\n");

  return true;
}


int FmDB::readFMF(std::istream& fs)
{
  int dataIsRead = 0;

#ifdef FM_DEBUG
#define DBG_PARSE std::cout <<"\nParsing "<< keyWord << std::endl;
#else
#define DBG_PARSE
#endif
#define FmdPARSE_AND_BUILD_LOG(objType) DBG_PARSE	    \
    if (objType::readAndConnect(statement,std::cout))	    \
      ++readLog[key-1];					    \
    else						    \
      dataIsRead = -1;

  int prevKey = -1;
  while (fs.good() && !dataIsRead)
    {
      std::stringstream statement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord,fs,statement,'{','}'))
	{
	  int key = FaParse::findIndex(key_words,keyWord);
	  if (key != prevKey) FFaMsg::setSubTask(keyWord);
	  prevKey = key;
	  switch (key)
	    {
	    case MECHANISM: FmdPARSE_AND_BUILD_LOG(FmMechanism); break;
	    case ANALYSIS: FmdPARSE_AND_BUILD_LOG(FmAnalysis); break;
	    case MODESOPTIONS: FmdPARSE_AND_BUILD_LOG(FmModesOptions); break;
	    case GAGEOPTIONS: FmdPARSE_AND_BUILD_LOG(FmGageOptions); break;
	    case FPPOPTIONS: FmdPARSE_AND_BUILD_LOG(FmFppOptions); break;
#ifdef FT_HAS_NCODE
	    case DUTYCYCLEOPTIONS: FmdPARSE_AND_BUILD_LOG(FmDutyCycleOptions); break;
#endif
	    case MODEL_EXPORT_OPTIONS: FmdPARSE_AND_BUILD_LOG(FmModelExpOptions); break;
	    case GENERIC_DB_OBJECT: FmdPARSE_AND_BUILD_LOG(FmGenericDBObject); break;
	    case FILE_REFERENCE: FmdPARSE_AND_BUILD_LOG(FmFileReference); break;
	    case TIRE: FmdPARSE_AND_BUILD_LOG(FmTire); break;
	    case ROAD: FmdPARSE_AND_BUILD_LOG(FmRoad); break;
	    case AXIAL_DAMPER: FmdPARSE_AND_BUILD_LOG(FmAxialDamper); break;
	    case AXIAL_SPRING: FmdPARSE_AND_BUILD_LOG(FmAxialSpring); break;
	    case BALL_JOINT: FmdPARSE_AND_BUILD_LOG(FmBallJoint); break;
	    case CAM_JOINT: FmdPARSE_AND_BUILD_LOG(FmCamJoint); break;
	    case CONTROL_ADDER: FmdPARSE_AND_BUILD_LOG(FmcAdder); break;
	    case CONTROL_AMPLIFIER: FmdPARSE_AND_BUILD_LOG(FmcAmplifier); break;
	    case CONTROL_POWER: FmdPARSE_AND_BUILD_LOG(FmcPower); break;
	    case CONTROL_COMPARATOR: FmdPARSE_AND_BUILD_LOG(FmcComparator); break;
	    case CONTROL_COMPCONJPOLE: FmdPARSE_AND_BUILD_LOG(FmcCompConjPole); break;
	    case CONTROL_DEAD_ZONE: FmdPARSE_AND_BUILD_LOG(FmcDeadZone); break;
	    case CONTROL_FIRST_ORDTF: FmdPARSE_AND_BUILD_LOG(Fmc1ordTF); break;
	    case CONTROL_HYSTERESIS: FmdPARSE_AND_BUILD_LOG(FmcHysteresis); break;
	    case CONTROL_INPUT: FmdPARSE_AND_BUILD_LOG(FmcInput); break;
	    case CONTROL_INTEGRATOR: FmdPARSE_AND_BUILD_LOG(FmcIntegrator); break;
	    case CONTROL_LIMITATION: FmdPARSE_AND_BUILD_LOG(FmcLimitation); break;
	    case CONTROL_LIM_DERIVATOR: FmdPARSE_AND_BUILD_LOG(FmcLimDerivator); break;
	    case CONTROL_LINE: FmdPARSE_AND_BUILD_LOG(FmCtrlLine); break;
	    case CONTROL_LOGICAL_SWITCH: FmdPARSE_AND_BUILD_LOG(FmcLogicalSwitch); break;
	    case CONTROL_MULTIPLIER: FmdPARSE_AND_BUILD_LOG(FmcMultiplier); break;
	    case CONTROL_OUTPUT: FmdPARSE_AND_BUILD_LOG(FmcOutput); break;
	    case CONTROL_PD: FmdPARSE_AND_BUILD_LOG(FmcPd); break;
	    case CONTROL_PI: FmdPARSE_AND_BUILD_LOG(FmcPi); break;
	    case CONTROL_PID: FmdPARSE_AND_BUILD_LOG(FmcPid); break;
	    case CONTROL_PILIMD: FmdPARSE_AND_BUILD_LOG(FmcPIlimD); break;
	    case CONTROL_PLIMD: FmdPARSE_AND_BUILD_LOG(FmcPlimD); break;
	    case CONTROL_PLIMI: FmdPARSE_AND_BUILD_LOG(FmcPlimI); break;
	    case CONTROL_PLIMILIMD: FmdPARSE_AND_BUILD_LOG(FmcPlimIlimD); break;
	    case CONTROL_REAL_POLE: FmdPARSE_AND_BUILD_LOG(FmcRealPole); break;
	    case CONTROL_SAMPLE_HOLD: FmdPARSE_AND_BUILD_LOG(FmcSampleHold); break;
	    case CONTROL_SEC_ORDTF: FmdPARSE_AND_BUILD_LOG(Fmc2ordTF); break;
	    case CONTROL_TIME_DELAY: FmdPARSE_AND_BUILD_LOG(FmcTimeDelay); break;
	    case CURVE_SET: FmdPARSE_AND_BUILD_LOG(FmCurveSet); break;
	    case CYL_JOINT: FmdPARSE_AND_BUILD_LOG(FmCylJoint); break;
	    case EIGENMODE: FmdPARSE_AND_BUILD_LOG(FmModesOptions); break;
	    case ELEMENT_GROUP: FmdPARSE_AND_BUILD_LOG(FmElementGroupProxy); break;
	    case ENGINE: FmdPARSE_AND_BUILD_LOG(FmEngine); break;
#ifdef FT_HAS_EXTCTRL
	    case EXTERNAL_CTRL_SYSTEM: FmdPARSE_AND_BUILD_LOG(FmExternalCtrlSys); break;
#endif
	    case FREE_JOINT: FmdPARSE_AND_BUILD_LOG(FmFreeJoint); break;
	    case FUNC_COMPL_SINUS: FmdPARSE_AND_BUILD_LOG(FmfComplSinus); break;
	    case FUNC_CONSTANT: FmdPARSE_AND_BUILD_LOG(FmfConstant); break;
	    case FUNC_MATH_EXPRESSION: FmdPARSE_AND_BUILD_LOG(FmfMathExpr); break;
	    case FUNC_DEVICE_FUNCTION: FmdPARSE_AND_BUILD_LOG(FmfDeviceFunction); break;
	    case FUNC_EXTERNAL_FUNCTION: FmdPARSE_AND_BUILD_LOG(FmfExternalFunction); break;
	    case FUNC_DELAYED_COMPL_SINUS: FmdPARSE_AND_BUILD_LOG(FmfDelayedComplSinus); break;
	    case FUNC_WAVE_SINUS: FmdPARSE_AND_BUILD_LOG(FmfWaveSinus); break;
	    case FUNC_WAVE_SPECTRUM: FmdPARSE_AND_BUILD_LOG(FmfWaveSpectrum); break;
	    case FUNC_DIRAC_PULS: FmdPARSE_AND_BUILD_LOG(FmfDiracPuls); break;
	    case FUNC_LIM_RAMP: FmdPARSE_AND_BUILD_LOG(FmfLimRamp); break;
	    case FUNC_LIN_VAR: FmdPARSE_AND_BUILD_LOG(FmfLinVar); break;
	    case FUNC_LIN_VEL_VAR: FmdPARSE_AND_BUILD_LOG(FmfLinVelVar); break;
	    case FUNC_RAMP: FmdPARSE_AND_BUILD_LOG(FmfRamp); break;
	    case ROT_FRICTION: FmdPARSE_AND_BUILD_LOG(FmRotFriction); break;
	    case TRANS_FRICTION: FmdPARSE_AND_BUILD_LOG(FmTransFriction); break;
	    case FUNC_REV_JNT_FRICTION: // For backward compatibility
	    case BEARING_FRICTION: FmdPARSE_AND_BUILD_LOG(FmBearingFriction); break;
	    case FUNC_PRISM_JNT_FRICTION: // For backward compatibility
	    case PRISMATIC_FRICTION: FmdPARSE_AND_BUILD_LOG(FmPrismaticFriction); break;
	    case FUNC_CAM_JNT_FRICTION: // For backward compatibility
	    case CAM_FRICTION: FmdPARSE_AND_BUILD_LOG(FmCamFriction); break;
	    case FUNC_SCALE: FmdPARSE_AND_BUILD_LOG(FmfScale); break;
	    case FUNC_SINUSOIDAL: FmdPARSE_AND_BUILD_LOG(FmfSinusoidal); break;
	    case FUNC_SMOOTH_TRAJ: FmdPARSE_AND_BUILD_LOG(FmfSmoothTraj); break;
	    case FUNC_SPLINE: FmdPARSE_AND_BUILD_LOG(FmfSpline); break;
	    case FUNC_SQUARE_PULS: FmdPARSE_AND_BUILD_LOG(FmfSquarePuls); break;
	    case FUNC_STEP: FmdPARSE_AND_BUILD_LOG(FmfStep); break;
	    case FUNC_USER_DEFINED: FmdPARSE_AND_BUILD_LOG(FmfUserDefined); break;
	    case GEAR: FmdPARSE_AND_BUILD_LOG(FmGear); break;
	    case GLOBAL_VIEW_SETTINGS: FmdPARSE_AND_BUILD_LOG(FmGlobalViewSettings); break;
	    case ANIMATION: FmdPARSE_AND_BUILD_LOG(FmAnimation); break;
	    case GRAPH: FmdPARSE_AND_BUILD_LOG(FmGraph); break;
	    case JOINT_DAMPER: FmdPARSE_AND_BUILD_LOG(FmJointDamper); break;
	    case JOINT_SPRING: FmdPARSE_AND_BUILD_LOG(FmJointSpring); break;
	    case JOINT_MOTION: FmdPARSE_AND_BUILD_LOG(FmJointMotion); break;
	    case JOINT_LOAD: // For backward compatibility
	    case DOF_LOAD: FmdPARSE_AND_BUILD_LOG(FmDofLoad); break;
	    case LINK: FmdPARSE_AND_BUILD_LOG(FmLink); break;
	    case PART: FmdPARSE_AND_BUILD_LOG(FmPart); break;
	    case BEAM: FmdPARSE_AND_BUILD_LOG(FmBeam); break;
	    case LOAD: FmdPARSE_AND_BUILD_LOG(FmLoad); break;
	    case PRISM_JOINT: FmdPARSE_AND_BUILD_LOG(FmPrismJoint); break;
	    case RACK_PINION: FmdPARSE_AND_BUILD_LOG(FmRackPinion); break;
	    case REF_PLANE: FmdPARSE_AND_BUILD_LOG(FmRefPlane); break;
	    case RELATIVE_SENSOR: FmdPARSE_AND_BUILD_LOG(FmRelativeSensor); break;
	    case REV_JOINT: FmdPARSE_AND_BUILD_LOG(FmRevJoint); break;
	    case RIGID_JOINT: FmdPARSE_AND_BUILD_LOG(FmRigidJoint); break;
	    case SENSOR: FmdPARSE_AND_BUILD_LOG(FmSimpleSensor); break;
	    case SPRING_CHAR: FmdPARSE_AND_BUILD_LOG(FmSpringChar); break;
	    case STICKER: FmdPARSE_AND_BUILD_LOG(FmSticker); break;
	    case TRIAD: FmdPARSE_AND_BUILD_LOG(FmTriad); break;
	    case STRAIN_ROSETTE: FmdPARSE_AND_BUILD_LOG(FmStrainRosette); break;
	    case TRIAD_MOTION: // For backward compatibility
	    case DOF_MOTION: FmdPARSE_AND_BUILD_LOG(FmDofMotion); break;
	    case MASTER_LINE: FmdPARSE_AND_BUILD_LOG(FmStraightMaster); break;
	    case MASTER_ARC_SEGMENT: FmdPARSE_AND_BUILD_LOG(FmArcSegmentMaster); break;
	    case PIPE_SURFACE: FmdPARSE_AND_BUILD_LOG(FmPipeSurface); break;
	    case PIPE_STRING_EXPORTER: FmdPARSE_AND_BUILD_LOG(FmPipeStringDataExporter); break;
	    case VESSEL_MOTION: FmdPARSE_AND_BUILD_LOG(FmVesselMotion); break;
	    case SIMULATION_EVENT: FmdPARSE_AND_BUILD_LOG(FmSimulationEvent); break;
	    case SEA_STATE: FmdPARSE_AND_BUILD_LOG(FmSeaState); break;
	    case AIR_STATE: FmdPARSE_AND_BUILD_LOG(FmAirState); break;
	    case SUBASSEMBLY: FmdPARSE_AND_BUILD_LOG(FmSubAssembly); break;
	    case STRUCT_ASSEMBLY: FmdPARSE_AND_BUILD_LOG(FmStructAssembly); break;
	    case RISER: FmdPARSE_AND_BUILD_LOG(FmRiser); break;
	    case SOIL_PILE: FmdPARSE_AND_BUILD_LOG(FmSoilPile); break;
	    case JACKET: FmdPARSE_AND_BUILD_LOG(FmJacket); break;
	    case TURBINE: FmdPARSE_AND_BUILD_LOG(FmTurbine); break;
	    case TOWER: FmdPARSE_AND_BUILD_LOG(FmTower); break;
	    case NACELLE: FmdPARSE_AND_BUILD_LOG(FmNacelle); break;
	    case GENERATOR: FmdPARSE_AND_BUILD_LOG(FmGenerator); break;
	    case GEARBOX: FmdPARSE_AND_BUILD_LOG(FmGearBox); break;
	    case SHAFT: FmdPARSE_AND_BUILD_LOG(FmShaft); break;
	    case ROTOR: FmdPARSE_AND_BUILD_LOG(FmRotor); break;
	    case BLADE: FmdPARSE_AND_BUILD_LOG(FmBlade); break;
	    case TURBINE_BLADE_DESIGN: FmdPARSE_AND_BUILD_LOG(FmBladeDesign); break;
	    case TURBINE_BLADE_PROPERTY: FmdPARSE_AND_BUILD_LOG(FmBladeProperty); break;
	    case BEAM_PROPERTY: FmdPARSE_AND_BUILD_LOG(FmBeamProperty); break;
	    case BEAMMATERIAL_PROPERTY: // For backward compatibility
	    case MATERIAL_PROPERTY: FmdPARSE_AND_BUILD_LOG(FmMaterialProperty); break;
	    case USER_DEFINED_ELEMENT: FmdPARSE_AND_BUILD_LOG(FmUserDefinedElement); break;
	    case FEDEMMODELFILE: break; // Avoid warning when rewinding old model files
	    case END: dataIsRead = true; break;

	    default:
	      ListUI <<"===> WARNING: unknown keyword: "<< keyWord <<"\n";
	      break;
	    }
	}
    }

  return dataIsRead;
}


static void findContainer(FFaFieldContainer*& found, int typeID, int ID,
			  const std::vector<int>& assemblyID)
{
  found = FmDB::findID(typeID,ID,assemblyID);
}


void FmDB::resolveObject(FmBase* obj)
{
  static FFaDynCB4<FFaFieldContainer*&,int,int,const std::vector<int>&> findCB = FFaDynCB4S(findContainer,FFaFieldContainer*&,int,int,const std::vector<int>&);

  if (obj)
    obj->resolve(findCB);
}


void FmDB::initAfterResolveObject(FmBase* obj)
{
  if (obj)
    obj->initAfterResolve();
}
