// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmIsMeasuredBase.C
  \brief Base class for all objects with measurable response quantities.
*/

#include "vpmDB/FmIsMeasuredBase.H"
#include "vpmDB/FmSimpleSensor.H"


//! \cond DO_NOT_DOCUMENT
Fmd_DB_SOURCE_INIT(FcIS_MEASURED_BASE, FmIsMeasuredBase, FmSimulationModelBase);
//! \endcond


FmIsMeasuredBase::FmIsMeasuredBase()
{
  Fmd_CONSTRUCTOR_INIT(FmIsMeasuredBase);

  // Add the base ID to the list of fields to be saved in the model file, as
  // all child class objects are assumed to be input to the dynamics solver.
  // They should therefore preserve the same base ID from session to session.
  FFA_FIELD_INIT(myBaseID, -1, "BASE_ID");
}


FmIsMeasuredBase::~FmIsMeasuredBase()
{
  FmSensorBase* sensor = NULL;
  while (this->hasReferringObjs(sensor))
  {
    sensor->removeMeasured();
    sensor->erase();
  }
}


void FmIsMeasuredBase::updateChildrenDisplayTopology()
{
  std::vector<FmSensorBase*> sensors;
  this->getReferringObjs(sensors);
  for (FmSensorBase* sensor : sensors)
    sensor->updateTopologyInViewer();
}


FmSensorBase* FmIsMeasuredBase::getSimpleSensor() const
{
  FmSimpleSensor* sensor = NULL;
  this->hasReferringObjs(sensor,"itsMeasuredPt");
  return sensor;
}


bool FmIsMeasuredBase::hasSensors() const
{
  FmSensorBase* sensor = NULL;
  return this->hasReferringObjs(sensor);
}


bool FmIsMeasuredBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmIsMeasuredBase::getClassTypeID()))
    return false;

  if (depth == FmBase::DEEP_REPLACE)
  {
    obj->releaseReferencesToMe("itsMeasuredPt",this);
    obj->releaseReferencesToMe("itsMeasure",this);
  }

  return true;
}


std::array<FmSensorChoice,FmIsMeasuredBase::NUM_DOF>
FmIsMeasuredBase::itsDofTable{{
  { FmIsMeasuredBase::X_TRANS, "X trans." },
  { FmIsMeasuredBase::Y_TRANS, "Y trans." },
  { FmIsMeasuredBase::Z_TRANS, "Z trans." },
  { FmIsMeasuredBase::X_ROT, "X rot." },
  { FmIsMeasuredBase::Y_ROT, "Y rot." },
  { FmIsMeasuredBase::Z_ROT, "Z rot." },
  { FmIsMeasuredBase::REL, "Length" },
  { FmIsMeasuredBase::REL_X, "Global DX" },
  { FmIsMeasuredBase::REL_Y, "Global DY" },
  { FmIsMeasuredBase::REL_Z, "Global DZ" },
  { FmIsMeasuredBase::REL_RX, "Rel X rot." },
  { FmIsMeasuredBase::REL_RY, "Rel Y rot." },
  { FmIsMeasuredBase::REL_RZ, "Rel Z rot." },
  { FmIsMeasuredBase::MAX_PR, "Max principal" },
  { FmIsMeasuredBase::MIN_PR, "Min principal" },
  { FmIsMeasuredBase::SA_MAX, "Signed Abs Max" },
  { FmIsMeasuredBase::VMISES, "von Mises" },
  { FmIsMeasuredBase::GAGE_1, "Gage 1" },
  { FmIsMeasuredBase::GAGE_2, "Gage 2" },
  { FmIsMeasuredBase::GAGE_3, "Gage 3" },
  { FmIsMeasuredBase::UNSIGNED,"Unsigned angle" },
  { FmIsMeasuredBase::ANGLE_YZ, "Angle in global YZ-plane" },
  { FmIsMeasuredBase::ANGLE_ZX, "Angle in global ZX-plane" },
  { FmIsMeasuredBase::ANGLE_XY, "Angle in global XY-plane" }
}};


std::array<FmSensorChoice,FmIsMeasuredBase::NUM_ENTITY>
FmIsMeasuredBase::itsEntityTable{{
  { FmIsMeasuredBase::POS, "Position, global coordinates" },
  { FmIsMeasuredBase::LOCAL_VEL, "Velocity, local coordinates" },
  { FmIsMeasuredBase::GLOBAL_VEL, "Velocity, global coordinates" },
  { FmIsMeasuredBase::LOCAL_ACC, "Acceleration, local coordinates" },
  { FmIsMeasuredBase::GLOBAL_ACC, "Acceleration, global coordinates" },
  { FmIsMeasuredBase::DISTANCE, "Distance" },
  { FmIsMeasuredBase::VEL, "Velocity" },
  { FmIsMeasuredBase::ACCEL, "Acceleration" },
  { FmIsMeasuredBase::ANGLE, "Angle" },
  { FmIsMeasuredBase::REL_POS, "Length/angle" },
  { FmIsMeasuredBase::JSPR_ANG, "Spring length/angle" },
  { FmIsMeasuredBase::JSPR_DEFL, "Spring deflection" },
  { FmIsMeasuredBase::JSPR_FORCE, "Spring force" },
  { FmIsMeasuredBase::JDAMP_ANG, "Damper length/angle" },
  { FmIsMeasuredBase::JDAMP_VEL, "Damper velocity" },
  { FmIsMeasuredBase::JDAMP_FORCE, "Damper force" },
  { FmIsMeasuredBase::LENGTH, "Length" },
  { FmIsMeasuredBase::DEFL, "Deflection" },
  { FmIsMeasuredBase::FORCE, "Force" },
  { FmIsMeasuredBase::LOCAL_FORCE,  "Force, local coordinates" },
  { FmIsMeasuredBase::GLOBAL_FORCE, "Force, global coordinates" },
  { FmIsMeasuredBase::WIND_SPEED, "Wind speed" },
  { FmIsMeasuredBase::FLUID_VEL, "Fluid particle velocity" },
  { FmIsMeasuredBase::FLUID_ACC, "Fluid particle acceleration" },
  { FmIsMeasuredBase::DYN_PRESS, "Dynamic pressure from fluid" },
  { FmIsMeasuredBase::STRAIN, "Strain" },
  { FmIsMeasuredBase::STRESS, "Stress" }
}};
