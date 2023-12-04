// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmIsMeasuredBase.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmRelativeSensor.H"

Fmd_DB_SOURCE_INIT(FcIS_MEASURED_BASE, FmIsMeasuredBase, FmSimulationModelBase);


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


FmSensorBase* FmIsMeasuredBase::getSimpleSensor(bool createIfNone)
{
  FmSimpleSensor* sensor = NULL;
  if (this->hasReferringObjs(sensor,"itsMeasuredPt") || !createIfNone)
    return sensor;

  sensor = new FmSimpleSensor();
  sensor->setUserDescription("Sensor on " + this->getIdString());
  sensor->setParentAssembly(this->getParentAssembly());
  sensor->setMeasured(this);
  sensor->connect();
  sensor->draw();

  return sensor;
}


FmSensorBase* FmIsMeasuredBase::getRelativeSensor(FmIsMeasuredBase* that,
                                                  bool createIfNone)
{
  std::vector<FmRelativeSensor*> sensors;
  this->getReferringObjs(sensors);
  for (FmRelativeSensor* sensor : sensors)
    if (sensor->getMeasured(1) == this &&
        sensor->getMeasured(2) == this)
      return sensor;

  if (!createIfNone)
    return NULL;

  FmRelativeSensor* sensor = new FmRelativeSensor();
  sensor->setUserDescription("Relative sensor between "+
                             this->getIdString() +" and "+ that->getIdString());
  sensor->setParentAssembly(this->getCommonAncestor(that));
  sensor->connect(this,that);
  sensor->draw();

  return sensor;
}


bool FmIsMeasuredBase::hasSensors() const
{
  FmSensorBase* sensor = NULL;
  return this->hasReferringObjs(sensor);
}


bool FmIsMeasuredBase::isMeasured() const
{
  return this->hasSensors();
}


void FmIsMeasuredBase::getEntities(std::vector<FmSensorChoice>& toFill, int)
{
  toFill.clear();
}


void FmIsMeasuredBase::getDofs(std::vector<FmSensorChoice>& toFill)
{
  toFill.clear();
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


FmSensorChoice FmIsMeasuredBase::itsDofTable[] = {
  FmSensorChoice(FmIsMeasuredBase::X_TRANS,"X trans."),
  FmSensorChoice(FmIsMeasuredBase::Y_TRANS,"Y trans."),
  FmSensorChoice(FmIsMeasuredBase::Z_TRANS,"Z trans."),
  FmSensorChoice(FmIsMeasuredBase::X_ROT,"X rot."),
  FmSensorChoice(FmIsMeasuredBase::Y_ROT,"Y rot."),
  FmSensorChoice(FmIsMeasuredBase::Z_ROT,"Z rot."),
  FmSensorChoice(FmIsMeasuredBase::REL,"Length"),
  FmSensorChoice(FmIsMeasuredBase::REL_X,"Global DX"),
  FmSensorChoice(FmIsMeasuredBase::REL_Y,"Global DY"),
  FmSensorChoice(FmIsMeasuredBase::REL_Z,"Global DZ"),
  FmSensorChoice(FmIsMeasuredBase::REL_RX,"Rel X rot."),
  FmSensorChoice(FmIsMeasuredBase::REL_RY,"Rel Y rot."),
  FmSensorChoice(FmIsMeasuredBase::REL_RZ,"Rel Z rot."),
  FmSensorChoice(FmIsMeasuredBase::MAX_PR,"Max principal"),
  FmSensorChoice(FmIsMeasuredBase::MIN_PR,"Min principal"),
  FmSensorChoice(FmIsMeasuredBase::SA_MAX,"Signed Abs Max"),
  FmSensorChoice(FmIsMeasuredBase::VMISES,"von Mises"),
  FmSensorChoice(FmIsMeasuredBase::GAGE_1,"Gage 1"),
  FmSensorChoice(FmIsMeasuredBase::GAGE_2,"Gage 2"),
  FmSensorChoice(FmIsMeasuredBase::GAGE_3,"Gage 3")
};


FmSensorChoice FmIsMeasuredBase::itsEntityTable[] = {
  FmSensorChoice(FmIsMeasuredBase::POS,"Position, global coordinates"),
  FmSensorChoice(FmIsMeasuredBase::LOCAL_VEL,"Velocity, local coordinates"),
  FmSensorChoice(FmIsMeasuredBase::GLOBAL_VEL,"Velocity, global coordinates"),
  FmSensorChoice(FmIsMeasuredBase::LOCAL_ACC,"Acceleration, local coordinates"),
  FmSensorChoice(FmIsMeasuredBase::GLOBAL_ACC,"Acceleration, global coordinates"),
  FmSensorChoice(FmIsMeasuredBase::DISTANCE,"Distance"),
  FmSensorChoice(FmIsMeasuredBase::VEL,"Velocity"),
  FmSensorChoice(FmIsMeasuredBase::ACCEL,"Acceleration"),
  FmSensorChoice(FmIsMeasuredBase::REL_POS,"Length/angle"), // joint variable
  FmSensorChoice(FmIsMeasuredBase::JSPR_ANG,"Spring length/angle"),
  FmSensorChoice(FmIsMeasuredBase::JSPR_DEFL,"Spring deflection"),
  FmSensorChoice(FmIsMeasuredBase::JSPR_FORCE,"Spring force"),
  FmSensorChoice(FmIsMeasuredBase::JDAMP_ANG,"Damper length/angle"),
  FmSensorChoice(FmIsMeasuredBase::JDAMP_VEL,"Damper velocity"),
  FmSensorChoice(FmIsMeasuredBase::JDAMP_FORCE,"Damper force"),
  FmSensorChoice(FmIsMeasuredBase::LENGTH,"Length"), // axial spring and damper
  FmSensorChoice(FmIsMeasuredBase::DEFL,"Deflection"),
  FmSensorChoice(FmIsMeasuredBase::FORCE,"Force"),
  FmSensorChoice(FmIsMeasuredBase::LOCAL_FORCE, "Force, local coordinates"),
  FmSensorChoice(FmIsMeasuredBase::GLOBAL_FORCE,"Force, global coordinates"),
  FmSensorChoice(FmIsMeasuredBase::WIND_SPEED,"Wind speed"),
  FmSensorChoice(FmIsMeasuredBase::FLUID_VEL,"Fluid particle velocity"),
  FmSensorChoice(FmIsMeasuredBase::FLUID_ACC,"Fluid particle acceleration"),
  FmSensorChoice(FmIsMeasuredBase::DYN_PRESS,"Dynamic pressure from fluid"),
  FmSensorChoice(FmIsMeasuredBase::STRAIN,"Strain"),
  FmSensorChoice(FmIsMeasuredBase::STRESS,"Stress")
};
