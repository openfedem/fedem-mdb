// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_IS_MEASURED_BASE_H
#define FM_IS_MEASURED_BASE_H

#include "vpmDB/FmSimulationModelBase.H"

class FmSensorBase;
using FmSensorChoice  = std::pair<unsigned short int,std::string>;
using FmSensorChoices = std::vector<FmSensorChoice>;


class FmIsMeasuredBase : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSensorBase* getSimpleSensor() const;

  bool hasSensors() const;

  virtual void updateChildrenDisplayTopology();

  virtual void getEntities(FmSensorChoices& ents, int = -1) { ents.clear(); }
  virtual void getDofs(FmSensorChoices& dofs) { dofs.clear(); }

  // Joint springs/dampers can be measured without having sensors when the joint
  // itself has the sensor, thus the virtual method isMeasured() to remedy this
  virtual bool isMeasured() const { return this->hasSensors(); }

  enum SensorDOF {
    X_TRANS = 0,
    Y_TRANS = 1,
    Z_TRANS = 2,
    X_ROT = 3,
    Y_ROT = 4,
    Z_ROT = 5,
    REL = 6,
    REL_X = 7,
    REL_Y = 8,
    REL_Z = 9,
    REL_RX = 10,
    REL_RY = 11,
    REL_RZ = 12,
    MAX_PR = 13,
    MIN_PR = 14,
    SA_MAX = 15,
    VMISES = 16,
    GAGE_1 = 17,
    GAGE_2 = 18,
    GAGE_3 = 19,
    UNSIGNED = 20,
    ANGLE_YZ = 21,
    ANGLE_ZX = 22,
    ANGLE_XY = 23,
    NUM_DOF = 24
  };

  enum SensorEntity {
    POS = 0,
    LOCAL_VEL = 1,
    GLOBAL_VEL = 2,
    LOCAL_ACC = 3,
    GLOBAL_ACC = 4,
    DISTANCE = 5, // relative sensor
    VEL = 6,
    ACCEL = 7,
    ANGLE = 8, // relative sensor
    REL_POS = 9, // joint variable
    JSPR_ANG = 10,
    JSPR_DEFL = 11,
    JSPR_FORCE = 12,
    JDAMP_ANG = 13,
    JDAMP_VEL = 14,
    JDAMP_FORCE = 15,
    LENGTH = 16, // axial spring and damper
    DEFL = 17,
    FORCE = 18,
    LOCAL_FORCE = 19,
    GLOBAL_FORCE = 20,
    WIND_SPEED = 21,
    FLUID_VEL = 22,
    FLUID_ACC = 23,
    DYN_PRESS = 24,
    STRAIN = 25,
    STRESS = 26,
    NUM_ENTITY = 27
  };

protected:
  FmIsMeasuredBase();
  virtual ~FmIsMeasuredBase();

  virtual bool cloneLocal(FmBase* obj, int depth);

  static FmSensorChoice itsDofTable[NUM_DOF];
  static FmSensorChoice itsEntityTable[NUM_ENTITY];
};

#endif
