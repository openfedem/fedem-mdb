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

typedef std::pair<unsigned short int,std::string> FmSensorChoice;


class FmIsMeasuredBase : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSensorBase* getSimpleSensor(bool createIfNone = false);
  FmSensorBase* getRelativeSensor(FmIsMeasuredBase* that,
                                  bool createIfNone = false);

  bool hasSensors() const;

  virtual void updateChildrenDisplayTopology();

  virtual void getEntities(std::vector<FmSensorChoice>& toFill, int = -1);
  virtual void getDofs(std::vector<FmSensorChoice>& toFill);

  // For instance joint springs can be measured without having sensors since the
  // joint itself has the sensor, thus the method isMeasured to remedy this
  virtual bool isMeasured() const;

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
    NUM_DOF = 20
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
    REL_POS = 8, // joint variable
    JSPR_ANG = 9,
    JSPR_DEFL = 10,
    JSPR_FORCE = 11,
    JDAMP_ANG = 12,
    JDAMP_VEL = 13,
    JDAMP_FORCE = 14,
    LENGTH = 15, // axial spring and damper
    DEFL = 16,
    FORCE = 17,
    LOCAL_FORCE = 18,
    GLOBAL_FORCE = 19,
    WIND_SPEED = 20,
    FLUID_VEL = 21,
    FLUID_ACC = 22,
    DYN_PRESS = 23,
    STRAIN = 24,
    STRESS = 25,
    NUM_ENTITY = 26
  };

protected:
  FmIsMeasuredBase();
  virtual ~FmIsMeasuredBase();

  virtual bool cloneLocal(FmBase* obj, int depth);

  static FmSensorChoice itsDofTable[NUM_DOF];
  static FmSensorChoice itsEntityTable[NUM_ENTITY];
};

#endif
