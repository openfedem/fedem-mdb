// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmIsMeasuredBase.H
  \brief Base class for all objects with measurable response quantities.
*/

#ifndef FM_IS_MEASURED_BASE_H
#define FM_IS_MEASURED_BASE_H

#include "vpmDB/FmSimulationModelBase.H"
#include <array>

class FmSensorBase;

//! Convenience type alias for named sensor quantities
using FmSensorChoice  = std::pair<unsigned short int,std::string>;
//! Convenience type alias for lists of named sensor quantities
using FmSensorChoices = std::vector<FmSensorChoice>;


/*!
  \brief Base class for all objects with measurable response quantities.
*/

class FmIsMeasuredBase : public FmSimulationModelBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
  //! \brief Returns the simple sensor measuring quantities in this object.
  FmSensorBase* getSimpleSensor() const;

  //! \brief Returns \e true if a sensor is measuring this object.
  bool hasSensors() const;

  //! \brief Updates the visualization of other objects connected to \a *this.
  virtual void updateChildrenDisplayTopology();

  //! \brief Returns the relevant entries from #itsDofTable for this object.
  virtual void getEntities(FmSensorChoices& ents, int = -1) { ents.clear(); }
  //! \brief Returns the relevant entries from #itsEntityTable for this object.
  virtual void getDofs(FmSensorChoices& dofs) { dofs.clear(); }

  //! \brief Returns \e true, if this object has measured response quantities.
  //! \details Joint springs and dampers can be measured without having sensors
  //! when the joint itself has the sensor. This method is therefore virtual
  //! to remedy this (overridden in FmJointSpring and FmJointDamper).
  virtual bool isMeasured() const { return this->hasSensors(); }

  //! \brief Enumerator for the DOF choices that may exist.
  enum SensorDOF {
    // Triad and joint DOF sensors
    X_TRANS = 0,   //!< Global X-translation
    Y_TRANS = 1,   //!< Global Y-translation
    Z_TRANS = 2,   //!< Global Z-translation
    X_ROT = 3,     //!< Global X-rotation
    Y_ROT = 4,     //!< Global Y-rotation
    Z_ROT = 5,     //!< Global Z-rotation
    // Relative sensors
    REL = 6,       //!< Distance between two triads
    REL_X = 7,     //!< Relative X-translation between two triads
    REL_Y = 8,     //!< Relative Y-translation between two triads
    REL_Z = 9,     //!< Relative Z-translation between two triads
    REL_RX = 10,   //!< Relative angle in the YZ-plane between two triads
    REL_RY = 11,   //!< Relative angle in the ZX-plane between two triads
    REL_RZ = 12,   //!< Relative angle in the XY-plane between two triads
    // Strain rosette sensors
    MAX_PR = 13,   //!< Maximum principal stress/strain
    MIN_PR = 14,   //!< Minimum principal stress/strain
    SA_MAX = 15,   //!< Signend absolute max stress/strain
    VMISES = 16,   //!< von Mises stress/strain
    GAGE_1 = 17,   //!< First gage direction
    GAGE_2 = 18,   //!< Second gage direction
    GAGE_3 = 19,   //!< Third gage direction
    // Relative sensors between two lines defined by 4 triads
    UNSIGNED = 20, //!< Unsigned angle between two lines in 3D space
    ANGLE_YZ = 21, //!< Signed angle in the YZ-plane
    ANGLE_ZX = 22, //!< Signed angle in the ZX-plane
    ANGLE_XY = 23, //!< Signed angle in the XY-plane
    // For array dimensioning
    NUM_DOF = 24   //!< Total number of DOF type chices
  };

  //! \brief Enumerator for the response entity that may exist.
  enum SensorEntity {
    POS = 0,           //!< Position in global coordinates
    LOCAL_VEL = 1,     //!< Velocity in local coordinates
    GLOBAL_VEL = 2,    //!< Velocity in global coordinates
    LOCAL_ACC = 3,     //!< Acceleration in local coordinates
    GLOBAL_ACC = 4,    //!< Acceleration in global coordinates
    DISTANCE = 5,      //!< Distance measured by relative sensors
    VEL = 6,           //!< Velocity measured by joint DOF sensors
    ACCEL = 7,         //!< Acceleration measured by joint DOF sensors
    ANGLE = 8,         //!< Angle measured by relative sensors
    REL_POS = 9,       //!< Joint DOF value
    JSPR_ANG = 10,     //!< Joint spring angle
    JSPR_DEFL = 11,    //!< Joint spring deflection
    JSPR_FORCE = 12,   //!< Joint spring force
    JDAMP_ANG = 13,    //!< Joint damper angle
    JDAMP_VEL = 14,    //!< Joint damper velocity
    JDAMP_FORCE = 15,  //!< Joint damper force
    LENGTH = 16,       //!< Axial spring and damper length
    DEFL = 17,         //!< Axial spring and damper deflection
    FORCE = 18,        //!< Axial spring and damper force
    LOCAL_FORCE = 19,  //!< Force in local coordinates
    GLOBAL_FORCE = 20, //!< Force in global coordinates
    WIND_SPEED = 21,   //!< Wind speed
    FLUID_VEL = 22,    //!< Fluid particle velocity
    FLUID_ACC = 23,    //!< Fluid particle acceleration
    DYN_PRESS = 24,    //!< Dynamic pressure from fluid
    STRAIN = 25,       //!< Gage strain
    STRESS = 26,       //!< Gage stress
    // For array dimensioning
    NUM_ENTITY = 27    //!< Total number of sensor entity types
  };

protected:
  //! \brief Default constructor.
  FmIsMeasuredBase();
  //! \brief The destructor deletes all sensors referring to this object.
  virtual ~FmIsMeasuredBase();

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

  //! List of all DOFs that can be measured, with associated name
  static std::array<FmSensorChoice,NUM_DOF>    itsDofTable;
  //! List of all response entities that can be measured, with associated name
  static std::array<FmSensorChoice,NUM_ENTITY> itsEntityTable;
};

#endif
