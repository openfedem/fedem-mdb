// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_JOINT_BASE_H
#define FM_JOINT_BASE_H

#include "vpmDB/FmHasDOFsBase.H"
#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmFrictionBase.H"
#include "vpmDB/FmTriad.H"

class FmHPBase;
class FmJointMotion;


class FmJointBase : public FmHasDOFsBase
{
  Fmd_DB_HEADER_INIT();

public:

  // Spring & Damper management

  bool setDamperAtDOF(int DOFno, FmJointDamper* dmp,
		      bool forceReplace = false);
  bool setSpringAtDOF(int DOFno, FmJointSpring* spr,
		      bool forceReplace = false);

  FmJointDamper* getDamperAtDOF(int DOFno, bool createIfNone = false);
  FmJointSpring* getSpringAtDOF(int DOFno, bool createIfNone = false);

  int getDamperBaseID(int DOFno) const;
  int getSpringBaseID(int DOFno) const;

  void removeDamperAtDOF(int DOFno);
  void removeSpringAtDOF(int DOFno);

  void releaseDamperAtDOF(int DOFno);
  void releaseSpringAtDOF(int DOFno);

  int atWhatDOF(const FmJointDamper* dmp) const;
  int atWhatDOF(const FmJointSpring* spr) const;
  int atWhatDOF(const FmJointMotion* pm) const;

  virtual FmDofMotion* getMotionAtDOF(int DOFno, bool createIfNone = false);

  // Friction management

  virtual int getValidFrictionType(int) const { return getValidFrictionType(); }
  virtual int getValidFrictionType() const { return 0; }

  FmFrictionBase* getFriction() const { return myFriction.getPointer(); }
  void setFriction(FmFrictionBase* fric) { myFriction.setRef(fric); }

  virtual int getFrictionDof() const { return -1; }
  virtual bool setFrictionDof(int) { return true; }

  // Convenience functions used by FmSolverParser

  bool isAxialJoint(bool useLocalDofsOnly = false) const;
  bool isContactElement() const;
  bool isGlobalSpringElement() const;
  bool isMasterSlaveInOtherJoint() const;
  virtual bool isSuppressed() const;

  // Computation and setting of joint variables

  double getJointVariable(int dof) const;

  virtual FaVec3 getTransJointVariables() const = 0;
  virtual FaVec3 getRotJointVariables() const = 0;
  virtual void setRotJointVariables(const FaVec3& rotations) = 0;

  // Management of DOF's

  virtual bool isLegalDOF(int DOFno) const;
  int getLegalDOFCount() const;
  int getJointVariableNumber(int dof) const;

  // SmartMove dof
#ifdef USE_INVENTOR
  virtual FaDOF getObjDegOfFreedom() const;
#endif

  // Topology management

  void eraseKeepDOFs();
  void eraseInternal();

  bool setAsSlaveTriad(FmTriad* slTriad);
  bool removeItsSlaveTriad();
  FmTriad* getSlaveTriad() const;
  bool isSlaveTriad(const FmTriad* triad) const;

  FmLink* getOtherLink(const FmLink* thisLink) const;
  FmLink* getOtherLink(const FmTriad* jointTriad) const;
  bool isAttachedToLink(const FmLink* thisLink = NULL) const;
  bool isSlaveAttachedToLink(bool allowMultipleLinks = false) const;
  FmLink* getSlaveLink() const;

  // Returns true if this joint is used in a higher pair (as master or slave)
  bool hasHPConnections() const;
  // Returns the higher pair for which this joint is the slave, if any
  FmHPBase* getHPConnection() const;

  virtual void updateChildrenDisplayTopology();

  virtual void getMasterTriads(std::vector<FmTriad*>& triads) const = 0;
  virtual bool isMasterTriad(const FmTriad* triad) const = 0;

  virtual bool isMasterAttachedToLink(bool allowMultiple = false) const = 0;
  virtual FmLink* getMasterLink() const = 0;
  virtual FmPart* getMasterPart(bool noEarth = false) const = 0;

  virtual void initAfterResolve();

  // FmIsMeasuredBase reimplementation

  virtual void getEntities(std::vector<FmSensorChoice>& choices, int dof = -1);
  virtual void getDofs(std::vector<FmSensorChoice>& choices);

  // Rotation formulation handling

  enum RotFormulationType{ FOLLOWER_AXIS, ORTHOGONAL_AXIS, ROT_AXIS };
  FFaEnumMapping(RotFormulationType) {
    FFaEnumEntry( FOLLOWER_AXIS,   "FOLLOWER_AXIS");
    FFaEnumEntry( ORTHOGONAL_AXIS, "ORTHOGONAL_AXIS");
    FFaEnumEntry( ROT_AXIS,        "ROT_AXIS");
    FFaEnumEntryEnd;
  };
  static const Strings& getRotFormulationUINames();

  FFaField<RotFormulationTypeEnum> rotFormulation;

  // Rotation sequence handling

  enum RotSequenceType{ rZYX, rYXZ, rXZY, rXYZ, rYZX, rZXY };
  FFaEnumMapping(RotSequenceType) {
    FFaEnumEntry( rZYX, "ZYX");
    FFaEnumEntry( rYXZ, "YXZ");
    FFaEnumEntry( rXZY, "XZY");
    FFaEnumEntry( rXYZ, "XYZ");
    FFaEnumEntry( rYZX, "YZX");
    FFaEnumEntry( rZXY, "ZXY");
    FFaEnumEntryEnd;
  };
  static const Strings& getRotSequenceUINames();
  static const char* getRotExplain(int rotFormulation, int rotSequence);

  FFaField<RotSequenceTypeEnum> rotSequence;

  // Spring coupling handling

  enum SpringCplType{ NONE, XY, YZ, ZX, XYZ };
  FFaEnumMapping(SpringCplType) {
    FFaEnumEntry( NONE,"NONE");
    FFaEnumEntry( XY,  "XY");
    FFaEnumEntry( YZ,  "YZ");
    FFaEnumEntry( ZX,  "ZX");
    FFaEnumEntry( XYZ, "XYZ");
    FFaEnumEntryEnd;
  };
  static const Strings& getSpringCplUINames();

  FFaField<SpringCplTypeEnum> tranSpringCpl;
  FFaField<SpringCplTypeEnum> rotSpringCpl;

  // DOF status handling

  virtual bool setStatusForDOF(int dof, DOFStatus status);
  virtual DOFStatus getStatusOfDOF(int dof) const;
  virtual bool hasConstraints(bool fixedOnly = false) const;

  static int checkJoints();

  // Initial condition handling

  void setInitVel(int dof, double vel);
  void setInitAcc(int dof, double acc);

protected:
  FmJointBase();
  virtual ~FmJointBase();

  void completeInitJVars();

  static bool localParse(const char* keyWord, std::istream& activeStatement,
			 FmJointBase* obj);

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;

  FaVec3 getJointRotations(const FaMat34& from, const FaMat34& to) const;
  void setJointRotations(const FaVec3& rotations, const FaMat34& masterCS);

  FFaReference<FmTriad>        itsSlaveTriad;
  FFaField<FFaReferenceBase*>  itsSlaveTriadField;

  FFaField<DOFStatusEnum>      myDofStatus[MAX_DOF];

  FFaReference<FmJointSpring>  mySprings[MAX_DOF];
  FFaReference<FmJointDamper>  myDampers[MAX_DOF];

  FFaField<FFaReferenceBase*>  mySpringFields[MAX_DOF];
  FFaField<FFaReferenceBase*>  myDamperFields[MAX_DOF];

  FFaReference<FmFrictionBase> myFriction;
  FFaField<FFaReferenceBase*>  myFrictionField;

  FFaField<IntVec>             myDOFQuadrant;

  bool myLegalDOFs[MAX_DOF];
};

#endif
