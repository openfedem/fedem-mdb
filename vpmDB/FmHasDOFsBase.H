// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_HAS_DOFS_BASE_H
#define FM_HAS_DOFS_BASE_H

#include "vpmDB/FmIsPositionedBase.H"
#include "vpmDB/FmDofLoad.H"
#include "vpmDB/FmDofMotion.H"


class FmHasDOFsBase : public FmIsPositionedBase
{
  Fmd_DB_HEADER_INIT();

public:

  // Load and Prescribed Motion management

  bool setLoadAtDOF  (int dof, FmDofLoad* load, bool forceReplace = false);
  bool setMotionAtDOF(int dof, FmDofMotion* pm, bool forceReplace = false);

  FmDofLoad* getLoadAtDOF(int dof, bool createIfNone = false);
  virtual FmDofMotion* getMotionAtDOF(int dof, bool createIfNone = false);

  int getLoadBaseID  (int dof) const;
  int getMotionBaseID(int dof) const;

  void releaseLoadAtDOF  (int dof);
  void releaseMotionAtDOF(int dof);

  int atWhatDOF(const FmDofLoad* load) const;
  int atWhatDOF(const FmDofMotion* pm) const;

  virtual void initAfterResolve();

  // DOF management

  enum DOFType {
    X_TRANS = 0,
    Y_TRANS = 1,
    Z_TRANS = 2,
    X_ROT   = 3,
    Y_ROT   = 4,
    Z_ROT   = 5,
    MAX_DOF = 6
  };

  enum DOFStatus {
    FREE,
    FIXED,
    PRESCRIBED,
    FREE_DYNAMICS,
    SPRING_CONSTRAINED,
    SPRING_DYNAMICS
  };

  FFaEnumMapping(DOFStatus) {
    FFaEnumEntry( FREE,               "FREE");
    FFaEnumEntry( FIXED,              "FIXED");
    FFaEnumEntry( PRESCRIBED,         "PRESCRIBED");
    FFaEnumEntry( FREE_DYNAMICS,      "FREE_DYNAMICS");
    FFaEnumEntry( SPRING_CONSTRAINED, "SPRING_CONSTRAINED");
    FFaEnumEntry( SPRING_DYNAMICS,    "SPRING_CONSTR_DYN");
    FFaEnumEntryEnd;
  };

  friend std::ostream& operator<<(std::ostream& s, const std::vector<DOFStatusEnum>& obj);
  friend std::istream& operator>>(std::istream& s, std::vector<DOFStatusEnum>& obj);

  void getDOFs(IntVec& dofs) const;

  bool setStatusForAllDOFs(bool fixed);
  bool setStatusForDOF(int dof, int status);
  int getStatusCode(int dof) const; // 0=fixed, 1=free, 2=additional BC

  virtual bool setStatusForDOF(int dof, DOFStatus status) = 0;
  virtual DOFStatus getStatusOfDOF(int dof) const = 0;
  virtual bool isLegalDOF(int dof) const = 0;
  virtual bool hasConstraints(bool fixedOnly = false) const = 0;
  virtual bool isSuppressed() const = 0;

  virtual void setInitVel(int dof, double vel) = 0;
  double getInitVel(int dof, bool includeFixed = false) const;
  double getInitAcc(int dof, bool includeFixed = false) const;

  virtual void updateChildrenDisplayTopology();

protected:
  FmHasDOFsBase();
  virtual ~FmHasDOFsBase();

  void completeInitDOFs();
  void completeInitDOF(int i, const std::string& dofName);

  static bool localParse(const char* keyWord, std::istream& activeStatement,
			 FmHasDOFsBase* obj);

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReference<FmDofLoad>   myLoads[MAX_DOF];
  FFaReference<FmDofMotion> myMotions[MAX_DOF];

  FFaField<FFaReferenceBase*> myLoadFields[MAX_DOF];
  FFaField<FFaReferenceBase*> myMotionFields[MAX_DOF];

  FFaField<DoubleVec> initVel;
  FFaField<DoubleVec> initAcc;
};

#endif
