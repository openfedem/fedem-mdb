// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_REV_JOINT_H
#define FM_REV_JOINT_H

#include "vpmDB/FmSMJointBase.H"


class FmRevJoint : public FmSMJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRevJoint();

  virtual int getValidFrictionType() const;
  virtual int getFrictionDof() const { return 5; }

  virtual FmDofMotion* getMotionAtDOF(int dof, bool createIfNone);

  void setHasTzDOF(bool yesOrNo);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Revolute joint"; }

protected:
  virtual ~FmRevJoint() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<bool> hasTzDOF;
};

#endif
