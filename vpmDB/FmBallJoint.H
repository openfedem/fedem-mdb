// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_BALL_JOINT_H
#define FM_BALL_JOINT_H

#include "vpmDB/FmSMJointBase.H"


class FmBallJoint : public FmSMJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmBallJoint();

  virtual int getValidFrictionType() const;

  virtual int getFrictionDof() const { return myFrictionDof.getValue(); }
  virtual bool setFrictionDof(int dof);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Ball joint"; }

protected:
  virtual ~FmBallJoint() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<int> myFrictionDof;
};

#endif
