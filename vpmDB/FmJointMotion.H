// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_JOINT_MOTION_H
#define FM_JOINT_MOTION_H

#include "vpmDB/FmDofMotion.H"

class FmJointBase;


class FmJointMotion : public FmDofMotion
{
  Fmd_DB_HEADER_INIT();

public:
  FmJointMotion();

  virtual void initAfterResolve();

  FmJointBase* getOwnerJoint() const;

  bool getInitLengthOrDefl(double& lenOrDefl) const;
  void setInitLengthOrDefl(double lenOrDefl, bool isDeflection = false);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Joint motion"; }

protected:
  virtual ~FmJointMotion() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaField<bool> useInitDeflection;
};

#endif
