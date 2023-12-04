// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PRISM_JOINT_H
#define FM_PRISM_JOINT_H

#include "vpmDB/FmMMJointBase.H"


class FmPrismJoint : public FmMMJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmPrismJoint();

  virtual int getValidFrictionType() const;
  virtual int getFrictionDof() const { return 6; }

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Prismatic joint"; }

protected:
  virtual ~FmPrismJoint() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
