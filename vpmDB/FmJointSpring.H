// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_JOINT_SPRING_H
#define FM_JOINT_SPRING_H

#include "vpmDB/FmSpringBase.H"

class FmJointBase;


class FmJointSpring : public FmSpringBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmJointSpring();

  virtual bool disconnect();

  FmJointBase* getOwnerJoint() const;

  virtual FmModelMemberBase* getActiveOwner() const;

  virtual bool isListable() const { return false; }
  virtual bool isMeasured() const;
  virtual int getDOF() const;
  virtual double getModelSpringLength() const;

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Joint spring"; }

  static int checkJointSprings();

protected:
  virtual ~FmJointSpring();

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;
};

#endif
