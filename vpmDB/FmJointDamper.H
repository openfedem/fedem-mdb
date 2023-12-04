// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_JOINT_DAMPER_H
#define FM_JOINT_DAMPER_H

#include "vpmDB/FmDamperBase.H"

class FmJointBase;


class FmJointDamper : public FmDamperBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmJointDamper();

  virtual bool disconnect();

  FmJointBase* getOwnerJoint() const;

  virtual FmModelMemberBase* getActiveOwner() const;

  virtual bool isListable() const { return false; }
  virtual bool isMeasured() const;
  virtual int getDOF() const;

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fp);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Joint damper"; }

  static int checkJointDampers();

protected:
  virtual ~FmJointDamper();

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;
};

#endif
