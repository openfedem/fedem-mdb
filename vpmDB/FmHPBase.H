// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_HP_BASE_H
#define FM_HP_BASE_H

#include "vpmDB/FmIsRenderedBase.H"
#include "vpmDB/FmJointBase.H"


class FmHPBase : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

public:
  bool setInputJoint(FmJointBase* jnt);
  virtual bool setOutputJoint(FmJointBase* jnt) = 0;

  FmJointBase* getInputJoint() const;
  virtual FmJointBase* getOutputJoint() const = 0;

  virtual int getInputJointDof() const = 0;
  virtual int getOutputJointDof() const = 0;

  bool   setTransmissionRatio(double ratio);
  double getTransmissionRatio() const;

  virtual int printSolverEntry(FILE* fp);

protected:
  FmHPBase();
  virtual ~FmHPBase();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmHPBase* obj);

private:
  FFaReference<FmJointBase>   itsInputJoint;
  FFaField<FFaReferenceBase*> itsInputJointField;
  FFaField<double>            itsHPRatio;
};

#endif
