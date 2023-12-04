// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_FRICTION_BASE_H
#define FM_FRICTION_BASE_H

#include "vpmDB/FmParamObjectBase.H"


class FmFrictionBase : public FmParamObjectBase
{
  Fmd_DB_HEADER_INIT();

public:
  double getStickStiffness() const; // beta feature

  virtual void getParameters(std::vector<FmParameter>& retArray) const;

  virtual int printSolverEntry(FILE* fp);

protected:
  FmFrictionBase();
  virtual ~FmFrictionBase();

  virtual const char* getFrictionFsiName() const = 0;
  virtual void getTypeDepVars(std::vector<double>& vars) const { vars.clear(); }

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmFrictionBase* obj);

  M_MOD_PARAMS(PrestressLoad,FmFrictionBase);
  M_MOD_PARAMS(CoulombCoeff,FmFrictionBase);
  M_MOD_PARAMS(StribeckMagn,FmFrictionBase);
  M_MOD_PARAMS(StribeckSpeed,FmFrictionBase);

/* Not used since R2.1.2
  M_MOD_PARAMS(ViscCoeff,FmFrictionBase);
  M_MOD_PARAMS(AsymmetrMagn,FmFrictionBase);
  M_MOD_PARAMS(FricAmpl,FmFrictionBase);
  M_MOD_PARAMS(Freq,FmFrictionBase);
  M_MOD_PARAMS(PhaseAngle,FmFrictionBase);
*/
};

#endif
