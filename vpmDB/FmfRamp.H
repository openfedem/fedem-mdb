// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_RAMP_H
#define FMF_RAMP_H

#include "vpmDB/FmMathFuncBase.H"


class FmfRamp : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfRamp();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Ramp"; }
  virtual const char* getFunctionFsiName() const { return "RAMP"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfRamp() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(AmplitudeDisplacement,FmfRamp);
  M_MOD_PARAMS(Slope,FmfRamp);
  M_MOD_PARAMS(Delay,FmfRamp);
};

#endif
