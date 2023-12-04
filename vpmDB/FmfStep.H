// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_STEP_H
#define FMF_STEP_H

#include "vpmDB/FmMathFuncBase.H"


class FmfStep : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfStep();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Step"; }
  virtual const char* getFunctionFsiName() const { return "STEP"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfStep() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(AmplitudeDisplacement,FmfStep);
  M_MOD_PARAMS(AmplitudeStep,FmfStep);
  M_MOD_PARAMS(DelayStep,FmfStep);
};

#endif
