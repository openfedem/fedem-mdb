// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_COMPL_SINUS2_H
#define FMF_COMPL_SINUS2_H

#include "vpmDB/FmMathFuncBase.H"


class FmfDelayedComplSinus : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfDelayedComplSinus();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Delayed combined sine"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfDelayedComplSinus() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(FreqForWave1,FmfDelayedComplSinus);
  M_MOD_PARAMS(FreqForWave2,FmfDelayedComplSinus);
  M_MOD_PARAMS(PeriodDelayWave1,FmfDelayedComplSinus);
  M_MOD_PARAMS(PeriodDelayWave2,FmfDelayedComplSinus);
  M_MOD_PARAMS(AmplitudeWave1,FmfDelayedComplSinus);
  M_MOD_PARAMS(AmplitudeWave2,FmfDelayedComplSinus);
  M_MOD_PARAMS(AmplitudeDisplacement,FmfDelayedComplSinus);
  M_MOD_PARAMS(MinTime,FmfDelayedComplSinus);
};

#endif
