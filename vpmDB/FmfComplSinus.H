// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_COMPL_SINUS1_H
#define FMF_COMPL_SINUS1_H

#include "vpmDB/FmMathFuncBase.H"


class FmfComplSinus : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfComplSinus();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Combined sine"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfComplSinus() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(FreqForWave1,FmfComplSinus);
  M_MOD_PARAMS(FreqForWave2,FmfComplSinus);
  M_MOD_PARAMS(PeriodDelayWave1,FmfComplSinus);
  M_MOD_PARAMS(PeriodDelayWave2,FmfComplSinus);
  M_MOD_PARAMS(AmplitudeWave1,FmfComplSinus);
  M_MOD_PARAMS(AmplitudeWave2,FmfComplSinus);
  M_MOD_PARAMS(AmplitudeDisplacement,FmfComplSinus);
  M_MOD_PARAMS(MaxTime,FmfComplSinus);
};

#endif
