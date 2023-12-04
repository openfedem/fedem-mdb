// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_SINUSOIDAL_H
#define FMF_SINUSOIDAL_H

#include "vpmDB/FmMathFuncBase.H"


class FmfSinusoidal : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfSinusoidal();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const;
  virtual const char* getFunctionFsiName() const;
  virtual const char** getPixmap() const;

  virtual bool hasSmartPoints() const;
  virtual bool initGetValue();

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
                                    bool waveFuncPermutation) const;

protected:
  virtual ~FmfSinusoidal() {}

  virtual void changedEvent();

  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual int printSolverData(FILE* fp);

  M_MOD_PARAMS(Frequency,FmfSinusoidal);
  M_MOD_PARAMS(PeriodDelay,FmfSinusoidal);
  M_MOD_PARAMS(Amplitude,FmfSinusoidal);
  M_MOD_PARAMS(AmplitudeDisplacement,FmfSinusoidal);
  M_MOD_PARAMS(MaxTime,FmfSinusoidal);
};

#endif
