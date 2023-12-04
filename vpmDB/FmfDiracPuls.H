// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_DIRAC_PULS_H
#define FMF_DIRAC_PULS_H

#include "vpmDB/FmMathFuncBase.H"


class FmfDiracPuls : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfDiracPuls();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Pulse"; }
  virtual const char* getFunctionFsiName() const { return "DIRAC_PULS"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfDiracPuls() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(AmplitudeDisplacement,FmfDiracPuls);
  M_MOD_PARAMS(PulseAmplitude,FmfDiracPuls);
  M_MOD_PARAMS(PulseWidth,FmfDiracPuls);
  M_MOD_PARAMS(Delay,FmfDiracPuls);
};

#endif
