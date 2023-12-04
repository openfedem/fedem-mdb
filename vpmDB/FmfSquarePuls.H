// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_SQUARE_PULS_H
#define FMF_SQUARE_PULS_H

#include "vpmDB/FmMathFuncBase.H"


class FmfSquarePuls : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfSquarePuls();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Periodic square pulse"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfSquarePuls() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(AmplitudeDisplacement,FmfSquarePuls);
  M_MOD_PARAMS(Amplitude,FmfSquarePuls);
  M_MOD_PARAMS(Period,FmfSquarePuls);
  M_MOD_PARAMS(PhaseAngle,FmfSquarePuls);
};

#endif
