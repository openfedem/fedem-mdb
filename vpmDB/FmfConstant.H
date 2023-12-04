// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_CONSTANT_H
#define FMF_CONSTANT_H

#include "vpmDB/FmMathFuncBase.H"


class FmfConstant : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfConstant(double defConst = 0.0);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Constant"; }
  virtual const char* getFunctionFsiName() const { return "CONSTANT"; }
  virtual const char** getPixmap() const;

  virtual int getExtrapolationType() const;
  virtual unsigned int getNoArgs() const { return 0; }

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfConstant() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(Constant,FmfConstant);
};

#endif
