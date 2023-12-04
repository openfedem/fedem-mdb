// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_MATH_EXPR_H
#define FMF_MATH_EXPR_H

#include "vpmDB/FmfMultiArgBase.H"


class FmfMathExpr : public FmfMultiArgBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfMathExpr(const char* expr = NULL);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Math expression"; }

  virtual bool initGetValueNoRecursion();
  virtual double getValue(const std::vector<double>& x, int& ierr) const;

  virtual unsigned int getNoArgs() const { return numArg.getValue(); }
  virtual void setNoArgs(unsigned int nArg) { numArg.setValue(nArg); }

  const std::string& getExpressionString() const { return expression.getValue(); }
  void setExpressionString(const std::string& expr) { expression.setValue(expr); }

protected:
  virtual ~FmfMathExpr() {}

  virtual int printSolverData(FILE* fp);

  virtual double getValueNoRecursion(double x, int& ierr) const;

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<std::string>  expression;
  FFaField<unsigned int> numArg;
};

#endif
