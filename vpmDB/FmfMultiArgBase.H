// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_MULTIARG_BASE_H
#define FMF_MULTIARG_BASE_H

#include "vpmDB/FmMathFuncBase.H"


class FmfMultiArgBase : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual bool initGetValueNoRecursion() = 0;
  virtual bool initGetValue();

  virtual double getValue(double x, int& ierr) const;
  virtual double getValue(const std::vector<double>& x, int& ierr) const
  { return this->FmMathFuncBase::getValue(x,ierr); }

protected:
  FmfMultiArgBase();
  virtual ~FmfMultiArgBase() {}

  virtual double getValueNoRecursion(double x, int& ierr) const = 0;

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
