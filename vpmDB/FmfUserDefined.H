// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_USER_DEFINED_H
#define FMF_USER_DEFINED_H

#include "vpmDB/FmfMultiArgBase.H"


class FmfUserDefined : public FmfMultiArgBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfUserDefined();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "User-defined function"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

  virtual bool isSurfaceFunc() const;
  virtual bool initGetValueNoRecursion();
  virtual double getValue(const DoubleVec& x, int& ierr) const;
  virtual unsigned int getNoArgs() const;

  virtual void initAfterParse() { this->setFuncId(myFuncId.getValue()); }

  using FmfMultiArgBase::getData;
  virtual const DoubleVec& getData() const { return myParameters.getValue(); }

  bool setFuncId(int fId);
  int  getFuncId() const { return myFuncId.getValue(); }

  bool setFuncNo(int fNo);
  int  getFuncNo() const { return myFuncNo.getValue(); }

  void   setPar(int iPar, double);
  double getPar(int iPar) const;

protected:
  virtual ~FmfUserDefined() {}

  virtual int printSolverData(FILE* fp);

  virtual double getValueNoRecursion(double x, int& ierr) const;

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<int>        myFuncId;
  FFaNoPrintField<int> myFuncNo;
  FFaField<DoubleVec>  myParameters;
};

#endif
