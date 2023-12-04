// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_EXTERNAL_FUNCTION_H
#define FMF_EXTERNAL_FUNCTION_H

#include "vpmDB/FmMathFuncBase.H"


class FmfExternalFunction : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfExternalFunction();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "External function"; }
  virtual const char* getFunctionFsiName() const { return "DEVICE_FUNCTION"; }

  virtual int getExtrapolationType() const;
  virtual unsigned int getNoArgs() const { return 0; }

  FFaField<int>    channel;
  FFaField<double> scale;
  FFaField<double> shift;

protected:
  virtual ~FmfExternalFunction() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
