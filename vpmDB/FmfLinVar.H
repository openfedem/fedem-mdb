// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMFLINVAR_H
#define FMFLINVAR_H

#include "vpmDB/FmfMultiVarBase.H"


class FmfLinVar : public FmfMultiVarBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfLinVar();

  virtual void addXYset(double X, double Y);
  virtual bool removeXYset(long place);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual int getBlockSize() const;

  virtual const char* getFunctionUIName() const { return "Poly line"; }
  virtual const char* getFunctionFsiName() const { return "LIN_VAR"; }
  virtual const char** getPixmap() const;

protected:
  virtual ~FmfLinVar() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
