// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_LIN_VEL_VAR_H
#define FMF_LIN_VEL_VAR_H

#include "vpmDB/FmfMultiVarBase.H"


class FmfLinVelVar : public FmfMultiVarBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfLinVelVar();

  void addIntervalBreak(double val);
  bool removeIntervalBreak(long int pos);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Linear derivative"; }
  virtual const char* getFunctionFsiName() const { return "LIN_VEL_VAR"; }
  virtual const char** getPixmap() const;

  virtual int getBlockSize() const;

  virtual void getXAxisDomain(double& start, double& stop) const;

protected:
  virtual ~FmfLinVelVar() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
