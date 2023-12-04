// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_SPLINE_H
#define FMF_SPLINE_H

#include "vpmDB/FmfMultiVarBase.H"


class FmfSpline : public FmfMultiVarBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfSpline();

  virtual void addXYset(double X, double Y);
  virtual bool removeXYset(long place);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Spline"; }
  virtual const char* getFunctionFsiName() const { return "SPLINE"; }
  virtual const char** getPixmap() const;

  virtual void getXAxisDomain(double& start, double& stop) const;

  static void setAllSplineICODE(bool verbose = false);
  virtual int getBlockSize() const;

  static int checkSplines();

protected:
  virtual ~FmfSpline() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  int itsSplineICODE;

  void setSplineICODE(int code);
};


struct splineTable
{
  FmfSpline* object;
  long int   size;
  int        ICODE;

  splineTable(FmfSpline* s = NULL, long int n = 0, int C = 0)
    : object(s), size(n), ICODE(C) {}
};

#endif
