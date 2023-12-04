// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_SMOOTH_TRAJ_H
#define FMF_SMOOTH_TRAJ_H

#include "vpmDB/FmMathFuncBase.H"


class FmfSmoothTraj : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfSmoothTraj();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Smooth trajectory"; }
  virtual const char* getFunctionFsiName() const { return "SMOOTH_TRAJ"; }
  virtual const char** getPixmap() const;

  virtual void getFunctionVariables(std::vector<FmFuncVariable>& retArray,
				    bool fortranPermutation = false) const;

protected:
  virtual ~FmfSmoothTraj() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(StartTime,FmfSmoothTraj);
  M_MOD_PARAMS(TotalTrajTime,FmfSmoothTraj);
  M_MOD_PARAMS(MaxAcc,FmfSmoothTraj);
  M_MOD_PARAMS(MaxSpeed,FmfSmoothTraj);
};

#endif
