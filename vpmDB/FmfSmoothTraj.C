// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfSmoothTraj.H"
#include "vpmDB/FuncPixmaps/smooth.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSMOOTH_TRAJ, FmfSmoothTraj, FmMathFuncBase);

FmfSmoothTraj::FmfSmoothTraj()
{
  Fmd_CONSTRUCTOR_INIT(FmfSmoothTraj);

  FFA_FIELD_INIT(myStartTime,     0.0, "START_TIME");
  FFA_FIELD_INIT(myTotalTrajTime, 5.0, "TOTAL_TIME");
  FFA_FIELD_INIT(myMaxAcc,        1.0, "MAX_ACC");
  FFA_FIELD_INIT(myMaxSpeed,      1.0, "MAX_SPEED");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfSmoothTraj::getPixmap() const
{
  return smooth;
}


void FmfSmoothTraj::getFunctionVariables(std::vector<FmFuncVariable>& retArray,
					 bool fortranPermuted) const
{
  M_APPEND_PARAMS("Start",StartTime,FmfSmoothTraj,retArray);
  M_APPEND_PARAMS("Length",TotalTrajTime,FmfSmoothTraj,retArray);
  if (fortranPermuted)
  {
    M_APPEND_PARAMS("Max(f'')",MaxAcc,FmfSmoothTraj,retArray);
    M_APPEND_PARAMS("Max(f')",MaxSpeed,FmfSmoothTraj,retArray);
  }
  else
  {
    M_APPEND_PARAMS("Max(f')",MaxSpeed,FmfSmoothTraj,retArray);
    M_APPEND_PARAMS("Max(f'')",MaxAcc,FmfSmoothTraj,retArray);
  }
}


std::ostream& FmfSmoothTraj::writeFMF(std::ostream& os)
{
  os <<"FUNC_SMOOTH_TRAJ\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfSmoothTraj::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 4\n");
  fprintf(fp,"  realData =");
  fprintf(fp," %14.6e",   myStartTime.getValue());
  fprintf(fp," %14.6e",   myTotalTrajTime.getValue());
  fprintf(fp," %14.6e",   myMaxAcc.getValue());
  fprintf(fp," %14.6e\n", myMaxSpeed.getValue());
  return 0;
}


bool FmfSmoothTraj::readAndConnect(std::istream& is, std::ostream&)
{
  FmfSmoothTraj* obj = new FmfSmoothTraj();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmMathFuncBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmfSmoothTraj::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfSmoothTraj::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfSmoothTraj::getClassTypeID());
}
