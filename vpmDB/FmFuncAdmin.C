// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <map>

#include "FFaFunctionLib/FFaUserFuncPlugin.H"

#include "vpmDB/FmAllFunctionHeaders.H"
#include "vpmDB/FmFuncAdmin.H"


namespace
{
  std::map<int,FmFuncTypeInfo> itsFuncInfoTable;

  int numClassTypes = 0;

  void initFuncInfoTable()
  {
    using namespace FmFuncAdmin;

    itsFuncInfoTable[NONE] = "   1:1";

    itsFuncInfoTable[GENERAL_HEADING] = "-- General Functions --";

    itsFuncInfoTable[LIN_VAR]         = FmFuncTypeInfo("    Poly line", FmfLinVar::getClassTypeID());
    itsFuncInfoTable[DEVICE]          = FmFuncTypeInfo("    Poly line from file", FmfDeviceFunction::getClassTypeID());
    itsFuncInfoTable[SPLINE]          = FmFuncTypeInfo("    Spline", FmfSpline::getClassTypeID());
    itsFuncInfoTable[MATH_EXPRESSION] = FmFuncTypeInfo("    Math expression", FmfMathExpr::getClassTypeID());

    itsFuncInfoTable[SIMPLE_HEADING] = "-- Simple Functions --";

    itsFuncInfoTable[CONSTANT]   = FmFuncTypeInfo("    Constant", FmfConstant::getClassTypeID());
    itsFuncInfoTable[SCALE]      = FmFuncTypeInfo("    Linear", FmfScale::getClassTypeID());
    itsFuncInfoTable[RAMP]       = FmFuncTypeInfo("    Ramp", FmfRamp::getClassTypeID());
    itsFuncInfoTable[LIM_RAMP]   = FmFuncTypeInfo("    Limited ramp", FmfLimRamp::getClassTypeID());
    itsFuncInfoTable[STEP]       = FmFuncTypeInfo("    Step", FmfStep::getClassTypeID());
    itsFuncInfoTable[DIRAC_PULS] = FmFuncTypeInfo("    Pulse", FmfDiracPuls::getClassTypeID());

    itsFuncInfoTable[PERIODIC_HEADING] = "-- Periodic Functions --";

    itsFuncInfoTable[SINUSOIDAL]          = FmFuncTypeInfo("    Sine", FmfSinusoidal::getClassTypeID());
    itsFuncInfoTable[COMPL_SINUS]         = FmFuncTypeInfo("    Combined sine", FmfComplSinus::getClassTypeID());
    itsFuncInfoTable[DELAYED_COMPL_SINUS] = FmFuncTypeInfo("    Delayed combined sine", FmfDelayedComplSinus::getClassTypeID());
    itsFuncInfoTable[WAVE_SINUS]          = FmFuncTypeInfo("    Wave sine", FmfWaveSinus::getClassTypeID());
    itsFuncInfoTable[WAVE_SPECTRUM]       = FmFuncTypeInfo("    JONSWAP sea wave spectrum", FmfWaveSpectrum::getClassTypeID());
    itsFuncInfoTable[FILE_SPECTRUM]       = FmFuncTypeInfo("    User defined wave spectrum", FmfDeviceFunction::getClassTypeID());
    itsFuncInfoTable[SQUARE_PULS]         = FmFuncTypeInfo("    Periodic square pulse", FmfSquarePuls::getClassTypeID());

    itsFuncInfoTable[SPECIAL_HEADING] = "-- Special Functions --";

    itsFuncInfoTable[SMOOTH_TRAJ] = FmFuncTypeInfo("    Smooth trajectory", FmfSmoothTraj::getClassTypeID());
    itsFuncInfoTable[LIN_VEL_VAR] = FmFuncTypeInfo("    Linear derivative", FmfLinVelVar::getClassTypeID());
    itsFuncInfoTable[EXTERNAL]    = FmFuncTypeInfo("    External function", FmfExternalFunction::getClassTypeID());
    itsFuncInfoTable[REFERENCE]   = FmFuncTypeInfo("    Refer to other function");

    const int maxUF = 400;
    int funcId[maxUF];
    int nUserFuncs = FFaUserFuncPlugin::instance()->getFuncs(maxUF,funcId);
    if (nUserFuncs > 0)
    {
      numClassTypes = FFaTypeCheck::getNewTypeID(NULL);

      std::string funcName(64,' ');
      char* fName = const_cast<char*>(funcName.c_str()+4);
      int funcIdx = USER_HEADING + 1;
      itsFuncInfoTable[USER_HEADING] = "-- User-defined Functions --";
      for (int i = 0; i < nUserFuncs; i++, funcIdx++)
        if (FFaUserFuncPlugin::instance()->getFuncName(funcId[i],60,fName) > 0)
          itsFuncInfoTable[funcIdx] = FmFuncTypeInfo(funcName.c_str(),
                                                     numClassTypes + funcId[i]);
    }

    for (std::pair<const int,FmFuncTypeInfo>& info : itsFuncInfoTable)
      info.second.funcMenuEnum = info.first;

    // Should not appear in the Function type menu
    itsFuncInfoTable[WAVE_SINUS].funcMenuEnum = INTERNAL;
  }
}


FmFuncTypeInfo::FmFuncTypeInfo(const char* fn, int ft)
{
  listName = fn ? fn : "(noname)";
  funcType = ft;
  funcMenuEnum = -1;
}


int FmFuncTypeInfo::getFuncType() const
{
  if (funcType > FFaTypeCheck::getNewTypeID(NULL))
    return FmfUserDefined::getClassTypeID();

  return funcType;
}


void FmFuncAdmin::clearInfoTable()
{
  itsFuncInfoTable.clear();
}


const std::vector<int>& FmFuncAdmin::getAllowableSprDmpFuncTypes()
{
  static std::vector<int> allowableFuncs = {
    FmfConstant::getClassTypeID(),
    FmfScale::getClassTypeID(),
    FmfRamp::getClassTypeID(),
    FmfLimRamp::getClassTypeID(),
    FmfLinVar::getClassTypeID(),
    FmfDeviceFunction::getClassTypeID()
  };

  return allowableFuncs;
}


bool FmFuncAdmin::hasSmartPoints(int type)
{
  if (type == FmfConstant::getClassTypeID() ||
      type == FmfScale::getClassTypeID() ||
      type == FmfRamp::getClassTypeID() ||
      type == FmfLimRamp::getClassTypeID() ||
      type == FmfStep::getClassTypeID() ||
      type == FmfDiracPuls::getClassTypeID() ||
      type == FmfLinVar::getClassTypeID() ||
      type == FmfDeviceFunction::getClassTypeID() ||
      type == FmfSinusoidal::getClassTypeID() ||
      type == FmfComplSinus::getClassTypeID() ||
      type == FmfDelayedComplSinus::getClassTypeID() ||
      type == FmfSquarePuls::getClassTypeID())
    return true;
  else
    return false;
}


void FmFuncAdmin::getCompatibleFunctionTypes(std::vector<FmFuncTypeInfo>& types,
                                             FmMathFuncBase* func)
{
  if (itsFuncInfoTable.empty())
    initFuncInfoTable();

  using FmFuncInfo = std::pair<const int,FmFuncTypeInfo>;

  switch (func ? func->getFunctionUse() : FmMathFuncBase::GENERAL)
    {
    case FmMathFuncBase::GENERAL:
      // General function, allow all function types,
      // except for internal ones and wave spectrums
      if (func && func->getTypeID() == FmfWaveSinus::getClassTypeID())
        // Internal function with predefined type, don't allow type switching
        types.push_back(itsFuncInfoTable[FmFuncAdmin::WAVE_SINUS]);
      else
        for (const FmFuncInfo& info : itsFuncInfoTable)
          if (info.second.funcMenuEnum > FmFuncAdmin::UNDEFINED &&
              info.second.funcMenuEnum != FmFuncAdmin::WAVE_SPECTRUM &&
              info.second.funcMenuEnum != FmFuncAdmin::FILE_SPECTRUM)
            types.push_back(info.second);
      return;

    case FmMathFuncBase::DRIVE_FILE:
      types.push_back(itsFuncInfoTable[FmFuncAdmin::DEVICE]);
      return;

    case FmMathFuncBase::NONE:
    case FmMathFuncBase::ROAD_FUNCTION:
    case FmMathFuncBase::CURR_FUNCTION:
      for (const FmFuncInfo& info : itsFuncInfoTable)
        if (info.second.funcMenuEnum  > FmFuncAdmin::NONE &&
            info.second.funcMenuEnum != FmFuncAdmin::WAVE_SPECTRUM &&
            info.second.funcMenuEnum != FmFuncAdmin::FILE_SPECTRUM &&
            info.second.funcMenuEnum != FmFuncAdmin::REFERENCE)
          types.push_back(info.second);
      return;

    case FmMathFuncBase::WAVE_FUNCTION:
      types.push_back(itsFuncInfoTable[FmFuncAdmin::SINUSOIDAL]);
      types.push_back(itsFuncInfoTable[FmFuncAdmin::WAVE_SPECTRUM]);
      types.push_back(itsFuncInfoTable[FmFuncAdmin::FILE_SPECTRUM]);
      // Check if we have user-defined wave functions
      for (const FmFuncInfo& info : itsFuncInfoTable)
        if (info.first > FmFuncAdmin::USER_HEADING &&
            info.second.funcType > numClassTypes)
          if (int fId = info.second.funcType - numClassTypes;
              FFaUserFuncPlugin::instance()->getFlag(fId) & 4)
            types.push_back(info.second);
      return;

    default: // Stiffness or Damper function
      break;
    }

    const std::vector<int>& ftyp = FmFuncAdmin::getAllowableSprDmpFuncTypes();
    for (const FmFuncInfo& info : itsFuncInfoTable)
      if (std::find(ftyp.begin(),ftyp.end(),info.second.funcType) != ftyp.end())
        if (info.first != FmFuncAdmin::FILE_SPECTRUM)
          types.push_back(info.second);
}


FmMathFuncBase* FmFuncAdmin::createFunction(int type)
{
#define CREATE_FUNC(T) if (type == T::getClassTypeID()) return new T()

  CREATE_FUNC(FmfLinVar);
  CREATE_FUNC(FmfConstant);
  CREATE_FUNC(FmfSinusoidal);
  CREATE_FUNC(FmfComplSinus);
  CREATE_FUNC(FmfDelayedComplSinus);
  CREATE_FUNC(FmfStep);
  CREATE_FUNC(FmfScale);
  CREATE_FUNC(FmfSpline);
  CREATE_FUNC(FmfRamp);
  CREATE_FUNC(FmfSquarePuls);
  CREATE_FUNC(FmfDiracPuls);
  CREATE_FUNC(FmfLimRamp);
  CREATE_FUNC(FmfSmoothTraj);
  CREATE_FUNC(FmfLinVelVar);
  CREATE_FUNC(FmfDeviceFunction);
  CREATE_FUNC(FmfExternalFunction);
  CREATE_FUNC(FmfMathExpr);
  CREATE_FUNC(FmfWaveSinus);
  CREATE_FUNC(FmfWaveSpectrum);
  CREATE_FUNC(FmfUserDefined);

  return NULL;
}
