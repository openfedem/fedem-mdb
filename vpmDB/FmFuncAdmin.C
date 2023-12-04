// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "FFaFunctionLib/FFaUserFuncPlugin.H"

#include "vpmDB/FmAllFunctionHeaders.H"
#include "vpmDB/FmFuncAdmin.H"

FuncInfoMap FmFuncAdmin::itsFuncInfoTable;
std::vector<int> FmFuncAdmin::allowableSprDmpFuncs;

static int numClassTypes = 0;


void FmFuncAdmin::init()
{
  itsFuncInfoTable[NONE]                = FmFuncTypeInfo("   1:1");

  itsFuncInfoTable[GENERAL_HEADING]     = FmFuncTypeInfo("-- General Functions --");

  itsFuncInfoTable[LIN_VAR]             = FmFuncTypeInfo("    Poly line", FmfLinVar::getClassTypeID());
  itsFuncInfoTable[DEVICE]              = FmFuncTypeInfo("    Poly line from file", FmfDeviceFunction::getClassTypeID());
  itsFuncInfoTable[SPLINE]              = FmFuncTypeInfo("    Spline", FmfSpline::getClassTypeID());
  itsFuncInfoTable[MATH_EXPRESSION]     = FmFuncTypeInfo("    Math expression", FmfMathExpr::getClassTypeID());

  itsFuncInfoTable[SIMPLE_HEADING]      = FmFuncTypeInfo("-- Simple Functions --");

  itsFuncInfoTable[CONSTANT]            = FmFuncTypeInfo("    Constant", FmfConstant::getClassTypeID());
  itsFuncInfoTable[SCALE]               = FmFuncTypeInfo("    Linear", FmfScale::getClassTypeID());
  itsFuncInfoTable[RAMP]                = FmFuncTypeInfo("    Ramp", FmfRamp::getClassTypeID());
  itsFuncInfoTable[LIM_RAMP]            = FmFuncTypeInfo("    Limited ramp", FmfLimRamp::getClassTypeID());
  itsFuncInfoTable[STEP]                = FmFuncTypeInfo("    Step", FmfStep::getClassTypeID());
  itsFuncInfoTable[DIRAC_PULS]          = FmFuncTypeInfo("    Pulse", FmfDiracPuls::getClassTypeID());

  itsFuncInfoTable[PERIODIC_HEADING]    = FmFuncTypeInfo("-- Periodic Functions --");

  itsFuncInfoTable[SINUSOIDAL]          = FmFuncTypeInfo("    Sine", FmfSinusoidal::getClassTypeID());
  itsFuncInfoTable[COMPL_SINUS]         = FmFuncTypeInfo("    Combined sine", FmfComplSinus::getClassTypeID());
  itsFuncInfoTable[DELAYED_COMPL_SINUS] = FmFuncTypeInfo("    Delayed combined sine", FmfDelayedComplSinus::getClassTypeID());
  itsFuncInfoTable[WAVE_SINUS]          = FmFuncTypeInfo("    Wave sine", FmfWaveSinus::getClassTypeID());
  itsFuncInfoTable[WAVE_SPECTRUM]       = FmFuncTypeInfo("    JONSWAP sea wave spectrum", FmfWaveSpectrum::getClassTypeID());
  itsFuncInfoTable[FILE_SPECTRUM]       = FmFuncTypeInfo("    User defined wave spectrum", FmfDeviceFunction::getClassTypeID());
  itsFuncInfoTable[SQUARE_PULS]         = FmFuncTypeInfo("    Periodic square pulse", FmfSquarePuls::getClassTypeID());

  itsFuncInfoTable[SPECIAL_HEADING]     = FmFuncTypeInfo("-- Special Functions --");

  itsFuncInfoTable[SMOOTH_TRAJ]         = FmFuncTypeInfo("    Smooth trajectory", FmfSmoothTraj::getClassTypeID());
  itsFuncInfoTable[LIN_VEL_VAR]         = FmFuncTypeInfo("    Linear derivative", FmfLinVelVar::getClassTypeID());
  itsFuncInfoTable[EXTERNAL]            = FmFuncTypeInfo("    External function", FmfExternalFunction::getClassTypeID());
  itsFuncInfoTable[REFERENCE]           = FmFuncTypeInfo("    Refer to other function");

  const int maxUF = 400;
  int funcId[maxUF];
  int nUserFuncs = FFaUserFuncPlugin::instance()->getFuncs(maxUF,funcId);
  if (nUserFuncs > 0)
  {
    numClassTypes = FFaTypeCheck::getNewTypeID(NULL);

    std::string funcName(64,' ');
    const char* fName = funcName.c_str()+4;
    itsFuncInfoTable[USER_HEADING] = FmFuncTypeInfo("-- User-defined Functions --");
    for (int i = 1; i <= nUserFuncs; i++)
      if (FFaUserFuncPlugin::instance()->getFuncName(funcId[i-1],60,const_cast<char*>(fName)) > 0)
	itsFuncInfoTable[USER_HEADING+i] = FmFuncTypeInfo(funcName,numClassTypes+funcId[i-1]);
  }

  for (FuncInfoMap::iterator it = itsFuncInfoTable.begin(); it != itsFuncInfoTable.end(); it++)
    it->second.funcMenuEnum = it->first;

  itsFuncInfoTable[WAVE_SINUS].funcMenuEnum = INTERNAL; // Should not appear in Function type menu
}


int FmFuncTypeInfo::getFuncType() const
{
  return funcType > FFaTypeCheck::getNewTypeID(NULL) ? FmfUserDefined::getClassTypeID() : funcType;
}


const std::vector<int>& FmFuncAdmin::getAllowableSprDmpFuncTypes()
{
  if (allowableSprDmpFuncs.empty()) {
    allowableSprDmpFuncs.push_back(FmfConstant::getClassTypeID());
    allowableSprDmpFuncs.push_back(FmfScale::getClassTypeID());
    allowableSprDmpFuncs.push_back(FmfRamp::getClassTypeID());
    allowableSprDmpFuncs.push_back(FmfLimRamp::getClassTypeID());
    allowableSprDmpFuncs.push_back(FmfLinVar::getClassTypeID());
    allowableSprDmpFuncs.push_back(FmfDeviceFunction::getClassTypeID());
  }

  return allowableSprDmpFuncs;
}


bool FmFuncAdmin::isAllowableSprDmpFuncType(int type)
{
  const std::vector<int>& ftypes = getAllowableSprDmpFuncTypes();
  return std::find(ftypes.begin(),ftypes.end(),type) != ftypes.end();
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


void FmFuncAdmin::getCompatibleFunctionTypes(std::vector<FmFuncTypeInfo>& toFill,
                                             FmMathFuncBase* compatibleFunc)
{
  if (itsFuncInfoTable.empty())
    FmFuncAdmin::init();

  toFill.clear();
  FuncInfoMap::const_iterator it;

  if (compatibleFunc)
    switch (compatibleFunc->getFunctionUse())
      {
      case FmMathFuncBase::GENERAL:
	if (compatibleFunc->getTypeID() == FmfWaveSinus::getClassTypeID()) {
	  // Internal function with predefined type, don't allow type switching
	  toFill.push_back(itsFuncInfoTable[FmFuncAdmin::WAVE_SINUS]);
	  return;
	}
	break;

      case FmMathFuncBase::DRIVE_FILE:
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::DEVICE]);
	return;

      case FmMathFuncBase::NONE:
      case FmMathFuncBase::ROAD_FUNCTION:
      case FmMathFuncBase::CURR_FUNCTION:
	for (it = itsFuncInfoTable.begin(); it != itsFuncInfoTable.end(); it++)
	  if (it->second.funcMenuEnum  > FmFuncAdmin::NONE &&
	      it->second.funcMenuEnum != FmFuncAdmin::WAVE_SPECTRUM &&
	      it->second.funcMenuEnum != FmFuncAdmin::FILE_SPECTRUM &&
	      it->second.funcMenuEnum != FmFuncAdmin::REFERENCE)
	    toFill.push_back(it->second);
	return;

      case FmMathFuncBase::WAVE_FUNCTION:
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::SINUSOIDAL]);
	/* Disabled by kmo 17.10.2012. Only to be consistent with documentation.
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::COMPL_SINUS]);
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::DELAYED_COMPL_SINUS]);
	*/
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::WAVE_SPECTRUM]);
	toFill.push_back(itsFuncInfoTable[FmFuncAdmin::FILE_SPECTRUM]);
	// Check if we have user-defined wave functions
	for (it = itsFuncInfoTable.begin(); it != itsFuncInfoTable.end(); it++)
	  if (it->first > FmFuncAdmin::USER_HEADING && it->second.funcType > numClassTypes)
	  {
	    int fId = it->second.funcType - numClassTypes;
	    if (FFaUserFuncPlugin::instance()->getFlag(fId) & 4)
	      toFill.push_back(it->second);
	  }
	return;

      default: // Stiffness or Damper function
	for (it = itsFuncInfoTable.begin(); it != itsFuncInfoTable.end(); it++)
	  if (isAllowableSprDmpFuncType(it->second.funcType))
	    if (it->first != FmFuncAdmin::FILE_SPECTRUM)
	      toFill.push_back(it->second);
	return;
      }

  // General function, allow all function types,
  // except for internal ones and wave spectrums
  for (it = itsFuncInfoTable.begin(); it != itsFuncInfoTable.end(); it++)
    if (it->second.funcMenuEnum > FmFuncAdmin::UNDEFINED &&
	it->second.funcMenuEnum != FmFuncAdmin::WAVE_SPECTRUM &&
	it->second.funcMenuEnum != FmFuncAdmin::FILE_SPECTRUM)
      toFill.push_back(it->second);
}


FmMathFuncBase* FmFuncAdmin::createFunction(int type)
{
  if      (type == FmfLinVar::getClassTypeID())
    return new FmfLinVar();
  else if (type == FmfConstant::getClassTypeID())
    return new FmfConstant();
  else if (type == FmfSinusoidal::getClassTypeID())
    return new FmfSinusoidal();
  else if (type == FmfComplSinus::getClassTypeID())
    return new FmfComplSinus();
  else if (type == FmfDelayedComplSinus::getClassTypeID())
    return new FmfDelayedComplSinus();
  else if (type == FmfStep::getClassTypeID())
    return new FmfStep();
  else if (type == FmfScale::getClassTypeID())
    return new FmfScale();
  else if (type == FmfSpline::getClassTypeID())
    return new FmfSpline();
  else if (type == FmfRamp::getClassTypeID())
    return new FmfRamp();
  else if (type == FmfSquarePuls::getClassTypeID())
    return new FmfSquarePuls();
  else if (type == FmfDiracPuls::getClassTypeID())
    return new FmfDiracPuls();
  else if (type == FmfLimRamp::getClassTypeID())
    return new FmfLimRamp();
  else if (type == FmfSmoothTraj::getClassTypeID())
    return new FmfSmoothTraj();
  else if (type == FmfLinVelVar::getClassTypeID())
    return new FmfLinVelVar();
  else if (type == FmfDeviceFunction::getClassTypeID())
    return new FmfDeviceFunction();
  else if (type == FmfExternalFunction::getClassTypeID())
    return new FmfExternalFunction();
  else if (type == FmfMathExpr::getClassTypeID())
    return new FmfMathExpr();
  else if (type == FmfWaveSinus::getClassTypeID())
    return new FmfWaveSinus();
  else if (type == FmfWaveSpectrum::getClassTypeID())
    return new FmfWaveSpectrum();
  else if (type == FmfUserDefined::getClassTypeID())
    return new FmfUserDefined();

  return NULL;
}
