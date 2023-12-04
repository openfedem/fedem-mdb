// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_FUNC_ADMIN_H
#define FM_FUNC_ADMIN_H

#include "FFaLib/FFaTypeCheck/FFaTypeCheck.H"
#include <string>
#include <vector>
#include <map>

class FmMathFuncBase;


struct FmFuncTypeInfo
{
  FmFuncTypeInfo() { funcType = FFaTypeCheck::NO_TYPE_ID; funcMenuEnum = -1; }
  FmFuncTypeInfo(const std::string& name, int ft = FFaTypeCheck::NO_TYPE_ID)
  { listName = name; funcType = ft; funcMenuEnum = -1; }
  int getFuncType() const;

  std::string listName;
  int funcType;
  int funcMenuEnum;
};

typedef std::map<int,FmFuncTypeInfo> FuncInfoMap;


class FmFuncAdmin
{
  static void init();

public:
  static const std::vector<int>& getAllowableSprDmpFuncTypes();

  static bool hasSmartPoints(int type);

  static void getCompatibleFunctionTypes(std::vector<FmFuncTypeInfo>& toFill,
					 FmMathFuncBase* compatibleFunc);

  static FmMathFuncBase* createFunction(int type);

  static void clearInfoTable() { itsFuncInfoTable.clear(); }

private:
  static FuncInfoMap itsFuncInfoTable;
  static std::vector<int> allowableSprDmpFuncs;

  static bool isAllowableSprDmpFuncType(int type);

public:
  enum
  {
    INTERNAL = -2,
    UNDEFINED = -1,

    NONE = 0,

    GENERAL_HEADING,

    LIN_VAR,
    DEVICE,
    SPLINE,
    MATH_EXPRESSION,

    SIMPLE_HEADING,

    CONSTANT,
    SCALE,
    STEP,
    RAMP,
    LIM_RAMP,
    DIRAC_PULS,

    PERIODIC_HEADING,

    SINUSOIDAL,
    COMPL_SINUS,
    DELAYED_COMPL_SINUS,
    WAVE_SINUS,
    WAVE_SPECTRUM,
    FILE_SPECTRUM,
    SQUARE_PULS,

    SPECIAL_HEADING,

    LIN_VEL_VAR,
    SMOOTH_TRAJ,
    EXTERNAL,
    REFERENCE,

    USER_HEADING
  };
};

#endif
