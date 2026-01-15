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

class FmMathFuncBase;


struct FmFuncTypeInfo
{
  FmFuncTypeInfo(const char* fn = NULL, int ft = FFaTypeCheck::NO_TYPE_ID);
  int getFuncType() const;

  std::string listName;
  int funcType;
  int funcMenuEnum;
};


namespace FmFuncAdmin
{
  const std::vector<int>& getAllowableSprDmpFuncTypes();

  bool hasSmartPoints(int type);

  void getCompatibleFunctionTypes(std::vector<FmFuncTypeInfo>& types,
                                  FmMathFuncBase* func);

  FmMathFuncBase* createFunction(int type);

  void clearInfoTable();

  enum FuncType
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
}

#endif
