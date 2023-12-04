// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CONTROL_ADMIN_H
#define FM_CONTROL_ADMIN_H

#include <cstdio>

class FmCtrlElementBase;
class FmCtrlLine;


class FmControlAdmin
{
public:
  static FmCtrlElementBase* createElement(int type);
  static FmCtrlElementBase* createElement(int type, float x, float y);

  static FmCtrlLine* createLine(FmCtrlElementBase* start,
                                FmCtrlElementBase* end, int portNo);

  static int checkControl();
  static int printControl(FILE* fp, int& baseId);
};

#endif
