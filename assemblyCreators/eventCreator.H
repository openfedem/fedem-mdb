// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef SIM_EVENT_CREATOR_H
#define SIM_EVENT_CREATOR_H

#include <iostream>


namespace FWP
{
  bool createEvents(std::istream& is);
  void resolveEvents();
}

#endif
