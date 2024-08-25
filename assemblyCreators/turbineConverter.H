// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FWP_H
#define FWP_H

#include <string>
#include <vector>

class FmBladeDesign;
class FmBladeProperty;
class FmRotor;
class FmTurbine;
class FmRevJoint;
class FmLink;
class FmModelMemberBase;


namespace FWP
{
  FmBladeDesign* readBladeDesign(const std::string& bladeDesignFile,
                                 FmBladeDesign* oldDesign);

  bool updateTurbine(int turbineID = 0);
  bool updateTurbineTower(int turbineID = 0);

  bool createTriadForceGraph(FmModelMemberBase* triad);
  bool createBeamForceGraph(FmModelMemberBase* beam, int iend = 1);

  bool createBladeElements(FmRotor* theRotor, FmLink* hub,
                           FmBladeDesign* bladeDef,
                           const std::vector<FmBladeProperty*>& blSeg);
  bool updateBladeElements(FmRotor* theRotor, FmBladeDesign* bladeDef,
                           const std::vector<FmBladeProperty*>& blSeg);

  bool createRegulationSystem(FmTurbine* turbine, FmRotor* rotor,
                              FmRevJoint* generator);

  bool createGraphs(FmModelMemberBase* hub,
                    FmModelMemberBase* tower,
                    FmModelMemberBase* generator);
}

#endif
