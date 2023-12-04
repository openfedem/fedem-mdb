// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FWP_H
#define FWP_H

#include <vector>
#include <cstddef>

class FmBladeDesign;
class FmBladeProperty;
class FmBeamProperty;
class FmTurbine;
class FmTower;
class FmNacelle;
class FmGenerator;
class FmGearBox;
class FmShaft;
class FmRotor;
class FmLink;
class FmTriad;
class FmRevJoint;
class FmModelMemberBase;
class FaMat33;


class FWP
{
public:
  static bool updateTurbine(int turbineID = 0);
  static bool updateTurbineTower(int turbineID = 0);

private:
  static bool createTurbine(FmTurbine* theTurbine,
			    FmTower* theTower,
			    FmNacelle* theNacelle,
			    FmGenerator* theGenerator,
			    FmGearBox* theGearBox,
			    FmShaft* theShaft,
			    FmShaft* hsShaft,
			    FmRotor* theRotor);

  static bool updateTurbine(FmTurbine* theTurbine,
			    FmTower* theTower,
			    FmNacelle* theNacelle,
			    FmGenerator* theGenerator,
			    FmGearBox* theGearBox,
			    FmShaft* theShaft,
			    FmShaft* hsShaft,
			    FmRotor* theRotor);

  static bool updateTower(FmTower* theTower);

  static bool updateShaftProps(const FmShaft* shaft, FmBeamProperty* prop);

  static bool createBladeElements(FmRotor* theRotor, FmLink* hub,
				  FmBladeDesign* bladeDef,
				  const std::vector<FmBladeProperty*>& blSeg);

  static bool updateBladeElements(FmRotor* theRotor, FmBladeDesign* bladeDef,
				  const std::vector<FmBladeProperty*>& blSeg);

  static bool createRegulationSystem(FmTurbine* turbine, FmRotor* rotor,
				     FmRevJoint* generator);

  static bool createGraphs(FmModelMemberBase* hub,
			   FmModelMemberBase* tower,
			   FmModelMemberBase* generator);

public:
  static bool createTriadForceGraph(FmModelMemberBase* triad);
  static bool createBeamForceGraph(FmModelMemberBase* beam, int iend = 1);
};

#endif
