// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAirState.H"
#include "FFaLib/FFaString/FFaParse.H"


Fmd_DB_SOURCE_INIT(FcAIR_STATE, FmAirState, FmSimulationModelBase);


FmAirState::FmAirState()
{
  Fmd_CONSTRUCTOR_INIT(FmAirState);

  FFA_FIELD_INIT(stallMod ,BEDDOES,"STALL_MODEL");
  FFA_FIELD_INIT(useCM    ,true   ,"USE_CM");
  FFA_FIELD_INIT(infMod   ,EQUIL  ,"INFLOW_MODEL");
  FFA_FIELD_INIT(indMod   ,WAKE   ,"INDUCTION_FACTOR_MODEL");
  FFA_FIELD_INIT(aToler   ,0.005  ,"INDUCTION_FACTOR_TOLERANCE");
  FFA_FIELD_INIT(tlMod    ,NOLOSS ,"TIP_LOSS_MODEL");
  FFA_FIELD_INIT(hlMod    ,NOLOSS ,"HUB_LOSS_MODEL");
  FFA_FIELD_INIT(useWindFile,false,"USE_WIND_FILE");
  FFA_FIELD_INIT(windSpeed,   10.0,"WIND_SPEED");
  FFA_FIELD_INIT(windDirection,0.0,"WIND_DIRECTION");
  FFA_FIELD_DEFAULT_INIT(windFile ,"WIND_FILE");
  FFA_FIELD_INIT(twrPot   ,false  ,"TOWER_POTENTIAL_FLOW");
  FFA_FIELD_INIT(twrShad  ,false  ,"TOWER_SHADOW");
  FFA_FIELD_INIT(airDens  ,1.225  ,"AIR_DENSITY");
  FFA_FIELD_INIT(kinVisc  ,1.46e-5,"AIR_VISCOSITY");
  FFA_FIELD_INIT(dtAero   ,0.01   ,"TIME_INCR");
  FFA_FIELD_INIT(useDSdt  ,true   ,"USE_SOLVER_TIMEINCR");
}


FmAirState::~FmAirState()
{
  this->disconnect();
}


bool FmAirState::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmAirState::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmAirState::getClassTypeID());
}


std::ostream& FmAirState::writeFMF(std::ostream& os)
{
  os <<"AIR_STATE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmAirState::readAndConnect(std::istream& is, std::ostream&)
{
  FmAirState* obj = new FmAirState();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  return obj->cloneOrConnect();
}
