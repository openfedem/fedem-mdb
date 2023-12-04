// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_AIR_STATE_H
#define FM_AIR_STATE_H

#include "vpmDB/FmSimulationModelBase.H"
#include "FFaLib/FFaString/FFaEnum.H"


class FmAirState : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmAirState();

  virtual const char* getUITypeName() const { return "Air state"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  virtual ~FmAirState();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  enum FmStallModel { STEADY, BEDDOES };
  FFaEnumMapping(FmStallModel) {
    FFaEnumEntry(STEADY,  "STEADY");
    FFaEnumEntry(BEDDOES, "BEDDOES");
    FFaEnumEntryEnd;
  };

  enum FmInfModel { DYNIN, EQUIL };
  FFaEnumMapping(FmInfModel) {
    FFaEnumEntry(DYNIN, "DYNIN");
    FFaEnumEntry(EQUIL, "EQUIL");
    FFaEnumEntryEnd;
  };

  enum FmIndModel { NONE, WAKE, SWIRL };
  FFaEnumMapping(FmIndModel) {
    FFaEnumEntry(NONE,  "NONE");
    FFaEnumEntry(WAKE,  "WAKE");
    FFaEnumEntry(SWIRL, "SWIRL");
    FFaEnumEntryEnd;
  };

  enum FmLossModel { NOLOSS, PRANDTL, GTECH };
  FFaEnumMapping(FmLossModel) {
    FFaEnumEntry(NOLOSS,  "NONE");
    FFaEnumEntry(PRANDTL, "PRANDTL");
    FFaEnumEntry(GTECH,   "GTECH");
    FFaEnumEntryEnd;
  };

  FFaField<FmStallModelEnum> stallMod;      //!< Dynamic stall model
  FFaField<bool>             useCM;         //!< Use aerodynamic pitching moment model?
  FFaField<FmInfModelEnum>   infMod;        //!< Inflow model
  FFaField<FmIndModelEnum>   indMod;        //!< Induction-factor model
  FFaField<double>           aToler;        //!< Induction-factor tolerance
  FFaField<FmLossModelEnum>  tlMod;         //!< Tip-loss model (EQUIL only)
  FFaField<FmLossModelEnum>  hlMod;         //!< Hub-loss model (EQUIL only)
  FFaField<bool>             useWindFile;   //!< Wind file defined?
  FFaField<double>           windSpeed;     //!< Wind speed (constant)
  FFaField<double>           windDirection; //!< Wind direction (degrees)
  FFaField<std::string>      windFile;      //!< Name of wind data file
  FFaField<bool>             twrPot;        //!< Calculate tower potential flow?
  FFaField<bool>             twrShad;       //!< Calculate tower shadow?
  FFaField<double>           airDens;       //!< Air density (kg/m^3)
  FFaField<double>           kinVisc;       //!< Kinematic air viscosity
  FFaField<double>           dtAero;        //!< Time interval for aerodynamic calculations
  FFaField<bool>             useDSdt;       //!< Use dynamics solver time interval?
};

#endif
