// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_FPP_OPTIONS_H
#define FM_FPP_OPTIONS_H

#include "vpmDB/FmSimulationModelBase.H"
#include "FFaLib/FFaString/FFaEnum.H"

typedef std::pair<double,double> FmRange;


class FmFppOptions : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmFppOptions();

  virtual const char* getUITypeName() const { return "FppOptions"; }

  bool useNCode() const;

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  virtual ~FmFppOptions();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<double> startTime;
  FFaField<double> stopTime;
  FFaField<double> timeIncr;
  FFaField<bool>   allTimeSteps;

  FFaField<int>  nElemsTogether;
  FFaField<bool> performRainflow;

  enum RainFlowType {
    NONE,
    S_N,
    E_N
  };

  FFaEnumMapping(RainFlowType) {
    FFaEnumEntry(NONE,"NONE");
    FFaEnumEntry(S_N, "S_N");
    FFaEnumEntry(E_N, "E_N");
    FFaEnumEntryEnd;
  };

  FFaField<RainFlowTypeEnum> histType;

  FFaField<double> pvxGate;
  FFaField<double> biaxGate;

  FFaField<int>     histNBins;
  FFaField<FmRange> histRange;

  FFaField<std::string> addOptions;
};

#endif
