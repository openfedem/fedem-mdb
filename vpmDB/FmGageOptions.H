// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_GAGE_OPTIONS_H
#define FM_GAGE_OPTIONS_H

#include "vpmDB/FmSimulationModelBase.H"


class FmGageOptions : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmGageOptions();

  virtual const char* getUITypeName() const { return "GageOptions"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  virtual ~FmGageOptions();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<double> startTime;
  FFaField<double> stopTime;
  FFaField<double> timeIncr;
  FFaField<bool>   allTimeSteps;
  FFaField<bool>   autoDacExport;
  FFaField<double> dacSampleRate;
  FFaField<bool>   fatigue;
  FFaField<double> binSize;

  FFaField<std::string> addOptions;
};

#endif
