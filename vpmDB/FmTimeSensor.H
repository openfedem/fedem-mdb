// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_TIME_SENSOR_H
#define FM_TIME_SENSOR_H

#include "vpmDB/FmSensorBase.H"


class FmTimeSensor : public FmSensorBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmTimeSensor();

  virtual bool connect();
  virtual bool isTime() const { return true; }
  virtual bool isListable() const { return false; }

  virtual const char* getUITypeName() const { return "Time sensor"; }
  virtual std::string getInfoString() const { return std::string("Time"); }

  virtual int printSolverData(FILE* fp, FmEngine*, int) const;

  // Note that no file IO methods are needed for this class since it contains
  // no data except for the IDs. There should only be one TimeSensor object
  // in a model and it is always genereted when needed.

protected:
  virtual ~FmTimeSensor();
};

#endif
