// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmTimeSensor.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaStringExt.H"


Fmd_DB_SOURCE_INIT(FcTIME_SENSOR, FmTimeSensor, FmSensorBase);


FmTimeSensor::FmTimeSensor() : FmSensorBase()
{
  Fmd_CONSTRUCTOR_INIT(FmTimeSensor);
}


FmTimeSensor::~FmTimeSensor()
{
  this->disconnect();
}


bool FmTimeSensor::connect()
{
  if (this->getNext() != this && this->getPrev() != this)
    return false; // already connected

  FmBase* hPt = FmDB::getHead(this->getTypeID());
  if (!hPt || hPt->getPrev() != hPt)
    return false; // there should only be one time sensor in the model

  this->setID(1);
  this->insertAfter(hPt);
  this->onMainConnected();
  return true;
}


int FmTimeSensor::printSolverData(FILE* fp, FmEngine* eng, int) const
{
  // Beta feature: Sensor measuring the number of iterations
  if (FFaString(eng->getUserDescription()).hasSubString("#NumIt"))
    fprintf(fp,"  type = 'NUM_ITERATIONS'\n");
  else
    fprintf(fp,"  type = 'TIME'\n");
  return 0;
}
