// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RELATIVE_SENSOR_H
#define FM_RELATIVE_SENSOR_H

#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmIsMeasuredBase.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmRelativeSensor : public FmSensorBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRelativeSensor();

  bool connect(FmIsMeasuredBase* mb1, FmIsMeasuredBase* mb2);
  using FmSensorBase::connect;

  virtual bool isDrawable() const { return this->isConnected(); }

  virtual bool hasEntityChoice() const { return this->isConnected(); }
  virtual bool hasDofChoice() const { return this->isConnected(); }

  virtual void getSensorEntities(std::vector<FmSensorChoice>& choices, int dof=-1);
  virtual void getSensorDofs(std::vector<FmSensorChoice>& choices);

  virtual FmIsMeasuredBase* getMeasured(int ind = 1) const;
  virtual void getMeasured(std::vector<FmIsMeasuredBase*>& measured) const;
  virtual void removeMeasured();

  void setMeasured(FmIsMeasuredBase* mb1, FmIsMeasuredBase* mb2);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual const char* getUITypeName() const { return "Relative sensor"; }
  virtual std::string getInfoString() const;

  virtual int printSolverData(FILE* fp, FmEngine* engine, int iarg) const;

protected:
  virtual ~FmRelativeSensor();

  virtual bool cloneLocal(FmBase* obj, int depth);

  bool isConnected() const;

private:
  FFaReferenceList<FmIsMeasuredBase> itsMeasure;
  FFaField<FFaReferenceListBase*>    itsMeasureField;
};

#endif
