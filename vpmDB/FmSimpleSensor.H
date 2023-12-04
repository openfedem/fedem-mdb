// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SIMPLE_SENSOR_H
#define FM_SIMPLE_SENSOR_H

#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmIsMeasuredBase.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmSimpleSensor : public FmSensorBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSimpleSensor();

#ifdef FT_HAS_EXTCTRL
  virtual bool isExternalCtrlSys() const;
#endif
  virtual bool isControlOutput() const;
  virtual bool isDrawable() const;
  virtual bool isListable() const;

  virtual bool hasEntityChoice() const;
  virtual bool hasDofChoice() const;

  virtual void getSensorEntities(std::vector<FmSensorChoice>& ents, int dof = -1);
  virtual void getSensorDofs(std::vector<FmSensorChoice>& dofs);

  void setMeasured(FmIsMeasuredBase* newPt);

  virtual FmIsMeasuredBase* getMeasured(int = 1) const;
  virtual void getMeasured(std::vector<FmIsMeasuredBase*>& measured) const;
  virtual void removeMeasured();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual const char* getUITypeName() const { return "Sensor"; }
  virtual std::string getInfoString() const;

  virtual int printSolverData(FILE* fp, FmEngine* engine, int iarg) const;

protected:
  virtual ~FmSimpleSensor();

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool eraseOptions();

private:
  FFaReference<FmIsMeasuredBase> itsMeasuredPt;
  FFaField<FFaReferenceBase*>    itsMeasuredPtField;
};

#endif
