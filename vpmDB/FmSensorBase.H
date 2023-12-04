// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SENSOR_BASE_H
#define FM_SENSOR_BASE_H

#include "vpmDB/FmIsRenderedBase.H"
#include <vector>

class FmEngine;


class FmSensorBase : public FmIsRenderedBase
{
 Fmd_DB_HEADER_INIT();

public:
  void getEngines(std::vector<FmEngine*>& toFill) const;

  virtual FmIsMeasuredBase* getMeasured(int = 1) const { return 0; }
  virtual void getMeasured(std::vector<FmIsMeasuredBase*>&) const {}
  virtual void removeMeasured() {}

  virtual bool isTime() const { return false; }
  virtual bool isControlOutput() const { return false; }
  virtual bool isExternalCtrlSys() const { return false; }
  virtual bool isDrawable() const { return false; }

  virtual bool hasEntityChoice() const { return false; }
  virtual bool hasDofChoice() const { return false; }

  virtual void getSensorEntities(std::vector<FmSensorChoice>&, int = -1) {}
  virtual void getSensorDofs(std::vector<FmSensorChoice>&) {}

  virtual int printSolverData(FILE* fp, FmEngine*, int) const = 0;

protected:
  FmSensorBase();
  virtual ~FmSensorBase();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmSensorBase* obj);
};

#endif
