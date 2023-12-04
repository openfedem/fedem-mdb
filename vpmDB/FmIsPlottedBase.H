// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_IS_PLOTTED_BASE_H
#define FM_IS_PLOTTED_BASE_H

#include "vpmDB/FmIsRenderedBase.H"

class FmCurveSet;


class FmIsPlottedBase : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

protected:
  FmIsPlottedBase();
  virtual ~FmIsPlottedBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual bool eraseOptions();

  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;

public:
  bool hasCurveSets() const;
  void getCurveSets(std::vector<FmCurveSet*>& curves) const;

  void writeSaveVar(FILE* fp, unsigned int nVar) const;

  FFaField<BoolVec> mySaveVar; //!< Toggles for result output
};

#endif
