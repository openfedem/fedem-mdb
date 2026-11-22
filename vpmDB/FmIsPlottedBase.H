// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmIsPlottedBase.H
  \brief Base class for all objects which can have results that can be plotted.
*/

#ifndef FM_IS_PLOTTED_BASE_H
#define FM_IS_PLOTTED_BASE_H

#include "vpmDB/FmIsRenderedBase.H"

class FmCurveSet;


class FmIsPlottedBase : public FmIsRenderedBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

protected:
  //! \brief Default constructor.
  FmIsPlottedBase();

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

  //! \copybrief FFaFieldContainer::eraseOptions()
  bool eraseOptions() override;

  //! \brief Returns a set of on/off toggles for activation of result output.
  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;

public:
  //! \brief Returns \e true if results from this object is being plotted.
  bool hasCurveSets() const;
  //! \brief Returns the set of curves plotting results from this object.
  void getCurveSets(std::vector<FmCurveSet*>& curves) const;

  //! \brief Writes toggles activating result output to the solver input file-
  void writeSaveVar(FILE* fp, unsigned int nVar) const;

  FFaField<BoolVec> mySaveVar; //!< Toggles for result output
};

#endif
