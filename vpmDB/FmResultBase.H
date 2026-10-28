// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmResultBase.H
  \brief Base class for all objects that postprocess the simulation results.
*/

#ifndef FM_RESULT_BASE_H
#define FM_RESULT_BASE_H

#include "vpmDB/FmModelMemberBase.H"


/*!
  \brief Base class for all objects that postprocess the simulation results.
*/

class FmResultBase : public FmModelMemberBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

protected:
  //! \brief Default constructor.
  FmResultBase();

  // From FmBase
  virtual bool cloneLocal(FmBase* obj, int depth);

  //! \brief Sets the user description to "New <object typename>".
  virtual void mainConnectedEvent();

  //! \brief Makes a copy of this object.
  FmResultBase* copy(int cloneDepth) const;

public:
  // From FmBase
  virtual bool connect() { return this->eraseOrConnect(); }
  virtual FmBase* clone(int depth) const { return this->copy(depth); }
  using FmBase::clone;
};

#endif
