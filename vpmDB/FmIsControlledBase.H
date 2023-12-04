// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * CLASS: FmIsControlledBase
 * AUTHOR: Jens LIEN
 * LAST MODIFIED: 14 jun 1995
 *
 * DESCRIPTION:
 * Parent class for the database entries.
 * Has an additional element for the controlled ring.
 **************************************************************************/

#ifndef FM_IS_CONTROLLED_BASE_H
#define FM_IS_CONTROLLED_BASE_H

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmIsControlledBase : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmEngine* getEngine() const { return myEngine.getPointer(); }
  bool setEngine(FmEngine* ctrl) { myEngine.setRef(ctrl); return true; }

  virtual FmModelMemberBase* getActiveOwner() const { return const_cast<FmIsControlledBase*>(this); }

protected:
  FmIsControlledBase();
  virtual ~FmIsControlledBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmIsControlledBase* obj);

private:
  FFaReference<FmEngine>      myEngine;
  FFaField<FFaReferenceBase*> myEngineField;
};

#endif
