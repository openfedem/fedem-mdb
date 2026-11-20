// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmIsRenderedBase.C
  \brief Base class for all objects which can be visualized in the 3D view.
*/

#include "vpmDB/FmIsRenderedBase.H"
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdBase.H"
#endif


//! \cond DO_NOT_DOCUMENT
Fmd_DB_SOURCE_INIT(FcIS_RENDERED_BASE, FmIsRenderedBase, FmIsMeasuredBase);
//! \endcond

FmTriad* FmIsRenderedBase::detachTriad = NULL;


FmIsRenderedBase::FmIsRenderedBase()
{
  Fmd_CONSTRUCTOR_INIT(FmIsRenderedBase);

#ifdef USE_INVENTOR
  itsDisplayPt = NULL;
#endif
}


FmIsRenderedBase::~FmIsRenderedBase()
{
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->fdErase();
#endif
}


bool FmIsRenderedBase::draw(bool updateChildren)
{
  if (FFaAppInfo::isConsole()) return false;

#ifdef USE_INVENTOR
  return itsDisplayPt ? itsDisplayPt->updateFdAll(updateChildren) : true;
#else
  return updateChildren;
#endif
}


bool FmIsRenderedBase::highlight(bool trueOrFalse)
{
  if (FFaAppInfo::isConsole()) return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->highlight(trueOrFalse);
  return true;
#else
  return trueOrFalse;
#endif
}


void FmIsRenderedBase::updateTopologyInViewer(bool updateChildren)
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt) {
    itsDisplayPt->updateFdTopology(updateChildren);
    itsDisplayPt->updateFdDetails();
    itsDisplayPt->updateFdApperance();
  }
#else
  // Dummy statement (to avoid gcc compiler warning)
  if (updateChildren) {}
#endif
}


void FmIsRenderedBase::updateDisplayTopology()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdTopology();
#endif
}


void FmIsRenderedBase::updateDisplayDetails()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdDetails();
#endif
}


void FmIsRenderedBase::updateDisplayCS()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdCS();
#endif
}


bool FmIsRenderedBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmIsRenderedBase::getClassTypeID());
}
