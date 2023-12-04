// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmIsRenderedBase.H"
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdBase.H"
#endif


Fmd_DB_SOURCE_INIT(FcIS_RENDERED_BASE, FmIsRenderedBase, FmIsMeasuredBase);

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


bool FmIsRenderedBase::draw()
{
  if (FFaAppInfo::isConsole()) return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    return itsDisplayPt->updateFdAll(true);
#endif

  return true;
}


void FmIsRenderedBase::drawObject()
{
#ifdef USE_INVENTOR
  if (itsDisplayPt && !FFaAppInfo::isConsole())
    itsDisplayPt->updateFdAll(false);
#endif
}


bool FmIsRenderedBase::highlight(bool trueOrFalse)
{
  if (FFaAppInfo::isConsole()) return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->highlight(trueOrFalse);
#else
  if (!trueOrFalse)
    return false;
#endif

  return true;
}


void FmIsRenderedBase::updateTopologyInViewer()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt) {
    itsDisplayPt->updateFdTopology(true);
    itsDisplayPt->updateFdDetails();
    itsDisplayPt->updateFdApperance();
  }
#endif
}


void FmIsRenderedBase::updateThisTopologyOnly()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt) {
    itsDisplayPt->updateFdTopology(false);
    itsDisplayPt->updateFdDetails();
    itsDisplayPt->updateFdApperance();
  }
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


void FmIsRenderedBase::updateDisplayApperance()
{
  if (FFaAppInfo::isConsole()) return;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdApperance();
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


bool FmIsRenderedBase::localParse(const char* keyWord,
                                  std::istream& activeStatement,
                                  FmIsRenderedBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmIsRenderedBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmIsRenderedBase::getClassTypeID());
}
