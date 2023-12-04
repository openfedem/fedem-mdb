// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_IS_RENDERED_BASE_H
#define FM_IS_RENDERED_BASE_H

#include "vpmDB/FmIsMeasuredBase.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FaDOF.H"
#else
class FaDOF {};
#endif

class FdObject;
class FmTriad;


class FmIsRenderedBase : public FmIsMeasuredBase
{
  Fmd_DB_HEADER_INIT();

public:
#ifdef USE_INVENTOR
  FdObject* getFdPointer() { return itsDisplayPt; }
#endif

  bool draw();

  virtual void drawObject();
  virtual bool highlight(bool trueOrFalse);
  virtual bool detach() { return false; }
  virtual bool isListable() const { return true; }

  void updateTopologyInViewer();
  void updateThisTopologyOnly();

  void updateDisplayTopology();
  void updateDisplayDetails();
  void updateDisplayApperance();
  void updateDisplayCS();

  virtual FaDOF getObjDegOfFreedom() const { return FaDOF(); }

protected:
  FmIsRenderedBase();
  virtual ~FmIsRenderedBase();

  FmIsRenderedBase(const FmIsRenderedBase&) = delete;
  FmIsRenderedBase& operator=(const FmIsRenderedBase&) = delete;

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmIsRenderedBase* obj);

#ifdef USE_INVENTOR
  FdObject* itsDisplayPt;
#endif

public:
  static FmTriad* detachTriad;
};

#endif
