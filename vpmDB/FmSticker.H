// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_STICKER_H
#define FM_STICKER_H

#include "vpmDB/FmIsRenderedBase.H"
#include "vpmDB/FmIsPositionedBase.H"


class FmSticker : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSticker(const FaVec3& pos = FaVec3());

  void placeAtPoint(const FaVec3& point) { myPosition.setValue(point); }
  const FaVec3& getPoint() const { return myPosition.getValue(); }

  FmIsPositionedBase* getStuckObject() const { return myOwner.getPointer(); }

  bool connect(FmBase* parent);
  using FmIsRenderedBase::connect;
  virtual bool disconnect();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual const char* getUITypeName() const { return "Sticker"; }

#ifdef USE_INVENTOR
  virtual FaDOF getObjDegOfFreedom() const;
#endif

protected:
  virtual ~FmSticker();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReference<FmIsPositionedBase> myOwner;
  FFaField<FFaReferenceBase*>      myOwnerField;

  FFaField<FaVec3> myPosition;
};

#endif
