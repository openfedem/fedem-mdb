// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PIPE_SURFACE_H
#define FM_PIPE_SURFACE_H

#include "vpmDB/FmArcSegmentMaster.H"
#include "vpmDB/FmColor.H"


class FmPipeSurface : public FmArcSegmentMaster
{
  Fmd_DB_HEADER_INIT();

public:
  FmPipeSurface();

  enum VisiblePartEnum {
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
  };

  FFaEnumMapping(VisiblePartEnum) {
    FFaEnumEntry(TOP,   "TOP");
    FFaEnumEntry(BOTTOM,"BOTTOM");
    FFaEnumEntry(LEFT,  "LEFT");
    FFaEnumEntry(RIGHT, "RIGHT");
    FFaEnumEntryEnd;
  };

  VisiblePartEnum getVisiblePart() const { return myVisiblePart.getValue(); }

  void   setPipeRadius(double radius);
  double getPipeRadius() const { return myRadius.getValue(); }

  bool isFacesOn() const { return myFacesIsOn.getValue(); }
  bool isLinesOn() const { return myLinesIsOn.getValue(); }

  const FmColor& getColor() const { return myColor.getValue(); }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual const char* getUITypeName() const { return "Pipe Surface"; }
  virtual bool isListable() const { return true; }

protected:
  virtual ~FmPipeSurface() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmPipeSurface* obj);

private:
  FFaField<VisiblePartEnumEnum> myVisiblePart;
  FFaField<double>              myRadius;

  FFaField<bool>    myFacesIsOn;
  FFaField<bool>    myLinesIsOn;
  FFaField<FmColor> myColor;
};

#endif
