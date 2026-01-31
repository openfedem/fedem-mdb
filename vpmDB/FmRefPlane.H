// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_REF_PLANE_H
#define FM_REF_PLANE_H

#include "vpmDB/FmIsPositionedBase.H"
#include "vpmDB/FmColor.H"

class FmTriad;


class FmRefPlane : public FmIsPositionedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRefPlane();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual bool isTranslatable() const { return true; }
  virtual char isRotatable() const    { return true; }

  const FmColor& getRGBColor() const { return myRGBColor.getValue(); }
  bool setRGBColor(const FmColor& col);

  double getTransparency() const { return myTransparency.getValue(); }
  bool   setTransparency(double var);

  double getHeight() const { return myHeight.getValue(); }
  double getWidth() const { return myWidth.getValue(); }

  void setHeight(double height);
  void setWidth(double width);

  bool attach(FmBase* attachObject);

  virtual const char* getUITypeName() const { return "Reference plane"; }
  virtual std::string getLinkIDString(bool) const { return "Gnd (Ref plane)"; }

protected:
  virtual ~FmRefPlane();

  virtual bool cloneLocal(FmBase* obj, int depth);

  bool attachTriad(FmTriad* attachTr, bool updateViz = true);

private:
  FFaField<double>  myWidth;
  FFaField<double>  myHeight;
  FFaField<FmColor> myRGBColor;
  FFaField<double>  myTransparency;
};

#endif
