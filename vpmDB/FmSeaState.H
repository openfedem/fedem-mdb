// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SEA_STATE_H
#define FM_SEA_STATE_H

#include "vpmDB/FmIsRenderedBase.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmColor.H"
#include "FFaLib/FFaAlgebra/FFaVec3.H"
#include "FFaLib/FFaAlgebra/FFaMat34.H"


class FmSeaState : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

  using FmRange = std::pair<double,double>;

public:
  FmSeaState();

  virtual const char* getUITypeName() const { return "Sea state"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual void onEventSwitched(const FmSimulationEvent*) { this->draw(); }

  bool setMeanSeaLevel(double z0, bool redraw = true);
  bool setWaveDir(const FaVec3& dir, bool redraw = true);
  bool setSeaDepth(double z, bool redraw = true);
  bool setLengthX(double x, bool redraw = true);
  bool setLengthY(double y, bool redraw = true);

  const FmColor& getRGBColor() const { return myRGBColor.getValue(); }
  bool setRGBColor(const FmColor& col, bool redraw = true);

  double getTransparency() const { return myTransparency.getValue(); }
  bool   setTransparency(double var, bool redraw = true);

  int  getQuantization() const { return myQuantization.getValue(); }
  bool setQuantization(double var, bool redraw = true);
  bool getShowGrid() const { return showGrid.getValue(); }
  bool setShowGrid(bool var, bool redraw = true);
  bool getShowSolid() const { return showSolid.getValue(); }
  bool setShowSolid(bool var, bool redraw = true);

  double getX() const { return x.getValue(); }
  bool   setX(double var, bool redraw = true);
  double getY() const { return y.getValue(); }
  bool   setY(double var, bool redraw = true);

  const FaMat34& getLocalCS() const;
  const FaVec3& getGrav() const;

protected:
  virtual ~FmSeaState();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<double> waterDensity;
  FFaField<double> meanSeaLevel;
  FFaField<double> seaDepth;
  FFaField<FaVec3> waveDir;

  FFaField<double>  growthDensity;
  FFaField<double>  growthThickness;
  FFaField<FmRange> growthLimit;

  FFaReference<FmMathFuncBase> waveFunction;
  FFaField<FFaReferenceBase*>  waveFunctionField;

  FFaReference<FmMathFuncBase> currFunction;
  FFaField<FFaReferenceBase*>  currFunctionField;

  FFaReference<FmMathFuncBase> currentDir;
  FFaField<FFaReferenceBase*>  currentDirField;

  FFaReference<FmEngine>       currScale;
  FFaField<FFaReferenceBase*>  currScaleField;

  FFaReference<FmEngine>       hdfScale;
  FFaField<FFaReferenceBase*>  hdfScaleField;

  // Sea visualization fields
  FFaField<double> xLength;
  FFaField<double> yLength;

private:
  FFaField<FmColor> myRGBColor;
  FFaField<double>  myTransparency;
  FFaField<int>     myQuantization;
  FFaField<bool>    showGrid;
  FFaField<bool>    showSolid;
  FFaField<double>  x;
  FFaField<double>  y;

  mutable FaMat34 myCS;
};

#endif
