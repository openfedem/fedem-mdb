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

public:
  FmSeaState();

  virtual const char* getUITypeName() const { return "Sea state"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual void onEventSwitched(const FmSimulationEvent*); // Updates visualization

  bool setWaterDensity(double rho);
  bool setMeanSeaLevel(double z0);
  bool setWaveDir(const FaVec3& dir);
  bool setSeaDepth(double z);
  bool setLengthX(double x);
  bool setLengthY(double y);

  const FmColor& getRGBColor() const { return myRGBColor.getValue(); }
  bool setRGBColor(const FmColor& col);

  double getTransparency() const { return myTransparency.getValue(); }
  bool   setTransparency(double var);

  int  getQuantization() const { return myQuantization.getValue(); }
  bool setQuantization(double var);
  bool getShowGrid() const { return showGrid.getValue(); }
  bool setShowGrid(bool var);
  bool getShowSolid() const { return showSolid.getValue(); }
  bool setShowSolid(bool var);

  double getX() const { return x.getValue(); }
  bool setX(double var);
  double getY() const { return y.getValue(); }
  bool setY(double var);

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

  FFaField<double> growthDensity;
  FFaField<double> growthThickness;
  FFaField< std::pair<double,double> > growthLimit;

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
