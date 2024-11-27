// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CURVE_SET_H
#define FM_CURVE_SET_H

#include "vpmDB/FmArray.H"
#include "vpmDB/FmColor.H"

#include "FFaLib/FFaString/FFaEnum.H"
#include "FFaLib/FFaDefinitions/FFaResultDescription.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"

#include "vpmDB/FmGraph.H"
#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmMathFuncBase.H"

#include "FFpLib/FFpCurveData/FFpDFTparams.H"


template<>
inline bool FFaField<FFaResultDescription>::isPrintable() const
{
  return !myData.empty();
}


class FmCurveSet : public FmResultBase
{
  Fmd_DB_HEADER_INIT();

public:
  enum { XAXIS, YAXIS, NAXES };

  enum InputMode {
    TEMPORAL_RESULT,
    SPATIAL_RESULT,
    RDB_RESULT,
    COMB_CURVES,
    EXT_CURVE,
    INT_FUNCTION,
    PREVIEW_FUNC
  };

  enum Analysis {
    NONE,
    DERIVATIVE,
    INTEGRAL,
    DFT,
    RAINFLOW
  };

  enum LifeUnit {
    REPEATS,
    HOURS,
    DAYS
  };

private:
  FFaEnumMapping(InputMode) {
    FFaEnumEntry(TEMPORAL_RESULT, "TEMPORAL_RESULT");
    FFaEnumEntry(SPATIAL_RESULT,  "SPATIAL_RESULT");
    FFaEnumEntry(RDB_RESULT,      "RDB_RESULT");
    FFaEnumEntry(COMB_CURVES,     "COMBINED_CURVES");
    FFaEnumEntry(EXT_CURVE,       "EXTERNAL_CURVE");
    FFaEnumEntry(INT_FUNCTION,    "INTERNAL_FUNCTION");
    FFaEnumEntry(PREVIEW_FUNC,    "PREVIEW_FUNCTION");
    FFaEnumEntryEnd;
  };

  FFaEnumMapping(Analysis) {
    FFaEnumEntry(NONE,       "NONE");
    FFaEnumEntry(DERIVATIVE, "DERIVATIVE");
    FFaEnumEntry(INTEGRAL,   "INTEGRAL");
    FFaEnumEntry(DFT,        "DFT");
    FFaEnumEntry(RAINFLOW,   "RAINFLOW");
    FFaEnumEntryEnd;
  };

  FFaEnumMapping(LifeUnit) {
    FFaEnumEntry(REPEATS, "REPEATS");
    FFaEnumEntry(HOURS,   "HOURS");
    FFaEnumEntry(DAYS,    "DAYS");
    FFaEnumEntryEnd;
  };

public:
  FmCurveSet(InputMode defaultMode = TEMPORAL_RESULT);

  void setOwnerGraph(FmGraph* graph) { myOwnerGraph.setRef(graph); }
  FmGraph* getOwnerGraph() const { return myOwnerGraph.getPointer(); }

  bool moveTo(FmGraph* graph) { return graph->addCurveSet(this); }

  bool clearResult(int axis);
  bool setResult(int axis, const FFaResultDescription& descr);
  const FFaResultDescription& getResult(int axis) const;

  bool setResultObj(int axis, const FFaResultDescription& descr);
  FmIsPlottedBase* getResultObj(int axis) const { return myResultObject[axis].getPointer(); }

  bool setResultOper(int axis, const std::string& oper);
  const std::string& getResultOper(int axis) const { return myRDBResultOpers[axis].getValue(); }

  bool setSpatialObjs(const std::vector<FmIsPlottedBase*>& objs);
  void getSpatialObjs(std::vector<FmIsPlottedBase*>& objs) const;

  bool setTimeRange(const FmRange& tRange);
  const FmRange& getTimeRange() const { return myTimeRange.getValue(); }

  bool setTimeOper(const std::string& oper);
  const std::string& getTimeOper() const { return myTimeOper.getValue(); }

  bool isTimeAxis(int axis);

  void onDataChanged();

  void reload(bool recursive = false);

  bool areAxesComplete() const;

  bool hasXYDataChanged() const { return myXYDataChanged; }
  bool hasScaleOrOffsetChanged() const { return myScaleOrOffsetChanged; }

  // Returns true if DFT has just been switched ON/OFF (only OFF when opt = 1),
  // or if some of the DFT-options have been changed while DFT is ON.
  bool hasDFTOptionsChanged(int opt = 0) const;

  bool setAnalysisFlag(Analysis flag, bool mChange = true);
  Analysis getAnalysisFlag() const { return myAnalysisFlag.getValue(); }
  bool doAnalysis() const { return myAnalysisFlag.getValue() > NONE; }
  bool derivate() const { return myAnalysisFlag.getValue() == DERIVATIVE; }
  bool integrate() const { return myAnalysisFlag.getValue() == INTEGRAL; }
  bool doDft() const { return myAnalysisFlag.getValue() == DFT; }
  bool doRainflow() const { return myAnalysisFlag.getValue() == RAINFLOW; }

  bool setAutoLegend(bool yesOrNo);
  bool getAutoLegend() const { return myAutoLegend.getValue(); }

  bool setLegend(const std::string& legend);
  const std::string& getLegend() const { return myLegend.getValue(); }

  // Appearance

  bool setColor(float r, float g, float b) { return this->setColor({r,g,b}); }
  bool setColor(const FmColor& color) { return myColor.setValue(color); }
  const FmColor& getColor() const { return myColor.getValue(); }

  bool setCurveStyle(int style) { return myCurveStyle.setValue(style); }
  int getCurveStyle() const { return myCurveStyle.getValue(); }

  bool setCurveWidth(int width) { return myCurveWidth.setValue(width); }
  int getCurveWidth() const { return myCurveWidth.getValue(); }

  bool setCurveSymbol(int symbol) { return myCurveSymbol.setValue(symbol); }
  int getCurveSymbol() const { return myCurveSymbol.getValue(); }

  bool setCurveSymbolSize(int size) { return myCurveSymbolSize.setValue(size); }
  int getCurveSymbolSize() const { return myCurveSymbolSize.getValue(); }

  bool setCurveNumSymbols(int num) { return myNumCurveSymbols.setValue(num); }
  int getCurveNumSymbols() const { return myNumCurveSymbols.getValue(); }

  // DFT settings

  bool setDftRemoveComp(bool yesOrNo);
  bool getDftRemoveComp() const { return myDftRemoveComp.getValue(); }

  bool setDftDomain(const FmRange& domain);
  const FmRange& getDftDomain() const { return myDftDomain.getValue(); }

  bool setDftEntireDomain(bool useOrNot);
  bool getDftEntireDomain() const { return myDftEntireDomain.getValue(); }

  bool setDftResample(bool doOrDont);
  bool getDftResample() const { return myDftResample.getValue(); }

  bool setDftResampleRate(double rate);
  double getDftResampleRate() const { return myDftResampleRate.getValue(); }

  DFTparams getDFTparameters() const; // used by FFpCurve

  // Scale and Offset

  bool setScaleFactor(double scaleX, double scaleY);
  double getXScale() const { return myScaleFactor[XAXIS].getValue(); }
  double getYScale() const { return myScaleFactor[YAXIS].getValue(); }

  bool setOffset(double offsetX, double offsetY);
  double getXOffset() const { return myOffset[XAXIS].getValue(); }
  double getYOffset() const { return myOffset[YAXIS].getValue(); }

  bool setZeroAdjust(bool doZeroAdjustX, bool doZeroAdjustY);
  bool getXZeroAdjust() const { return myZeroAdjust[XAXIS].getValue(); }
  bool getYZeroAdjust() const { return myZeroAdjust[YAXIS].getValue(); }

  bool hasNonDefaultScaleShift() const;

  // View of data on file

  bool setFilePath(const std::string& path);
  const std::string& getFilePath() const { return myFilePath.getValue(); }

  bool setChannelName(const std::string& channelName);
  const std::string& getChannelName() const;

  // View of internal functions

  bool setFunctionRef(FmModelMemberBase* func);
  FmMathFuncBase* getFunctionRef() const { return myFunction.getPointer(); }

  bool setFuncDomain(const FmRange& domain);
  const FmRange& getFuncDomain() const { return myFuncDomain.getValue(); }

  bool setIncX(double incX);
  double getIncX() const { return myFuncInc.getValue(); }

  bool setUseSmartPoints(bool use);
  bool getUseSmartPoints() const { return myFuncAutoInc.getValue(); }

  // Combined curves

  bool setExpression(const std::string& expression);
  const std::string& getExpression() const { return myExpression.getValue(); }

  bool   setCurveComp(FmCurveSet* curve, int icomp = 0);
  size_t getCurveComps(std::vector<FmCurveSet*>& curves,
                       std::vector<bool>& active) const;
  size_t getActiveCurveComps(std::vector<FmCurveSet*>& curves) const;

  static const char** getCompNames();

  // Various control measures

  bool isResultDependent() const;
  bool needsManualRefresh() const;

  bool useInputMode(InputMode mode, bool isChanged = true);
  InputMode usingInputMode() const { return myInputMode.getValue(); }

  void setToBeExportedBatch(bool yesOrNo) { myAutoCurveExportFlag = yesOrNo; }
  bool isAutoExported() const { return myAutoCurveExportFlag.getValue(); }

  // Fatigue & Rainflow

  bool setFatigueDomain(const FmRange& domain);
  const FmRange& getFatigueDomain() const { return myFatigueDomain.getValue(); }

  bool setFatigueEntireDomain(bool useOrNot);
  bool getFatigueEntireDomain() const { return myFatigueEntireDomain.getValue(); }

  bool setFatigueLifeUnit(LifeUnit unit) { return myFatigueLifeUnit.setValue(unit); }
  LifeUnit getFatigueLifeUnit() const { return myFatigueLifeUnit.getValue(); }

  bool setFatigueSNCurve(int curve) { return myFatigueSNCurve.setValue(curve); }
  int getFatigueSNCurve() const { return myFatigueSNCurve.getValue(); }

  bool setFatigueSNStd(int snStd) { return myFatigueSNStd.setValue(snStd); }
  int getFatigueSNStd() const { return myFatigueSNStd.getValue(); }

  bool setFatigueGateValue(double value);
  double getFatigueGateValue() const { return myFatigueGateValue.getValue(); }

  bool isFatigueCurve() const;

  // Standard stuff

  bool connect(FmBase* parent);
  using FmResultBase::connect;
  virtual bool disconnect();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual const char* getUITypeName() const { return "Curve"; }

protected:
  virtual ~FmCurveSet();

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual void changedEvent();

private:
  static bool localParse(const char* keyWord, std::istream& activeStatement,
                         FmCurveSet* obj, std::string& obsolete);

protected:
  FFaReference<FmGraph>       myOwnerGraph;
  FFaField<FFaReferenceBase*> myOwnerGraphField;

  // Result definition

  FFaField<InputModeEnum> myInputMode;

  FFaReference<FmIsPlottedBase> myResultObject[NAXES];
  FFaField<FFaReferenceBase*>   myResultObjectField[NAXES];

  FFaField<FFaResultDescription> myRDBResults[NAXES];
  FFaField<std::string>          myRDBResultOpers[NAXES];

public:
  FFaField<std::string> myFilePath; // public due to relative path convertion
protected:
  FFaField<std::string> myChannelName;

  FFaReference<FmMathFuncBase> myFunction;
  FFaField<FFaReferenceBase*>  myFunctionField;

  FFaField<FmRange> myFuncDomain;
  FFaField<double>  myFuncInc;
  FFaField<bool>    myFuncAutoInc;

  FFaField<std::string>           myExpression;
  FFaReferenceList<FmCurveSet>    myCurves;
  FFaField<FFaReferenceListBase*> myCurvesField;
  std::set<size_t>                myActiveCurves;

  FFaReferenceList<FmIsPlottedBase> mySpatialObjects;
  FFaField<FFaReferenceListBase*>   mySpatialObjectsField;

  FFaField<FmRange>      myTimeRange;
  FFaField<std::string>  myTimeOper;

  // Curve appearance variables

  FFaField<FmColor> myColor;
  FFaField<int>     myCurveStyle;
  FFaField<int>     myCurveWidth;
  FFaField<int>     myCurveSymbol;
  FFaField<int>     myCurveSymbolSize;
  FFaField<int>     myNumCurveSymbols;

  FFaField<std::string> myLegend;
  FFaField<bool>        myAutoLegend;

  // Options and variables for curve data processing

  FFaField<double> myScaleFactor[NAXES];
  FFaField<double> myOffset[NAXES];
  FFaField<bool>   myZeroAdjust[NAXES];

  FFaField<AnalysisEnum> myAnalysisFlag;

  FFaField<bool>    myDftResample;
  FFaField<bool>    myDftRemoveComp;
  FFaField<bool>    myDftEntireDomain;
  FFaField<FmRange> myDftDomain;
  FFaField<double>  myDftResampleRate;

  FFaField<bool>         myFatigueEntireDomain;
  FFaField<FmRange>      myFatigueDomain;
  FFaField<double>       myFatigueGateValue;
  FFaField<int>          myFatigueSNCurve;
  FFaField<int>          myFatigueSNStd;
  FFaField<LifeUnitEnum> myFatigueLifeUnit;

  FFaField<bool> myAutoCurveExportFlag;

private:
  bool myXYDataChanged;
  bool myScaleOrOffsetChanged;
  char myAnalysisOptionChanged;
};

#endif
