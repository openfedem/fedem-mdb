// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmfDeviceFunction.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDB.H"
#include "FFaMathExpr/FFaMathExprFactory.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FiDeviceFunctions/FiDeviceFunctionFactory.H"


Fmd_DB_SOURCE_INIT(FcCURVE_SET, FmCurveSet, FmResultBase);


FmCurveSet::FmCurveSet(InputMode defaultMode)
{
  Fmd_CONSTRUCTOR_INIT(FmCurveSet);

  FFA_REFERENCE_FIELD_INIT(myOwnerGraphField, myOwnerGraph, "OWNER_GRAPH");

  FFA_FIELD_DEFAULT_INIT(myColor,       "CURVE_COLOR");
  FFA_FIELD_INIT(myCurveStyle,       0, "CURVE_STYLE");
  FFA_FIELD_INIT(myCurveWidth,       1, "CURVE_WIDTH");
  FFA_FIELD_INIT(myCurveSymbol,      0, "CURVE_SYMBOL");
  FFA_FIELD_INIT(myCurveSymbolSize,  5, "CURVE_SYMBOL_SIZE");
  FFA_FIELD_INIT(myNumCurveSymbols, 10, "CURVE_NUMBER_SYMBOLS");

  FFA_FIELD_DEFAULT_INIT(myLegend,   "LEGEND");
  FFA_FIELD_INIT(myAutoLegend, true, "AUTOLEGEND");

  FFA_FIELD_INIT(myScaleFactor[XAXIS],  1.0, "SCALE_FACTOR_X");
  FFA_FIELD_INIT(myScaleFactor[YAXIS],  1.0, "SCALE_FACTOR_Y");
  FFA_FIELD_INIT(myOffset[XAXIS],       0.0, "OFFSET_X");
  FFA_FIELD_INIT(myOffset[YAXIS],       0.0, "OFFSET_Y");
  FFA_FIELD_INIT(myZeroAdjust[XAXIS], false, "ZERO_ADJUST_X");
  FFA_FIELD_INIT(myZeroAdjust[YAXIS], false, "ZERO_ADJUST_Y");

  FFA_FIELD_INIT(myInputMode,     defaultMode, "INPUT_MODE");
  FFA_FIELD_INIT(myAutoCurveExportFlag, false, "EXPORT_AUTOMATICALLY");

  if (defaultMode == TEMPORAL_RESULT) {
    FFA_FIELD_INIT(myRDBResults[XAXIS], FFaTimeDescription(), "X_AXIS_RESULT");
    FFA_FIELD_INIT(myRDBResultOpers[XAXIS], "None",   "X_AXIS_RESULT_OPER");
  }
  else if (defaultMode == SPATIAL_RESULT) {
    FFA_FIELD_DEFAULT_INIT(myRDBResults[XAXIS],       "X_AXIS_RESULT");
    FFA_FIELD_INIT(myRDBResultOpers[XAXIS], "Length", "X_AXIS_RESULT_OPER");
  }
  else {
    FFA_FIELD_DEFAULT_INIT(myRDBResults[XAXIS],       "X_AXIS_RESULT");
    FFA_FIELD_DEFAULT_INIT(myRDBResultOpers[XAXIS],   "X_AXIS_RESULT_OPER");
  }
  FFA_FIELD_DEFAULT_INIT(myRDBResults[YAXIS],         "Y_AXIS_RESULT");
  FFA_FIELD_DEFAULT_INIT(myRDBResultOpers[YAXIS],     "Y_AXIS_RESULT_OPER");

  FFA_REFERENCE_FIELD_INIT(myResultObjectField[XAXIS],
                           myResultObject[XAXIS],     "X_AXIS_RESULT_OBJECT");
  FFA_REFERENCE_FIELD_INIT(myResultObjectField[YAXIS],
                           myResultObject[YAXIS],     "Y_AXIS_RESULT_OBJECT");
  myResultObject[XAXIS].setPrintIfZero(false);
  myResultObject[YAXIS].setPrintIfZero(false);

  FFA_REFERENCELIST_FIELD_INIT(mySpatialObjectsField,
                               mySpatialObjects, "SPATIAL_OBJECTS");

  FFA_FIELD_INIT(myTimeRange, FmRange(0.0,1.0), "TIME_RANGE");
  if (defaultMode == SPATIAL_RESULT) {
    FFA_FIELD_INIT(myTimeOper, "None", "TIME_OPER");
  }
  else {
    FFA_FIELD_DEFAULT_INIT(myTimeOper, "TIME_OPER");
  }

  FFA_FIELD_DEFAULT_INIT(myFilePath,    "FILE_PATH");
  FFA_FIELD_DEFAULT_INIT(myChannelName, "CHANNEL_NAME");

  FFA_REFERENCE_FIELD_INIT(myFunctionField, myFunction, "FUNCTION");
  myFunction.setPrintIfZero(false);

  FFA_FIELD_INIT(myFuncDomain, FmRange(0.0,1.0), "FUNC_DOMAIN");
  FFA_FIELD_INIT(myFuncInc,                0.1 , "FUNC_INC");
  FFA_FIELD_INIT(myFuncAutoInc,           false, "FUNC_AUTOINC");

  FFA_FIELD_DEFAULT_INIT(myExpression, "COMBINE_EXPRESSION");
  FFA_REFERENCELIST_FIELD_INIT(myCurvesField, myCurves, "COMBINE_CURVES");

  FFA_FIELD_INIT(myAnalysisFlag,          NONE, "DATA_ANALYSIS");
  FFA_FIELD_INIT(myDftDomain, FmRange(0.0,1.0), "DFT_DOMAIN");
  FFA_FIELD_INIT(myDftResampleRate,       0.01, "DFT_RESAMPLE_RATE");
  FFA_FIELD_INIT(myDftEntireDomain,       true, "DFT_USING_ENTIRE_DOMAIN");
  FFA_FIELD_INIT(myDftRemoveComp,        false, "DFT_REMOVE_STATIC_COMPONENT");
  FFA_FIELD_INIT(myDftResample,          false, "DFT_RESAMPLE_DATA");

  FFA_FIELD_INIT(myFatigueDomain, FmRange(0.0,1.0), "FATIGUE_DOMAIN");
  FFA_FIELD_INIT(myFatigueEntireDomain, true, "FATIGUE_USING_ENTIRE_DOMAIN");
  FFA_FIELD_INIT(myFatigueLifeUnit,  REPEATS, "FATIGUE_LIFE_UNIT");
  FFA_FIELD_INIT(myFatigueGateValue,     1.0, "FATIGUE_GATE_VALUE");
  FFA_FIELD_INIT(myFatigueSNCurve,         0, "FATIGUE_SN_CURVE");
  FFA_FIELD_INIT(myFatigueSNStd,           0, "FATIGUE_SN_STD");

  myXYDataChanged = false;
  myScaleOrOffsetChanged = false;
  myAnalysisOptionChanged = 0;
}


FmCurveSet::~FmCurveSet()
{
  FmGraph* previewGraph = NULL;
  if (myInputMode.getValue() == PREVIEW_FUNC)
    previewGraph = this->getOwnerGraph();

  this->disconnect();

  // Erasing empty preview graphs
  if (previewGraph)
    if (!previewGraph->hasCurveSets())
      previewGraph->erase();
}


bool FmCurveSet::connect(FmBase* parent)
{
  if (parent && parent->isOfType(FmGraph::getClassTypeID()))
    this->setOwnerGraph(static_cast<FmGraph*>(parent));

  return this->mainConnect();
}


bool FmCurveSet::disconnect()
{
  bool status = this->mainDisconnect();
  this->setOwnerGraph(NULL);

  return status;
}


void FmCurveSet::initAfterResolve()
{
  this->FmResultBase::initAfterResolve();

  // To initialize the myActiveCurves array
  if (myInputMode.getValue() == COMB_CURVES)
    this->setExpression(this->getExpression());

  // Align EntryDescription and Reference. Reference is the preferred source.
  // If it is empty, use the entry description, it is then probably a model file
  // saved with R4.1.1 or older.

  for (int axis = 0; axis < NAXES; axis++)
    if (myResultObject[axis].isNull())
      this->setResultObj(axis,myRDBResults[axis].getValue());
    else {
      FFaResultDescription& descr = myRDBResults[axis].getValue();
      descr.OGType = myResultObject[axis]->getItemName();
      descr.baseId = myResultObject[axis]->getItemBaseID();
      descr.userId = myResultObject[axis]->getItemID();
    }
}


bool FmCurveSet::setFilePath(const std::string& path)
{
  if (!myFilePath.setValue(path))
    return false;

  return myXYDataChanged = true;
}


const std::string& FmCurveSet::getChannelName() const
{
  if (!myChannelName.getValue().empty())
    return myChannelName.getValue();

  static const std::string channelNotSet("Not set");
  return channelNotSet;
}


bool FmCurveSet::setChannelName(const std::string& name)
{
  if (!myChannelName.setValue(name == "Not set" ? "" : name.c_str()))
    return false;

  return myXYDataChanged = true;
}


static const char* curveCompNames[17] = {
  "A", "B", "C", "D", "E", "F", "G", "H",
  "I", "J", "K", "L", "M", "N", "O", "P",
  NULL };

const char** FmCurveSet::getCompNames()
{
  return curveCompNames;
}


size_t FmCurveSet::getCurveComps(std::vector<FmCurveSet*>& curves,
				 std::vector<bool>& active) const
{
  myCurves.getPtrs(curves,true);
  active = myActiveCurves;

  return active.size() < curves.size() ? active.size() : curves.size();
}


size_t FmCurveSet::getActiveCurveComps(std::vector<FmCurveSet*>& curves) const
{
  size_t nCurvs = curves.size();
  for (size_t i = 0; i < myCurves.size(); i++)
    if (i < myActiveCurves.size())
      if (myActiveCurves[i] && !myCurves[i].isNull())
	curves.push_back(myCurves.getPtr(i));

  return curves.size() - nCurvs;
}


bool FmCurveSet::setCurveComp(FmCurveSet* curve, int icomp)
{
  if (curve == myCurves.getPtr(icomp))
    return false;

  myCurves.setPtr(curve,icomp);

  return myXYDataChanged = true;
}


bool FmCurveSet::setExpression(const std::string& expression)
{
  FFaMathExprFactory::countArgs(expression,curveCompNames,&myActiveCurves);

  if (!myExpression.setValue(expression))
    return false;

  return myXYDataChanged = true;
}


bool FmCurveSet::setScaleFactor(double scaleX, double scaleY)
{
  bool changed = false;
  if (myScaleFactor[XAXIS].setValue(scaleX))
    changed = myScaleOrOffsetChanged = true;

  if (myScaleFactor[YAXIS].setValue(scaleY))
  {
    changed = myScaleOrOffsetChanged = true;
    if (this->doRainflow() && !myAnalysisOptionChanged)
      myAnalysisOptionChanged = 1;
  }

  if (this->doDft() && myScaleOrOffsetChanged)
    myAnalysisOptionChanged = 2;

  return changed;
}


bool FmCurveSet::setOffset(double offsetX, double offsetY)
{
  bool changed = false;
  if (!myOffset[XAXIS].setValue(offsetX))
    changed = myScaleOrOffsetChanged = true;

  if (!myOffset[YAXIS].setValue(offsetY))
    changed = myScaleOrOffsetChanged = true;

  if (this->doDft() && myScaleOrOffsetChanged)
    myAnalysisOptionChanged = 2;

  return changed;
}


bool FmCurveSet::setZeroAdjust(bool doZeroAdjustX, bool doZeroAdjustY)
{
  bool changed = false;
  if (!myZeroAdjust[XAXIS].setValue(doZeroAdjustX))
    changed = myScaleOrOffsetChanged = true;

  if (!myZeroAdjust[YAXIS].setValue(doZeroAdjustY))
    changed = myScaleOrOffsetChanged = true;

  if (this->doDft() && myScaleOrOffsetChanged)
    myAnalysisOptionChanged = 2;

  return changed;
}


bool FmCurveSet::hasNonDefaultScaleShift() const
{
  for (int a = 0; a < NAXES; a++)
    if (myScaleFactor[a].getValue() != 1.0 ||
	myOffset[a].getValue() != 0.0 ||
	myZeroAdjust[a].getValue())
      return true;

  return false;
}


bool FmCurveSet::isTimeAxis(int axis)
{
  switch (myInputMode.getValue())
    {
    case TEMPORAL_RESULT:
      return this->getResult(axis).isTime();
    case EXT_CURVE:
      // External curves are always assumed to represent some time history
      return (axis == XAXIS);
    case INT_FUNCTION:
    case PREVIEW_FUNC:
      // Internal functions are assumed to be functions of time, only if
      // used as general functions (Engines) or time history input files
      if (axis != XAXIS || myFunction.isNull())
        return false;
      return (myFunction->getFunctionUse() <= FmMathFuncBase::DRIVE_FILE);
    case COMB_CURVES:
      for (size_t i = 0; i < myCurves.size() && i < myActiveCurves.size(); i++)
	if (myActiveCurves[i] && !myCurves[i].isNull())
	  if (!myCurves[i]->isTimeAxis(axis))
	    return false;
      return true;
    default:
      break;
    }

  return false;
}


bool FmCurveSet::isResultDependent() const
{
  switch (myInputMode.getValue())
    {
    case TEMPORAL_RESULT:
    case SPATIAL_RESULT:
      return true;
    case COMB_CURVES:
      for (size_t i = 0; i < myCurves.size(); i++)
	if (i >= myActiveCurves.size())
	  return false;
	else if (myActiveCurves[i] && !myCurves[i].isNull())
	  if (myCurves[i]->isResultDependent())
	    return true;
    default:
      break;
    }

  return false;
}


bool FmCurveSet::needsManualRefresh() const
{
  switch (myInputMode.getValue())
    {
    case TEMPORAL_RESULT:
      return false;
    case SPATIAL_RESULT:
    case EXT_CURVE:
      return true;
    case INT_FUNCTION:
    case PREVIEW_FUNC:
      if (myFunction.isNull())
	return false;
      else
	return myFunction->isOfType(FmfDeviceFunction::getClassTypeID());
    case COMB_CURVES:
      for (size_t i = 0; i < myCurves.size(); i++)
	if (i >= myActiveCurves.size())
	  return false;
	else if (myActiveCurves[i] && !myCurves[i].isNull())
	  if (myCurves[i]->needsManualRefresh())
	    return true;
    default:
      break;
    }

  return false;
}


bool FmCurveSet::setFuncDomain(const FmRange& domain)
{
  if (!myFuncDomain.setValue(domain))
    return false;

  return myXYDataChanged = true;
}


bool FmCurveSet::setIncX(double incX)
{
  if (!myFuncInc.setValue(incX))
    return false;

  return myXYDataChanged = true;
}


bool FmCurveSet::setUseSmartPoints(bool use)
{
  if (!myFuncAutoInc.setValue(use))
    return false;

  return myXYDataChanged = true;
}


bool FmCurveSet::setFunctionRef(FmModelMemberBase* func)
{
  if (func == myFunction.getPointer())
    return false;

  myFunction.setRef(dynamic_cast<FmMathFuncBase*>(func));
  myXYDataChanged = true;

  if (myFunction.isNull())
    return false;

  myFuncAutoInc.setValue(myFunction->hasSmartPoints());

  return true;
}


bool FmCurveSet::setAnalysisFlag(Analysis flag, bool mChange)
{
  bool wasNone = myAnalysisFlag.getValue() == NONE;
  if (!myAnalysisFlag.setValue(flag))
    return false;

  if (mChange)
    myAnalysisOptionChanged = wasNone || flag == NONE ? 1 : 2;

  return true;
}


bool FmCurveSet::hasDFTOptionsChanged(int opt) const
{
  if (this->doAnalysis())
    // Has options changed while analysis is ON?
    return myAnalysisOptionChanged > opt;
  else
    // Has analysis just been switched OFF?
    return myAnalysisOptionChanged == 1;
}


bool FmCurveSet::setDftRemoveComp(bool yesOrNo)
{
  if (!myDftRemoveComp.setValue(yesOrNo))
    return false;

  myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setDftDomain(const FmRange& domain)
{
  if (!myDftDomain.setValue(domain))
    return false;

  myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setDftEntireDomain(bool useOrNot)
{
  if (!myDftEntireDomain.setValue(useOrNot))
    return false;

  myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setDftResample(bool doOrDont)
{
  if (!myDftResample.setValue(doOrDont))
    return false;

  myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setDftResampleRate(double rate)
{
  if (!myDftResampleRate.setValue(rate))
    return false;

  myAnalysisOptionChanged = 2;

  return true;
}


DFTparams FmCurveSet::getDFTparameters() const
{
  return { DFTparams::MAGNITUDE,
    myDftEntireDomain.getValue(),
    myDftResample.getValue(),
    myZeroAdjust[XAXIS].getValue(),
    myZeroAdjust[YAXIS].getValue(),
    myDftRemoveComp.getValue(),
    myDftDomain.getValue().first,
    myDftDomain.getValue().second,
    myDftResampleRate.getValue(),
    myOffset[XAXIS].getValue(),
    myScaleFactor[XAXIS].getValue(),
    myOffset[YAXIS].getValue(),
    myScaleFactor[YAXIS].getValue()
  };
}


bool FmCurveSet::useInputMode(enum InputMode mode, bool isChanged)
{
  if (!myInputMode.setValue(mode))
    return false;

  // Don't set the changed flag in initializations, etc.
  if (isChanged) myXYDataChanged = true;

  return true;
}


const FFaResultDescription& FmCurveSet::getResult(int axis) const
{
  if (!myResultObject[axis].isNull()) {
    FFaResultDescription& descr = ((FmCurveSet*)this)->myRDBResults[axis].getValue();
    descr.OGType = myResultObject[axis]->getItemName();
    descr.baseId = myResultObject[axis]->getItemBaseID();
    descr.userId = myResultObject[axis]->getItemID();
  }

  return myRDBResults[axis].getValue();
}


bool FmCurveSet::clearResult(int axis)
{
  if (myInputMode.getValue() > RDB_RESULT)
    return false; // this curve axis is not plotting results

  if (myRDBResults[axis].getValue().empty() &&
      myRDBResultOpers[axis].getValue().empty())
    return false; // already cleared

  myRDBResults[axis].getValue().clear();
  myRDBResultOpers[axis].getValue().clear();

  return myXYDataChanged = true;
}


bool FmCurveSet::setResult(int axis, const FFaResultDescription& descr)
{
  if (myInputMode.getValue() > RDB_RESULT)
    return false; // this curve axis is not plotting results
  if (!myRDBResults[axis].setValue(descr))
    return false; // unchanged

  myXYDataChanged = true;

  if (myInputMode.getValue() == TEMPORAL_RESULT)
    return this->setResultObj(axis,descr);
  else
    return true;
}


bool FmCurveSet::setResultObj(int axis, const FFaResultDescription& descr)
{
  FmBase* resObj = NULL;
  if (descr.baseId > 0)
    resObj = FmDB::findObject(descr.baseId);
  else if (descr.userId > 0)
    resObj = FmDB::findID(descr.OGType,descr.userId);

  myResultObject[axis].setRef(dynamic_cast<FmIsPlottedBase*>(resObj));

  return !myResultObject[axis].isNull();
}


bool FmCurveSet::setResultOper(int axis, const std::string& oper)
{
  if (myInputMode.getValue() > RDB_RESULT)
    return false; // this curve axis is not plotting results
  if (!myRDBResultOpers[axis].setValue(oper))
    return false; // unchanged

  return myXYDataChanged = true;
}


void FmCurveSet::getSpatialObjs(std::vector<FmIsPlottedBase*>& objs) const
{
  if (myInputMode.getValue() == SPATIAL_RESULT)
    mySpatialObjects.getPtrs(objs);
}


bool FmCurveSet::setSpatialObjs(const std::vector<FmIsPlottedBase*>& objs)
{
  if (myInputMode.getValue() != SPATIAL_RESULT)
    return false; // not a spatial (beam diagram) curve

  mySpatialObjects.setPtrs(objs);

  return myXYDataChanged = true;
}


bool FmCurveSet::setTimeRange(const FmRange& tRange)
{
  if (!myTimeRange.setValue(tRange))
    return false;

  return myXYDataChanged = true;
}


bool FmCurveSet::setTimeOper(const std::string& oper)
{
  if (!myTimeOper.setValue(oper))
    return false;

  return myXYDataChanged = true;
}


void FmCurveSet::changedEvent()
{
  if (!myXYDataChanged)
    return; // no recent changes in the XY-data

  // If this is a preview curve for some function, get the owner graph too
  FmGraph* previewGraph = NULL;
  if (myInputMode.getValue() == PREVIEW_FUNC)
  {
    previewGraph = this->getOwnerGraph();
    // If preview graph description has been changed manually, don't touch it
    if (previewGraph->getUserDescription() != this->getUserDescription() &&
	previewGraph->getUserDescription() != "New Function preview")
      previewGraph = NULL;
  }

  // Update curve legend and description
  this->setAutoLegend(myAutoLegend.getValue());

  if (previewGraph)
  {
    // Make the preview graph description equal to that of the preview curve
    previewGraph->setUserDescription(this->getUserDescription());
    previewGraph->onChanged();
  }

  // Also update any combined curves using this curve as one of its components
  std::vector<FmCurveSet*> curves;
  this->getReferringObjs(curves,"myCurves");
  for (FmCurveSet* curve : curves) curve->reload(true);
}


void FmCurveSet::reload(bool)
{
  if (myInputMode.getValue() >= INT_FUNCTION && !myFunction.isNull())
    if (myFunction->isOfType(FmfDeviceFunction::getClassTypeID()))
      static_cast<FmfDeviceFunction*>(myFunction.getPointer())->close();

  myXYDataChanged = true;

  this->onDataChanged();
}


void FmCurveSet::onDataChanged()
{
  this->onChanged();

  myXYDataChanged = myScaleOrOffsetChanged = false;
  myAnalysisOptionChanged = 0;
}


static bool hasDefaultDescription(const FmCurveSet* curve)
{
  std::string descr = curve->getUserDescription();
  return descr.empty() || descr == curve->getLegend() || descr == "New Curve";
}


bool FmCurveSet::setAutoLegend(bool yesOrNo)
{
  // Lambda function returning a shortened axis legend
  // by removal of unneccesary text from the result description
  auto&& axisText = [this](int axis)
  {
    if (this->getResult(axis).isTime())
      return std::string("Time");

    std::string text = this->getResult(axis).getText();
    std::string textToRemove = " joint variables";
    size_t tPos = text.find(textToRemove);
    if (tPos != std::string::npos)
      text.erase(tPos,textToRemove.size());

    textToRemove = " variables";
    tPos = text.find(textToRemove);
    if (tPos != std::string::npos)
      text.erase(tPos,textToRemove.size());

    textToRemove = "Damper coefficient";
    tPos = text.find(textToRemove);
    if (tPos != std::string::npos)
      text.replace(tPos,textToRemove.size(),"Coefficient");

    textToRemove = ", Position matrix";
    tPos = text.find(textToRemove);
    if (tPos != std::string::npos)
      text.erase(tPos,textToRemove.size());

    textToRemove = "Gage str";
    tPos = text.find(textToRemove);
    if (tPos != std::string::npos)
      text.replace(tPos,textToRemove.size(),"Str");

    textToRemove = "Mechanism [1], ";
    if (text.find(textToRemove) == 0)
      text.erase(0,textToRemove.size());
    else if (!myResultObject[axis].isNull())
    {
      // Insert the user description of the plotted object
      std::string descr = this->myResultObject[axis]->getUserDescription();
      if (!descr.empty())
      {
        tPos = text.find(",");
        if (tPos != std::string::npos)
          text.insert(tPos, " " + descr);
        else
          text += " " + descr;
      }
    }

    // Append the result operation, if any
    const std::string& oper = myRDBResultOpers[axis].getValue();
    if (!oper.empty() && oper != "None")
      text += ", " + oper;

    return text;
  };

  bool isDefaultDescription = hasDefaultDescription(this);
  bool changed = myAutoLegend.setValue(yesOrNo);

  if (myAutoLegend.getValue() && this->areAxesComplete())
    switch (myInputMode.getValue())
      {
      case TEMPORAL_RESULT:
      case SPATIAL_RESULT:
        myLegend = axisText(YAXIS) + " vs " + axisText(XAXIS);
        break;

      case INT_FUNCTION:
      case PREVIEW_FUNC:
        myLegend = "Function: " + myFunction->getInfoString();
        break;

      case EXT_CURVE:
        myLegend = "File: " + myFilePath.getValue();
        if (!myChannelName.getValue().empty())
          myLegend.getValue() += " - " + myChannelName.getValue();
        break;

      case COMB_CURVES:
        myLegend = "Curve combination: " + myExpression.getValue();
        break;

      default:
        break;
      }

  if (isDefaultDescription)
    changed |= this->setUserDescription(myLegend.getValue());

  return changed;
}


bool FmCurveSet::setLegend(const std::string& legend)
{
  bool isDefaultDescription = hasDefaultDescription(this);
  bool changed = myLegend.setValue(legend);

  if (isDefaultDescription)
    this->setUserDescription(myLegend.getValue());

  return changed || isDefaultDescription;
}


bool FmCurveSet::setFatigueDomain(const FmRange& domain)
{
  if (!myFatigueDomain.setValue(domain))
    return false;

  if (this->doRainflow())
    myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setFatigueEntireDomain(bool useOrNot)
{
  if (!myFatigueEntireDomain.setValue(useOrNot))
    return false;

  if (this->doRainflow())
    myAnalysisOptionChanged = 2;

  return true;
}


bool FmCurveSet::setFatigueGateValue(double value)
{
  if (!myFatigueGateValue.setValue(value))
    return false;

  if (this->doRainflow())
    myAnalysisOptionChanged = 2;

  return true;
}


/*!
  Checks if fatigue calculation can be performed on the curve data.
*/

bool FmCurveSet::isFatigueCurve() const
{
  switch (myInputMode.getValue())
    {
    case TEMPORAL_RESULT:
      break;
    case EXT_CURVE:
    case COMB_CURVES:
      return true;
    default:
      return false;
    }

  if (!this->getResult(XAXIS).isTime())
    return false;

  std::string result = this->getResult(YAXIS).getText();
  if (result.find("Strain rosette") == std::string::npos)
    return false;
  else if (result.find("Gage stress") != std::string::npos)
    return true;
  else if (result.find("Stress tensor") != std::string::npos)
    return this->getResultOper(YAXIS).find("Signed Abs Max") != std::string::npos;
  else
    return false;
}


bool FmCurveSet::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmCurveSet::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmCurveSet::getClassTypeID()))
    return false;

  FmCurveSet* copyObj = static_cast<FmCurveSet*>(obj);
  myActiveCurves = copyObj->myActiveCurves; // TT #3018

  if (depth >= FmBase::DEEP_APPEND)
  {
    this->disconnect();
    this->connect(copyObj->getOwnerGraph());
  }

  return true;
}


bool FmCurveSet::areAxesComplete() const
{
  switch (myInputMode.getValue()) {
  case EXT_CURVE:
    if (!myFilePath.getValue().empty())
    {
      const std::string& cFile = myFilePath.getValue();
      const std::string& mPath = FmDB::getMechanismObject()->getAbsModelFilePath();
      int fileType = FiDeviceFunctionFactory::identify(cFile,mPath);
      if (fileType == NON_EXISTING)
	return false;
      else if (fileType != RPC_TH_FILE && fileType != ASC_MC_FILE)
	return true;
      else if (!myChannelName.getValue().empty())
	return true;
    }
    break;

  case INT_FUNCTION:
  case PREVIEW_FUNC:
    if (!myFunction.isNull())
    {
      if (myFuncDomain.getValue().first == myFuncDomain.getValue().second)
	return true;
      else if (myFuncDomain.getValue().first < myFuncDomain.getValue().second)
      {
	if (myFuncAutoInc.getValue())
	  return true;
	else if (myFuncInc.getValue() > 0.0)
	  return true;
      }
    }
    break;

  case COMB_CURVES:
    if (myExpression.getValue().empty())
      return false;

    for (size_t i = 0; i < myActiveCurves.size(); i++)
      if (myActiveCurves[i])
      {
	if (i >= myCurves.size() || myCurves[i].isNull())
	  return false;
	else if (!myCurves[i]->areAxesComplete())
	  return false;
      }
    return true;

  case SPATIAL_RESULT:
    if (mySpatialObjects.size() < 2)
      return false;
    else if (mySpatialObjects[0].isNull())
      return false;
    else if (mySpatialObjects[1].isNull())
      return false;
    else if (myTimeOper.getValue().empty())
      return false;
    else if (this->getResult(YAXIS).empty())
      return false;
    else if (myRDBResultOpers[YAXIS].getValue().empty())
      return false;
    return true;

  case TEMPORAL_RESULT:
    for (int axis = 0; axis < NAXES; axis++)
      if (this->getResult(axis).empty())
	return false;
      else if (myRDBResultOpers[axis].getValue().empty())
	return false;
    return true;

  default:
    break;
  }

  return false;
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmCurveSet::writeFMF(std::ostream &os)
{
  os <<"CURVE_SET\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmCurveSet::readAndConnect(std::istream& is, std::ostream&)
{
  FmCurveSet* obj = new FmCurveSet(EXT_CURVE);

  // Obsolete fields
  FFaObsoleteField<bool> dftDo;
  FFaObsoleteField<double> startDft, stopDft, startFat, stopFat, startX, stopX;
  FFA_OBSOLETE_FIELD_INIT(dftDo,  false, "DFT_PERFORMED", obj);
  FFA_OBSOLETE_FIELD_INIT(startDft, 0.0, "DFT_DOMAIN_START", obj);
  FFA_OBSOLETE_FIELD_INIT(stopDft,  1.0, "DFT_DOMAIN_STOP", obj);
  FFA_OBSOLETE_FIELD_INIT(startFat, 0.0, "FATIGUE_DOMAIN_START", obj);
  FFA_OBSOLETE_FIELD_INIT(stopFat,  1.0, "FATIGUE_DOMAIN_STOP", obj);
  FFA_OBSOLETE_FIELD_INIT(startX,   0.0, "START_X", obj);
  FFA_OBSOLETE_FIELD_INIT(stopX,    1.0, "STOP_X", obj);
  std::string veryOld212fields;

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      if (localParse(keyWord, activeStatement, obj, veryOld212fields))
      {
        // A severe error has occured that requires the parsing to abort
        delete obj;
        return false;
      }
  }

  FFA_OBSOLETE_FIELD_REMOVE("DFT_PERFORMED", obj);
  FFA_OBSOLETE_FIELD_REMOVE("DFT_DOMAIN_START", obj);
  FFA_OBSOLETE_FIELD_REMOVE("DFT_DOMAIN_STOP", obj);
  FFA_OBSOLETE_FIELD_REMOVE("FATIGUE_DOMAIN_START", obj);
  FFA_OBSOLETE_FIELD_REMOVE("FATIGUE_DOMAIN_STOP", obj);
  FFA_OBSOLETE_FIELD_REMOVE("START_X", obj);
  FFA_OBSOLETE_FIELD_REMOVE("STOP_X", obj);

  // Update from old model file
  if (dftDo.wasOnFile() && dftDo.getValue())
    obj->setAnalysisFlag(DFT,false);
  if (startDft.wasOnFile() && stopDft.wasOnFile())
    obj->setDftDomain({startDft.getValue(),stopDft.getValue()});
  if (startFat.wasOnFile() && stopFat.wasOnFile())
    obj->setFatigueDomain({startFat.getValue(),stopFat.getValue()});
  if (startX.wasOnFile() && stopX.wasOnFile())
    obj->setFuncDomain({startX.getValue(),stopX.getValue()});

  FFaFilePath::checkName(obj->myFilePath.getValue());
  if (obj->myChannelName.getValue() == "Not set")
    obj->myChannelName.setValue("");

  // Models older than this have only TEMPORAL_RESULT files
  if (FmDB::getModelFileVer() < FFaVersionNumber(2,5,3,3))
    obj->useInputMode(TEMPORAL_RESULT,false);
  else if (obj->usingInputMode() == RDB_RESULT)
    obj->useInputMode(TEMPORAL_RESULT,false);

  // Give warning in case 2.1.2 keywords were encountered.
  if (obj->connect() && !veryOld212fields.empty())
    ListUI <<"===> The field(s) ["<< veryOld212fields
           <<"] in the CURVE_SET record are no longer supported.\n"
           <<"     Probably, this is an old model created in version 2.1.2.\n"
           <<"     You will have to manually redefine "<< obj->getIdString(true)
           <<".\n";

  return true;
}


bool FmCurveSet::localParse(const char* keyWord, std::istream& activeStatement,
			    FmCurveSet* obj, std::string& obsolete)
{
  // Only some obsolete fields that need to be converted remains here

  enum {CURVE_NUM_SYMBOLS = 1,
	X_AXIS_OBJECT = 2,
	X_AXIS_OBJECT_ID = 3,
	X_AXIS_OBJECT_OPT = 4,
	X_AXIS_ITEM = 5,
	X_AXIS_ITEM_OPT = 6,
	X_AXIS_PROPERTY = 7,
	X_AXIS_PROPERTY_OPT = 8,
	Y_AXIS_OBJECT = 9,
	Y_AXIS_OBJECT_ID = 10,
	Y_AXIS_OBJECT_OPT = 11,
	Y_AXIS_ITEM = 12,
	Y_AXIS_ITEM_OPT = 13,
	Y_AXIS_PROPERTY = 14,
	Y_AXIS_PROPERTY_OPT = 15,
	CURVE_SET_NO = 16,
	USE_EXT_FILE = 17,
	SCALE_FACTOR = 18,
	OFFSET = 19,
	ZERO_ADJUST = 20,
	COLORVEC = 21,
	INC_X = 22,
	USE_SMART_POINTS = 23};

  static const char* key_words[] = {"CURVE_NUM_SYMBOLS",
				    "X_AXIS_OBJECT",
				    "X_AXIS_OBJECT_ID",
				    "X_AXIS_OBJECT_OPT",
				    "X_AXIS_ITEM",
				    "X_AXIS_ITEM_OPT",
				    "X_AXIS_PROPERTY",
				    "X_AXIS_PROPERTY_OPT",
				    "Y_AXIS_OBJECT",
				    "Y_AXIS_OBJECT_ID",
				    "Y_AXIS_OBJECT_OPT",
				    "Y_AXIS_ITEM",
				    "Y_AXIS_ITEM_OPT",
				    "Y_AXIS_PROPERTY",
				    "Y_AXIS_PROPERTY_OPT",
				    "CURVE_SET_NO",
				    "USE_EXT_FILE",
				    "SCALE_FACTOR",
				    "OFFSET",
				    "ZERO_ADJUST",
				    "COLORVEC",
				    "INC_X",
				    "USE_SMART_POINTS",
				    NULL};

  switch (FaParse::findIndex(key_words, keyWord))
    {
    case COLORVEC:
      return parentParse("CURVE_COLOR", activeStatement, obj);

    case INC_X:
      return parentParse("FUNC_INC", activeStatement, obj);

    case USE_SMART_POINTS:
      return parentParse("FUNC_AUTOINC", activeStatement, obj);

    case CURVE_NUM_SYMBOLS:
      {
        // Correcting R2.5 style on number of symbols
        int tmp;
        activeStatement >> tmp;
        switch (tmp)
          {
          case 0:
            obj->myNumCurveSymbols = 10;
            break;
          case 1:
            obj->myNumCurveSymbols = 20;
            break;
          case 2:
            obj->myNumCurveSymbols = 40;
            break;
          case 3:
            obj->myNumCurveSymbols = 60;
            break;
          case 4:
            obj->myNumCurveSymbols = -1;
            break;
          default:
            obj->myNumCurveSymbols = 10;
            break;
          }
      }
      break;

    case X_AXIS_OBJECT:
      return parentParse("X_AXIS_RESULT_OBJECT", activeStatement, obj);

    case Y_AXIS_OBJECT:
      return parentParse("Y_AXIS_RESULT_OBJECT", activeStatement, obj);

    case X_AXIS_OBJECT_ID:
    case X_AXIS_OBJECT_OPT:
    case X_AXIS_ITEM:
    case X_AXIS_ITEM_OPT:
    case X_AXIS_PROPERTY:
    case X_AXIS_PROPERTY_OPT:
    case Y_AXIS_OBJECT_ID:
    case Y_AXIS_OBJECT_OPT:
    case Y_AXIS_ITEM:
    case Y_AXIS_ITEM_OPT:
    case Y_AXIS_PROPERTY:
    case Y_AXIS_PROPERTY_OPT:
      // These fields are no longer supported - convertion is discontinued
      if (obsolete.empty())
	obsolete = keyWord;
      else
	obsolete += std::string(", ") + keyWord;
      break;

    case CURVE_SET_NO:
      {
        int curveSetNo = -1;
        activeStatement >> curveSetNo;
        if (curveSetNo >= 0)
	  obj->setUserDescription(FFaNumStr("Set %d",curveSetNo));
      }
      break;

    case USE_EXT_FILE:
      // In model files written by R3.1 and R3.1.1 (or R3.1-i4 through R3.2-i9),
      // this field contains a single "-character that causes the file parser
      // to miss the end }-character in R5.0 and later (due to bugfix #2835 in
      // FaParse::parseFMFASCII). This is best resolved by aborting the model
      // file parsing and letting the user fix the file manually, since it is
      // very unlikely such old files are around still and used, anyway...
      if (FmDB::getModelFileVer() >= FFaVersionNumber(3,1,0,4))
      {
	ListUI <<"===> ERROR: Can not recover from bug in old model file.\n"
	       <<"            Delete the \"USE_EXT_FILE\" field in all\n   "
	       <<"         \"CURVE_SET\" records in the file and try again.\n";
	return true;
      }
      else
      {
	int oldUseFile = 0;
	activeStatement >> oldUseFile;
	if (!oldUseFile) obj->useInputMode(TEMPORAL_RESULT,false);
      }
      break;

    case SCALE_FACTOR:
      return parentParse("SCALE_FACTOR_Y", activeStatement, obj);

    case OFFSET:
      return parentParse("OFFSET_Y", activeStatement, obj);

    case ZERO_ADJUST:
      return parentParse("ZERO_ADJUST_Y", activeStatement, obj);

    default:
      return parentParse(keyWord, activeStatement, obj);
    }

  return false;
}
