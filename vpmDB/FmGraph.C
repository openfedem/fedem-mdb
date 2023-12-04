// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmGraph.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcGRAPH, FmGraph, FmResultBase);


FmGraph::FmGraph(bool beamDiagram)
{
  Fmd_CONSTRUCTOR_INIT(FmGraph);

  FFA_FIELD_DEFAULT_INIT(myTitle, "TITLE");
  FFA_FIELD_DEFAULT_INIT(mySubTitle, "SUB_TITLE");
  FFA_FIELD_DEFAULT_INIT(myXaxisLabel, "X_AXIS_LABEL");
  FFA_FIELD_DEFAULT_INIT(myYaxisLabel, "Y_AXIS_LABEL");
  FFA_FIELD_INIT(myFontSize, GFonts({10,8,8}), "FONT_SIZE");
  FFA_FIELD_INIT(myGridType, 2, "GRID_TYPE");
  FFA_FIELD_INIT(myShowLegend,   false, "SHOW_LEGEND");
  FFA_FIELD_INIT(myAutoScaleFlag, true, "AUTO_SCALE");
  FFA_FIELD_INIT(myUseTimeRange, false, "USE_TIME_RANGE");
  FFA_FIELD_INIT(myTimeRange,  FmRange( 0,1), "TIME_RANGE");
  FFA_FIELD_INIT(myXaxisRange, FmRange(-1,1), "X_AXIS_RANGE");
  FFA_FIELD_INIT(myYaxisRange, FmRange(-1,1), "Y_AXIS_RANGE");
  FFA_FIELD_INIT(myIsBeamDiagram, beamDiagram, "BEAM_DIAGRAM");
}


FmGraph::~FmGraph()
{
  this->disconnect();
  this->removeAllCurves();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char* FmGraph::getUITypeName() const
{
  if (this->isFuncPreview())
    return "Function preview";
  else if (this->isBeamDiagram())
    return "Beam diagram";
  else
    return "Graph";
}


void FmGraph::getCurveSets(std::vector<FmCurveSet*>& curves,
                           bool sortOnId) const
{
  curves.clear();
  this->getReferringObjs(curves,"myOwnerGraph",sortOnId);
}


int FmGraph::getCurveSetCount() const
{
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  return curves.size();
}


/*!
  Returns true if this graph has curves with InputMode \a mode
  or of any InputMode if \a mode < 0.
*/

bool FmGraph::hasCurveSets(int mode) const
{
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);

  if (curves.empty())
    return false;
  else if (mode < 0)
    return true;

  for (FmCurveSet* curve : curves)
    if (curve->usingInputMode() == mode)
      return true;
    else if (curve->usingInputMode() == FmCurveSet::COMB_CURVES)
      curve->getActiveCurveComps(curves); // appends to the existing curves

  return false;
}


bool FmGraph::hasCurve(FmCurveSet* curve) const
{
  if (!curve) return false;

  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  return std::find(curves.begin(),curves.end(),curve) != curves.end();
}


bool FmGraph::addCurveSet(FmCurveSet* curve)
{
  if (!curve) return false;

  curve->disconnect();
  curve->setParentAssembly(this->getParentAssembly());
  curve->connect(this);

  return true;
}


bool FmGraph::addCurveSets(const std::vector<FmCurveSet*>& sets)
{
  for (FmCurveSet* curve : sets)
    this->addCurveSet(curve);

  return true;
}


void FmGraph::removeAllCurves()
{
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  for (FmCurveSet* curve : curves)
  {
    // Prevent auto-deletion of empty preview graphs by FmCurveSet destructor
    if (curve->usingInputMode() == FmCurveSet::PREVIEW_FUNC)
      curve->setOwnerGraph(NULL);
    curve->erase();
  }
}


/*!
  Returns true if this graph has function preview curves only.
*/

bool FmGraph::isFuncPreview() const
{
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  if (curves.empty()) return false;

  for (FmCurveSet* curve : curves)
    if (curve->usingInputMode() != FmCurveSet::PREVIEW_FUNC)
      return false;

  return true;
}


void FmGraph::setTimeRange(double min, double max)
{
  myTimeRange.setValue(FmRange(min,max));
}


void FmGraph::getTimeRange(double& min, double& max) const
{
  min = myTimeRange.getValue().first;
  max = myTimeRange.getValue().second;
}


bool FmGraph::useTimeRange(FmRange& timeRange) const
{
  if (!myUseTimeRange.getValue()) return false;

  // The time range should be used only if all RDB-curves in this graph
  // use Time as their X-axis definition
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  for (FmCurveSet* curve : curves)
    switch (curve->usingInputMode())
      {
      case FmCurveSet::TEMPORAL_RESULT:
      case FmCurveSet::COMB_CURVES:
	if (!curve->isTimeAxis(FmCurveSet::XAXIS))
	  return false;
      default:
        break;
      }

  timeRange = myTimeRange.getValue();
  return true;
}


void FmGraph::setXaxisScale(double min, double max)
{
  myXaxisRange.setValue(FmRange(min,max));
}


void FmGraph::setYaxisScale(double min, double max)
{
  myYaxisRange.setValue(FmRange(min,max));
}


void FmGraph::getXaxisScale(double& min, double& max) const
{
  min = myXaxisRange.getValue().first;
  max = myXaxisRange.getValue().second;
}


void FmGraph::getYaxisScale(double& min, double& max) const
{
  min = myYaxisRange.getValue().first;
  max = myYaxisRange.getValue().second;
}


bool FmGraph::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmGraph::getClassTypeID()))
    return false;

  FmGraph* copyObj = static_cast<FmGraph*>(obj);

  std::vector<FmCurveSet*> tmpCurves;
  copyObj->getCurveSets(tmpCurves);

  if (depth >= FmBase::DEEP_APPEND)
    this->addCurveSets(tmpCurves);

  for (FmCurveSet* curve : tmpCurves)
    curve->sendSignal(MODEL_MEMBER_CONNECTED);

  return true;
}


bool FmGraph::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


void FmGraph::mainConnectedEvent()
{
  if (this->getUserDescription().empty())
    this->setUserDescription(FFaNumStr("Graph %d",this->getID()));
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmGraph::writeFMF(std::ostream& os)
{
  os <<"GRAPH\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmGraph::readAndConnect(std::istream& is, std::ostream&)
{
  FmGraph* obj = new FmGraph();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmGraph::localParse(const char* keyWord, std::istream& activeStatement,
			 FmGraph* obj)
{
  // Only some obsolete fields that need to be converted remains here

  enum {GRIDTYPE = 1,
	SHOWLEGEND = 2,
	X_AXIS_SPEC = 3,
	Y_AXIS_SPEC = 4,
	GRAPH_NO = 5};

  const char* key_words[] = {"GRIDTYPE",    // replaced by GRID_TYPE
			     "SHOWLEGEND",  // replaced by SHOW_LEGEND
			     "X_AXIS_SPEC", // replaced by X_AXIS_RANGE
			     "Y_AXIS_SPEC", // replaced by Y_AXIS_RANGE
			     "GRAPH_NO",    // obsolete, from before R2.5
			     0};

  switch (FaParse::findIndex(key_words, keyWord))
    {
    case GRIDTYPE:
      return parentParse("GRID_TYPE", activeStatement, obj);

    case SHOWLEGEND:
      return parentParse("SHOW_LEGEND", activeStatement, obj);

    case X_AXIS_SPEC:
      activeStatement >> obj->myXaxisRange.getValue().second
		      >> obj->myXaxisRange.getValue().first;
      break;

    case Y_AXIS_SPEC:
      activeStatement >> obj->myYaxisRange.getValue().second
		      >> obj->myYaxisRange.getValue().first;
      break;

    case GRAPH_NO:
      {
	int graphNo = -1;
	activeStatement >> graphNo;
	if (graphNo >= 0) {
	  obj->setUserDescription(FFaNumStr("Graph %d",graphNo));
	  obj->setAutoScaleFlag(true);
	}
      }
      break;

    default:
      return parentParse(keyWord, activeStatement, obj);
    }

  return false;
}


const FmColor& FmGraph::getCurveDefaultColor() const
{
  const std::vector<FmCurveColor>& colors = FmGraph::getCurveDefaultColors();

  int count = this->getCurveSetCount()-1;
  return colors[count > 0 ? count%colors.size() : 0].first;
}


std::vector<FmCurveColor>& FmGraph::getCurveDefaultColors()
{
  static std::vector<FmCurveColor> defaultColors;

  if (defaultColors.empty())
  {
    defaultColors.push_back(FmCurveColor(FmColor(0,           0,           0),           "Black"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           0,           0.823529412), "Dark Blue"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           0,           1),           "Blue"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           0.784313725, 0),           "Dark Green"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           1,           0),           "Green"));
    defaultColors.push_back(FmCurveColor(FmColor(0.941176471, 0,           0.823529412), "Magenta"));
    defaultColors.push_back(FmCurveColor(FmColor(1,           0.666666667, 0),           "Orange"));
    defaultColors.push_back(FmCurveColor(FmColor(1,           0.5,         0),           "Dark Orange"));
    defaultColors.push_back(FmCurveColor(FmColor(0.784313725, 0.784313725, 0),           "Dark Yellow"));
    defaultColors.push_back(FmCurveColor(FmColor(1,           0,           0),           "Red"));
    defaultColors.push_back(FmCurveColor(FmColor(1,           0,           1),           "Purple"));
    defaultColors.push_back(FmCurveColor(FmColor(1,           0.549,       0.549),       "Light Red"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           1,           1),           "Cyan"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           0.784313725, 0.521568627), "Green Cyan"));
    defaultColors.push_back(FmCurveColor(FmColor(0.392156863, 0.392156863, 1),           "Light Blue"));
    defaultColors.push_back(FmCurveColor(FmColor(0.666666667, 0,           1),           "Blue Magenta"));
    defaultColors.push_back(FmCurveColor(FmColor(0,           0.666666667, 1),           "Blue Cyan"));
    defaultColors.push_back(FmCurveColor(FmColor(0.4,         0.4,         0.4),         "Dark Gray"));
  }

  return defaultColors;
}
