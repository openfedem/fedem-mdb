// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAnimation.H"
#include "FFaLib/FFaString/FFaParse.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcANIMATION, FmAnimation, FmResultBase);


FmAnimation::FmAnimation()
{
  Fmd_CONSTRUCTOR_INIT(FmAnimation);

  // User controlled animation variables

  FFA_FIELD_INIT(isModesAnimation,    false, "IS_MODES_ANIMATION");
  FFA_FIELD_INIT(isSummaryAnimation,  false, "IS_SUMMARY_ANIMATION");

  FFA_FIELD_INIT(loadFringeData,      false, "LOAD_FRINGE_DATA");
  FFA_FIELD_INIT(loadLineFringeData,  false, "LOAD_LINE_FRINGE_DATA");
  FFA_FIELD_INIT(loadDeformationData, false, "LOAD_DEFORMATION_DATA");

  FFA_FIELD_INIT(autoExport,          false, "EXPORT_AUTOMATICALLY");

  // Fringe setup

  FFA_FIELD_INIT(fringeResultClass,        "Element node", "FRINGE_RESULT_CLASS");
  FFA_FIELD_INIT(fringeVariableName,       "Von Mises stress", "FRINGE_VARIABLE_NAME");
  FFA_FIELD_INIT(fringeToScalarOpName,     "None", "FRINGE_TO_SCALAR_OP_NAME");

  FFA_FIELD_INIT(resultSetSelectionByName, false,          "RESULT_SET_SELECTION_BY_NAME");
  FFA_FIELD_INIT(resSetSelectionOpName,    "Absolute Max", "RESULT_SET_SELECTION_OP_NAME");
  FFA_FIELD_INIT(resSetName,               "Basic",        "RESULT_SET_NAME");

  FFA_FIELD_INIT(multiFaceAveragingOpName, "Absolute Max", "MULTI_FACERES_AVERAGING_OP_NAME");

  // Fringe Averaging setup

  FFA_FIELD_INIT(averagingOpName,          "None", "AVERAGING_OP_NAME");
  FFA_FIELD_INIT(averagingItem,            "Node", "AVERAGING_ITEM");
  FFA_FIELD_INIT(maxMembraneAngleToAverage,  0.17, "MAX_MEMBRANE_ANGLE_TO_AVERAGE_ACROSS");
  FFA_FIELD_INIT(averagingAcrossMaterials,  false, "AVERAGING_ACROSS_MATERIALS");
  FFA_FIELD_INIT(averagingAcrossProperties, false, "AVERAGING_ACROSS_PROPERTIES");
  FFA_FIELD_INIT(averagingAcrossElmTypes,   false, "AVERAGING_ACROSS_ELMTYPES");

  // Time Animation setup

  FFA_FIELD_INIT(usingTimeInterval,              false, "USING_TIME_INTERVAL");
  FFA_FIELD_INIT(timeRange,           FmRange(0.0,1.0), "TIME_INTERVAL");
  FFA_FIELD_INIT(makeFrameForMostFrequentResult, false, "MAKE_FRAME_FOR_MOST_FREQUENT_RESULT");

  // Modes Animation setup

  FFA_REFERENCE_FIELD_INIT(eigenmodePartField, eigenmodePart, "EIGENMODE_PART");
  eigenmodePart.setPrintIfZero(false);

  FFA_FIELD_INIT(eigenmodeType,            SYSTEM_MODES, "EIGENMODE_TYPE");
  FFA_FIELD_INIT(eigenmodeTime,                   0.0,   "EIGENMODE_TIME");
  FFA_FIELD_INIT(eigenmodeNr,                     1,     "EIGENMODE_NR");
  FFA_FIELD_INIT(eigenmodeAmplitude,              1.0,   "EIGENMODE_AMPLITUDE");
  FFA_FIELD_INIT(eigenmodeFramesPrCycle,          50,    "EIGENMODE_FRAMES_PR_CYCLE");
  FFA_FIELD_INIT(eigenmodeDurationUseTime,        false, "EIGENMODE_DURATION_USE_TIME");
  FFA_FIELD_INIT(eigenmodeDurationUseNCycles,     true,  "EIGENMODE_DURATION_USE_NCYCLES");
  FFA_FIELD_INIT(eigenmodeDurationUseUntilDamped, false, "EIGENMODE_DURATION_USE_UNTILDAMPED");
  FFA_FIELD_INIT(eigenmodeDurationTime,           1.0,   "EIGENMODE_DURATION_TIME");
  FFA_FIELD_INIT(eigenmodeDurationNCycles,        1,     "EIGENMODE_DURATIONN_CYCLES");
  FFA_FIELD_INIT(eigenmodeDurationUntilDamped,    90.0,  "EIGENMODE_DURATION_UNTIL_DAMPED");

  // Animation Control

  FFA_REFERENCE_FIELD_INIT(linkToFollowField, linkToFollow, "LINK_TO_FOLLOW");
  linkToFollow.setPrintIfZero(false);

  FFA_FIELD_INIT(showLinkMotion,  true, "SHOW_LINK_MOTION");
  FFA_FIELD_INIT(showTriadMotion, true, "SHOW_TRIAD_MOTION");
  FFA_FIELD_INIT(showDeformation, true, "SHOW_DEFORMATION");
  FFA_FIELD_INIT(deformationScale, 1.0, "DEFORMATION_SCALE");

  FFA_FIELD_INIT(showFringes,    false, "SHOW_FRINGES");
  FFA_FIELD_INIT(showLegend,      true, "SHOW_LEGEND");

  // Fringe Legend Data

  FFA_FIELD_INIT(legendMappingOpName,      "Linear", "LEGEND_MAPPING_OP_NAME");
  FFA_FIELD_INIT(legendRange,      FmRange(0.0,0.0), "LEGEND_RANGE");
  FFA_FIELD_INIT(colorMappingOpName,   "Full color", "COLOR_MAPPING_OP_NAME");
  FFA_FIELD_INIT(smoothLegend,                 true, "SMOOTH_LEGEND");

  FFA_FIELD_INIT(useLegendTickCount,           true, "USE_LEGEND_TICK_COUNT");
  FFA_FIELD_INIT(legendTickCount,                 7, "LEGEND_TICK_COUNT");
  FFA_FIELD_INIT(legendTickSpacing,           1.0e5, "LEGEND_TICK_SPACING");
  FFA_FIELD_INIT(legendTickSpacingIsPrDecade, false, "LEGEND_TICK_SPACING_IS_PR_DECADE");
}


FmAnimation::~FmAnimation()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmAnimation::isHistoryAnimation() const
{
  return !isModesAnimation.getValue() && !isSummaryAnimation.getValue();
}


std::string FmAnimation::getFringeQuantity() const
{
  std::string name = fringeVariableName.getValue();
  if (fringeToScalarOpName.getValue() != "None")
    name = fringeToScalarOpName.getValue() + " " + name;
  if (fringeResultClass.getValue() == "Element node" && averagingOpName.getValue() != "None")
    name += " (" + averagingItem.getValue() + " " + averagingOpName.getValue() + ")";
  else
    name += " on " + fringeResultClass.getValue();

  return name;
}


bool FmAnimation::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmAnimation::getClassTypeID());
}


bool FmAnimation::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmAnimation::writeFMF(std::ostream& os)
{
  os <<"ANIMATION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmAnimation::readAndConnect(std::istream& is, std::ostream&)
{
  FmAnimation* obj = new FmAnimation();

  // Obsolete fields
  FFaObsoleteField<double> startTime;
  FFaObsoleteField<double> endTime;
  FFaObsoleteField<int>  legendMax;
  FFaObsoleteField<int>  legendMin;
  FFaObsoleteField<bool> fringeAveraging;
  FFA_OBSOLETE_FIELD_INIT(startTime,        0.0,"START_TIME",obj);
  FFA_OBSOLETE_FIELD_INIT(endTime,          1.0,"END_TIME",obj);
  FFA_OBSOLETE_FIELD_INIT(legendMax,        0.0,"LEGEND_MAX_VALUE",obj);
  FFA_OBSOLETE_FIELD_INIT(legendMin,        0.0,"LEGEND_MIN_VALUE",obj);
  FFA_OBSOLETE_FIELD_INIT(fringeAveraging,false,"FRINGE_AVERAGING",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"EIGENMODE_LINK") == 0)
	parentParse("EIGENMODE_PART", activeStatement, obj);
      else if (strcmp(keyWord,"TIME_INVERVAL") == 0)
	parentParse("TIME_INTERVAL", activeStatement, obj);
      else
	parentParse(keyWord, activeStatement, obj);
    }
  }

  FFA_OBSOLETE_FIELD_REMOVE("START_TIME",obj);
  FFA_OBSOLETE_FIELD_REMOVE("END_TIME",obj);
  FFA_OBSOLETE_FIELD_REMOVE("LEGEND_MAX_VALUE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("LEGEND_MIN_VALUE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("FRINGE_AVERAGING",obj);

  // Update from old model file
  if (startTime.wasOnFile() && endTime.wasOnFile())
    obj->timeRange = FmRange(startTime.getValue(),endTime.getValue());
  if (legendMax.wasOnFile() && legendMin.wasOnFile())
    obj->timeRange = FmRange(legendMin.getValue(),legendMax.getValue());
  if (fringeAveraging.wasOnFile())
    if (!fringeAveraging.getValue())
      obj->averagingOpName = "None";

  obj->connect();
  return true;
}
