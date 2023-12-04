// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdDB.H"
#include "vpmDisplay/FdCtrlDB.H"
#include "vpmDisplay/FdSymbolDefs.H"
#endif

#include "vpmDB/FmDB.H"
#include "vpmDB/FmGlobalViewSettings.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcGLOBAL_VIEW_SETTINGS, FmGlobalViewSettings, FmBase);


FmGlobalViewSettings::FmGlobalViewSettings()
{
  Fmd_CONSTRUCTOR_INIT(FmGlobalViewSettings);

  FFA_FIELD_INIT(itsSpecTriads,    FmSymbolSpec(0.0, 1.0, 0.0), "TRIAD_DATA");
  FFA_FIELD_INIT(itsSpecJoints,    FmSymbolSpec(1.0, 1.0, 0.0), "JOINT_DATA");
  FFA_FIELD_INIT(itsSpecSprDas,    FmSymbolSpec(1.0, 0.3, 1.0), "SPR_DA_DATA");
  FFA_FIELD_INIT(itsSpecHPs,       FmSymbolSpec(0.0, 0.5, 1.0), "HP_DATA");
  FFA_FIELD_INIT(itsSpecStickers,  FmSymbolSpec(1.0, 0.4, 0.0), "STICKER_DATA");
  FFA_FIELD_INIT(itsSpecLoads,     FmSymbolSpec(1.0, 0.0, 0.5), "LOAD_DATA");
  FFA_FIELD_INIT(itsSpecFeedbacks, FmSymbolSpec(0.0, 0.0, 1.0), "FEEDBACK_DATA");
  FFA_FIELD_INIT(itsSpecTires,     FmSymbolSpec(0.0, 0.0, 1.0), "TIRE_DATA");
  FFA_FIELD_INIT(itsSpecRosettes,  FmSymbolSpec(0.0, 0.0, 1.0), "STRAINROSETTE_DATA");

  FFA_FIELD_INIT(itsViewerBackgroundColor, FmColor(0.4, 0.4, 0.4), "BACKGROUND_COLOR");
  FFA_FIELD_INIT(itsInactiveColor,         FmColor(1.0, 1.0, 1.0), "INACTIVE_COLOR");
  FFA_FIELD_INIT(itsGroundedColor,         FmColor(0.0, 0.3, 1.0), "GROUNDED_COLOR");

  FFA_FIELD_INIT(itsIsRevoluteJointsVisible,  true, "VISIBLE_REVOLUTE_JOINTS");
  FFA_FIELD_INIT(itsIsBallJointsVisible,      true, "VISIBLE_BALL_JOINTS");
  FFA_FIELD_INIT(itsIsRigidJointsVisible,     true, "VISIBLE_RIGID_JOINTS");
  FFA_FIELD_INIT(itsIsFreeJointsVisible,      true, "VISIBLE_FREE_JOINTS");
  FFA_FIELD_INIT(itsIsPrismaticJointsVisible, true, "VISIBLE_PRISMATIC_JOINTS");
  FFA_FIELD_INIT(itsIsCylindricJointsVisible, true, "VISIBLE_CYLINDRIC_JOINTS");
  FFA_FIELD_INIT(itsIsCamJointsVisible,       true, "VISIBLE_CAM_JOINTS");

  FFA_FIELD_INIT(itsIsRefPlaneVisible,        true,  "VISIBLE_REF_PLANE");
  FFA_FIELD_INIT(itsIsSeaStateVisible,        false, "VISIBLE_SEA_STATE");
  FFA_FIELD_INIT(itsIsWavesVisible,           true,  "VISIBLE_WAVES");
  FFA_FIELD_INIT(itsIsPartsVisible,           true,  "VISIBLE_PARTS");
  FFA_FIELD_INIT(itsIsPartCSVisible,          false, "VISIBLE_PART_CS");
  FFA_FIELD_INIT(itsIsInternalPartCSsVisible, false, "VISIBLE_INTERNAL_PART_CSS");
  FFA_FIELD_INIT(itsIsPartCoGCSsVisible,      true,  "VISIBLE_PART_COG_CS");
  FFA_FIELD_INIT(itsIsBeamTriadsVisible,      true,  "VISIBLE_BEAM_TRIADS");
  FFA_FIELD_INIT(itsIsBeamsVisible,           true,  "VISIBLE_BEAMS");
  FFA_FIELD_INIT(itsIsBeamCSVisible,          true,  "VISIBLE_BEAM_CS");

  FFA_FIELD_INIT(itsSolidMode,           true,  "SOLID_MODE");
  FFA_FIELD_INIT(itsSolidModeWithEdges,  true,  "SOLID_MODE_WITH_EDGES");
  FFA_FIELD_INIT(itsNiceTransparency,    true,  "NICE_TRANSPARENCY");
  FFA_FIELD_INIT(itsUseAntialiazingFlag, true,  "USE_ANTIALIAZING");
  FFA_FIELD_INIT(itsUseFogFlag,          false, "USE_FOG");
  FFA_FIELD_INIT(itsFogVisibility,       7.5,   "FOG_VISIBILITY");

  FFA_FIELD_INIT(itsSymbolScale,  0.1f, "SYMBOL_SCALE");
  FFA_FIELD_INIT(itsSymbolLineWidth, 1, "SYMBOL_LINE_WIDTH");

  FFA_FIELD_INIT(cameraOrientation, FaMat34(FaVec3(0,0,1.32)), "CAMERA_ORIENTATION");
  FFA_FIELD_INIT(cameraFocalDistance,    1.32,                 "CAMERA_FOCAL_DIST");
  FFA_FIELD_INIT(cameraHeight,           1.2,                  "CAMERA_HEIGHT");
  FFA_FIELD_INIT(cameraOrthographicFlag, true,                 "CAMERA_ORTHOGRAPHIC_FLAG");

  FFA_FIELD_INIT(ctrlTranslation,   FaVec3(0,0,10.7), "CTRLVIEW_TRANSLATION");
  FFA_FIELD_INIT(ctrlFocalDistance, 10.7,             "CTRLVIEW_FOCAL_DIST");

  FFA_FIELD_INIT(ctrlGridOn,        true, "CTRLVIEW_GRID_ON");
  FFA_FIELD_INIT(ctrlGridSizeX,     1.0,  "CTRLVIEW_GRIDSIZE_X");
  FFA_FIELD_INIT(ctrlGridSizeY,     1.0,  "CTRLVIEW_GRIDSIZE_Y");
  FFA_FIELD_INIT(ctrlSnapOn,        true, "CTRLVIEW_SNAP_ON");
  FFA_FIELD_INIT(ctrlSnapDistanceX, 0.25, "CTRLVIEW_SNAPDISTANCE_X");
  FFA_FIELD_INIT(ctrlSnapDistanceY, 0.25, "CTRLVIEW_SNAPDISTANCE_Y");
}


FmGlobalViewSettings::~FmGlobalViewSettings()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmGlobalViewSettings::sync() const
{
  if (FFaAppInfo::isConsole()) return; // TT #2815

#ifdef USE_INVENTOR
  FdDB::showTriads(this->visibleTriads());
  FdDB::showJoints(this->visibleJoints());
  if (this->visibleJoints())
  {
    FdDB::showRevoluteJoints(this->visibleRevoluteJoints());
    FdDB::showBallJoints(this->visibleBallJoints());
    FdDB::showRigidJoints(this->visibleRigidJoints());
    FdDB::showFreeJoints(this->visibleFreeJoints());
    FdDB::showPrismaticJoints(this->visiblePrismaticJoints());
    FdDB::showCylindricJoints(this->visibleCylindricJoints());
    FdDB::showCamJoints(this->visibleCamJoints());
  }
  FdDB::showBeams(this->visibleBeams());
  FdDB::showBeamCS(this->visibleBeamCS());
  FdDB::showParts(this->visibleParts());
  FdDB::showPartCS(this->visiblePartCS());
  FdDB::showInternalPartCSs(this->visibleInternalPartCSs());
  FdDB::setFEBeamSysScale(this->visibleInternalPartCSs() ? this->getSymbolScale() : -1.0);
  FdDB::showPartCoGCSs(this->visiblePartCoGCSs());
  FdDB::showSprDas(this->visibleSprDas());
  FdDB::showHPs(this->visibleHPs());
  FdDB::showStickers(this->visibleStickers());
  FdDB::showLoads(this->visibleLoads());
  FdDB::showFeedbacks(this->visibleFeedbacks());
  FdDB::showStrainRosettes(this->visibleStrainRosettes());
  FdDB::showRefPlanes(this->visibleRefPlanes());
  FdDB::showSeaStates(this->visibleSeaStates());
  FdDB::showWaves(this->visibleWaves());
  FdDB::setTireColor(this->getTireColor());
  FdDB::setViewerBackground(this->getViewerBackgroundColor());
  FdDB::setSolidView(this->getSolidMode());
  FdDB::setNiceTransparency(this->getNiceTransparency());
  FdDB::setAntialiazingOn(this->isAntialiazingOn());
  FdDB::setFogOn(this->isFogOn());
  FdDB::setFogVisibility(this->getFogVisibility());
  FdDB::setView(this->getCameraData());
  FdCtrlDB::setView(this->getCtrlViewData());

  FdSymbolDefs::setTriadColor(this->getTriadColor());
  FdSymbolDefs::setGndTriadColor(this->getGroundedTriadColor());
  FdSymbolDefs::setJointColor(this->getJointColor());
  FdSymbolDefs::setSprDaColor(this->getSprDaColor());
  FdSymbolDefs::setHPColor(this->getHPColor());
  FdSymbolDefs::setStickerColor(this->getStickerColor());
  FdSymbolDefs::setLoadColor(this->getLoadColor());
  FdSymbolDefs::setSensorColor(this->getFeedbackColor());
  FdSymbolDefs::setStrainRosetteColor(this->getStrainRosetteColor());
  FdSymbolDefs::setDefaultColor(this->getInactiveColor());
  FdSymbolDefs::setSymbolLineWidth(this->getSymbolLineWidth());
  FdSymbolDefs::setSymbolScale(this->getSymbolScale());
#endif
}


//****************************** REFERENCE PLANE

bool FmGlobalViewSettings::visibleRefPlanes() const
{
  return itsIsRefPlaneVisible.getValue();
}

void FmGlobalViewSettings::showRefPlanes(bool var)
{
  itsIsRefPlaneVisible = var;
#ifdef USE_INVENTOR
  FdDB::showRefPlanes(var);
#endif
}


//****************************** SEA STATE

bool FmGlobalViewSettings::visibleSeaStates() const
{
  return itsIsSeaStateVisible.getValue();
}

void FmGlobalViewSettings::showSeaStates(bool var)
{
  itsIsSeaStateVisible = var;
#ifdef USE_INVENTOR
  FdDB::showSeaStates(var);
#endif
}

//****************************** WAVES
bool FmGlobalViewSettings::visibleWaves() const
{
  return itsIsWavesVisible.getValue();
}

void FmGlobalViewSettings::showWaves(bool var)
{
  itsIsWavesVisible = var;
#ifdef USE_INVENTOR
  FdDB::showWaves(var);
#endif
}

//****************************** SOLID MODE

bool FmGlobalViewSettings::getSolidMode() const
{
  return itsSolidMode.getValue();
}

bool FmGlobalViewSettings::getSolidModeWithEdges() const
{
  return itsSolidModeWithEdges.getValue();
}

void FmGlobalViewSettings::setSolidMode(bool var, bool showEdgesInSolid)
{
  itsSolidMode = var;
  itsSolidModeWithEdges = showEdgesInSolid;
#ifdef USE_INVENTOR
  FdDB::setSolidView(var);
#endif
}


//****************************** BACKGROUND COLOR

const FmColor& FmGlobalViewSettings::getViewerBackgroundColor() const
{
  return itsViewerBackgroundColor.getValue();
}

void FmGlobalViewSettings::setViewerBackgroundColor(const FmColor& newColor)
{
  itsViewerBackgroundColor = newColor;
#ifdef USE_INVENTOR
  FdDB::setViewerBackground(newColor);
#endif
}


//****************************** INACTIVE COLOR

const FmColor& FmGlobalViewSettings::getInactiveColor() const
{
  return itsInactiveColor.getValue();
}

void FmGlobalViewSettings::setInactiveColor(const FmColor& newColor)
{
  itsInactiveColor = newColor;
#ifdef USE_INVENTOR
  FdSymbolDefs::setDefaultColor(newColor);
#endif
}


//****************************** SYMBOL LINE WIDTH

int FmGlobalViewSettings::getSymbolLineWidth() const
{
  return itsSymbolLineWidth.getValue();
}

void FmGlobalViewSettings::setSymbolLineWidth(int width)
{
  itsSymbolLineWidth = width;
#ifdef USE_INVENTOR
  FdDB::setLineWidth(width);
#endif
}


//****************************** TRANSPARENCY

bool FmGlobalViewSettings::getNiceTransparency() const
{
  return itsNiceTransparency.getValue();
}

void FmGlobalViewSettings::setNiceTransparency(bool mode)
{
  itsNiceTransparency = mode;
#ifdef USE_INVENTOR
  FdDB::setNiceTransparency(mode);
#endif
}


//****************************** ANTIALIAZING

bool FmGlobalViewSettings::isAntialiazingOn() const
{
  return itsUseAntialiazingFlag.getValue();
}

void FmGlobalViewSettings::setAntialiazingOn(bool flag)
{
  itsUseAntialiazingFlag = flag;
#ifdef USE_INVENTOR
  FdDB::setAntialiazingOn(flag);
#endif
}


//****************************** FOG AND FOG VISIBILITY

bool FmGlobalViewSettings::isFogOn() const
{
  return itsUseFogFlag.getValue();
}

double FmGlobalViewSettings::getFogVisibility() const
{
  return itsFogVisibility.getValue();
}

void FmGlobalViewSettings::setFogOn(bool flag)
{
  itsUseFogFlag = flag;
#ifdef USE_INVENTOR
  FdDB::setFogOn(flag);
#endif
}

void FmGlobalViewSettings::setFogVisibility(double visibility)
{
  itsFogVisibility = visibility;
#ifdef USE_INVENTOR
  FdDB::setFogVisibility(visibility);
#endif
}


//****************************** TRIAD

bool FmGlobalViewSettings::visibleTriads() const
{
  return itsSpecTriads.getValue().visible;
}

void FmGlobalViewSettings::showTriads(bool var)
{
  itsSpecTriads.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showTriads(var);
#endif
}


bool FmGlobalViewSettings::visibleBeamTriads() const
{
  return itsIsBeamTriadsVisible.getValue();
}

void FmGlobalViewSettings::showBeamTriads(bool var)
{
  itsIsBeamTriadsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showBeamTriads(var);
#endif
}


const FmColor& FmGlobalViewSettings::getTriadColor() const
{
  return itsSpecTriads.getValue().color;
}

void FmGlobalViewSettings::setTriadColor(const FmColor& color)
{
  itsSpecTriads.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setTriadColor(color);
#endif
}


const FmColor& FmGlobalViewSettings::getGroundedTriadColor() const
{
  return itsGroundedColor.getValue();
}

void FmGlobalViewSettings::setGroundedTriadColor(const FmColor& color)
{
  itsGroundedColor = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setGndTriadColor(color);
#endif
}


//****************************** JOINTS

bool FmGlobalViewSettings::visibleJoints() const
{
  return itsSpecJoints.getValue().visible;
}

void FmGlobalViewSettings::showJoints(bool var)
{
  itsSpecJoints.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showJoints(var);
#endif
}


const FmColor& FmGlobalViewSettings::getJointColor() const
{
  return itsSpecJoints.getValue().color;
}

void FmGlobalViewSettings::setJointColor(const FmColor& color)
{
  itsSpecJoints.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setJointColor(color);
#endif
}

bool FmGlobalViewSettings::visibleRevoluteJoints() const
{
  return itsIsRevoluteJointsVisible.getValue();
}

void FmGlobalViewSettings::showRevoluteJoints(bool var)
{
  itsIsRevoluteJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showRevoluteJoints(var);
#endif
}

bool FmGlobalViewSettings::visibleBallJoints() const
{
  return itsIsBallJointsVisible.getValue();
}

void FmGlobalViewSettings::showBallJoints(bool var)
{
  itsIsBallJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showBallJoints(var);
#endif
}

bool FmGlobalViewSettings::visibleRigidJoints() const
{
  return itsIsRigidJointsVisible.getValue();
}

void FmGlobalViewSettings::showRigidJoints(bool var)
{
  itsIsRigidJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showRigidJoints(var);
#endif
}

bool FmGlobalViewSettings::visibleFreeJoints() const
{
  return itsIsFreeJointsVisible.getValue();
}

void FmGlobalViewSettings::showFreeJoints(bool var)
{
  itsIsFreeJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showFreeJoints(var);
#endif
}

bool FmGlobalViewSettings::visiblePrismaticJoints() const
{
  return itsIsPrismaticJointsVisible.getValue();
}

void FmGlobalViewSettings::showPrismaticJoints(bool var)
{
  itsIsPrismaticJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showPrismaticJoints(var);
#endif
}

bool FmGlobalViewSettings::visibleCylindricJoints() const
{
  return itsIsCylindricJointsVisible.getValue();
}

void FmGlobalViewSettings::showCylindricJoints(bool var)
{
  itsIsCylindricJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showCylindricJoints(var);
#endif
}

bool FmGlobalViewSettings::visibleCamJoints() const
{
  return itsIsCamJointsVisible.getValue();
}

void FmGlobalViewSettings::showCamJoints(bool var)
{
  itsIsCamJointsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showCamJoints(var);
#endif
}

//****************************** BEAMS

bool FmGlobalViewSettings::visibleBeams() const
{
  return itsIsBeamsVisible.getValue();
}

void FmGlobalViewSettings::showBeams(bool var)
{
  itsIsBeamsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showBeams(var);
#endif
}

bool FmGlobalViewSettings::visibleBeamCS() const
{
  return itsIsBeamCSVisible.getValue();
}

void FmGlobalViewSettings::showBeamCS(bool var)
{
  itsIsBeamCSVisible = var;
#ifdef USE_INVENTOR
  FdDB::showBeamCS(var);
#endif
}


//****************************** PARTS

bool FmGlobalViewSettings::visibleParts() const
{
  return itsIsPartsVisible.getValue();
}

void FmGlobalViewSettings::showParts(bool var)
{
  itsIsPartsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showParts(var);
#endif
}

bool FmGlobalViewSettings::visiblePartCS() const
{
  return itsIsPartCSVisible.getValue();
}

void FmGlobalViewSettings::showPartCS(bool var)
{
  itsIsPartCSVisible = var;
#ifdef USE_INVENTOR
  FdDB::showPartCS(var);
#endif
}


bool FmGlobalViewSettings::visibleInternalPartCSs() const
{
  return itsIsInternalPartCSsVisible.getValue();
}

void FmGlobalViewSettings::showInternalPartCSs(bool var)
{
  itsIsInternalPartCSsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showInternalPartCSs(var);
  FdDB::setFEBeamSysScale(var ? this->getSymbolScale() : -1.0);
#endif
}


bool FmGlobalViewSettings::visiblePartCoGCSs() const
{
  return itsIsPartCoGCSsVisible.getValue();
}


void FmGlobalViewSettings::showPartCoGCSs(bool var)
{
  itsIsPartCoGCSsVisible = var;
#ifdef USE_INVENTOR
  FdDB::showPartCoGCSs(var);
#endif
}

const FmColor& FmGlobalViewSettings::getLinkDefaultColor(int index)
{
  static std::vector<FmColor> defaultColors;
  if (defaultColors.empty())
  {
    defaultColors.push_back(FmColor(0.5, 0.5 , 0.36));
    defaultColors.push_back(FmColor(0.0, 0.0 , 1.0));
    defaultColors.push_back(FmColor(0.0, 0.33, 1.0));
    defaultColors.push_back(FmColor(0.0, 0.67, 1.0));
    defaultColors.push_back(FmColor(0.0, 1.0 , 1.0));
    defaultColors.push_back(FmColor(0.0, 0.33, 0.0));
    defaultColors.push_back(FmColor(0.0, 0.67, 0.0));
    defaultColors.push_back(FmColor(0.0, 1.0 , 0.0));
    defaultColors.push_back(FmColor(1.0, 1.0 , 0.0));
    defaultColors.push_back(FmColor(1.0, 0.66, 0.0));
    defaultColors.push_back(FmColor(1.0, 0.33, 0.0));
    defaultColors.push_back(FmColor(1.0, 0.0 , 1.0));
    defaultColors.push_back(FmColor(1.0, 0.33, 1.0));
    defaultColors.push_back(FmColor(1.0, 0.67, 1.0));
  }

  static int n = 0;
  if (index < 0)
    return defaultColors[(n++)%defaultColors.size()];
  else
    return defaultColors[(index)%defaultColors.size()];
}


//****************************** SPRINGS AND DAMPERS

bool FmGlobalViewSettings::visibleSprDas() const
{
  return itsSpecSprDas.getValue().visible;
}

void FmGlobalViewSettings::showSprDas(bool var)
{
  itsSpecSprDas.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showSprDas(var);
#endif
}


const FmColor& FmGlobalViewSettings::getSprDaColor() const
{
  return itsSpecSprDas.getValue().color;
}

void FmGlobalViewSettings::setSprDaColor(const FmColor& color)
{
  itsSpecSprDas.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setSprDaColor(color);
#endif
}


//****************************** HIGHER PAIRS

bool FmGlobalViewSettings::visibleHPs() const
{
  return itsSpecHPs.getValue().visible;
}

void FmGlobalViewSettings::showHPs(bool var)
{
  itsSpecHPs.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showHPs(var);
#endif
}


const FmColor& FmGlobalViewSettings::getHPColor() const
{
  return itsSpecHPs.getValue().color;
}

void FmGlobalViewSettings::setHPColor(const FmColor& color)
{
  itsSpecHPs.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setHPColor(color);
#endif
}


//****************************** STICKERS

bool FmGlobalViewSettings::visibleStickers() const
{
  return itsSpecStickers.getValue().visible;
}

void FmGlobalViewSettings::showStickers(bool var)
{
  itsSpecStickers.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showStickers(var);
#endif
}


const FmColor& FmGlobalViewSettings::getStickerColor() const
{
  return itsSpecStickers.getValue().color;
}

void FmGlobalViewSettings::setStickerColor(const FmColor& color)
{
  itsSpecStickers.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setStickerColor(color);
#endif
}


//****************************** LOADS

bool FmGlobalViewSettings::visibleLoads() const
{
  return itsSpecLoads.getValue().visible;
}

void FmGlobalViewSettings::showLoads(bool var)
{
  itsSpecLoads.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showLoads(var);
#endif
}


const FmColor& FmGlobalViewSettings::getLoadColor() const
{
  return itsSpecLoads.getValue().color;
}

void FmGlobalViewSettings::setLoadColor(const FmColor& color)
{
  itsSpecLoads.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setLoadColor(color);
#endif
}


//****************************** FEEDBACKS (sensors)

bool FmGlobalViewSettings::visibleFeedbacks() const
{
  return itsSpecFeedbacks.getValue().visible;
}

void FmGlobalViewSettings::showFeedbacks(bool var)
{
  itsSpecFeedbacks.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showFeedbacks(var);
#endif
}


const FmColor& FmGlobalViewSettings::getFeedbackColor() const
{
  return itsSpecFeedbacks.getValue().color;
}

void FmGlobalViewSettings::setFeedbackColor(const FmColor& color)
{
  itsSpecFeedbacks.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setSensorColor(color);
#endif
}


//****************************** STRAIN ROSETTES

bool FmGlobalViewSettings::visibleStrainRosettes() const
{
  return itsSpecRosettes.getValue().visible;
}

void FmGlobalViewSettings::showStrainRosettes(bool var)
{
  itsSpecRosettes.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showStrainRosettes(var);
#endif
}


const FmColor& FmGlobalViewSettings::getStrainRosetteColor() const
{
  return itsSpecRosettes.getValue().color;
}

void FmGlobalViewSettings::setStrainRosetteColor(const FmColor& color)
{
  itsSpecRosettes.getValue().color = color;
#ifdef USE_INVENTOR
  FdSymbolDefs::setStrainRosetteColor(color);
#endif
}


//****************************** TIRES

bool FmGlobalViewSettings::visibleTires() const
{
  return itsSpecTires.getValue().visible;
}

void FmGlobalViewSettings::showTires(bool var)
{
  itsSpecTires.getValue().visible = var;
#ifdef USE_INVENTOR
  FdDB::showTires(var);
#endif
}


const FmColor& FmGlobalViewSettings::getTireColor() const
{
  return itsSpecTires.getValue().color;
}

void FmGlobalViewSettings::setTireColor(const FmColor& color)
{
  itsSpecTires.getValue().color = color;
#ifdef USE_INVENTOR
  FdDB::setTireColor(color);
#endif
}


//****************************** SYMBOL SCALING

float FmGlobalViewSettings::getSymbolScale() const
{
  return itsSymbolScale.getValue();
}

void FmGlobalViewSettings::setSymbolScale(float var)
{
#ifdef USE_INVENTOR
  FdSymbolDefs::setSymbolScale(var);
  if (itsSymbolScale.setValue(var))
    FdDB::setFEBeamSysScale(var);
#else
  itsSymbolScale.setValue(var);
#endif
}


//****************************** CAMERA DATA

cameraData& FmGlobalViewSettings::getCameraData() const
{
  static cameraData cData;

  cData.itsCameraOrientation  = cameraOrientation.getValue();
  cData.itsFocalDistance      = cameraFocalDistance.getValue();
  cData.itsHeight             = cameraHeight.getValue();
  cData.itsIsOrthographicFlag = cameraOrthographicFlag.getValue();

  return cData;
}

void FmGlobalViewSettings::setCameraData(const cameraData& cData,
                                         bool updateDisplay)
{
  cameraOrientation      = cData.itsCameraOrientation;
  cameraFocalDistance    = cData.itsFocalDistance;
  cameraHeight           = cData.itsHeight;
  cameraOrthographicFlag = cData.itsIsOrthographicFlag;

#ifdef USE_INVENTOR
  if (updateDisplay) FdDB::setView(cData);
#else
  if (updateDisplay) std::cout <<"FdDB::setView() skipped."<< std::endl;
#endif
}


//****************************** CONTROL VIEW DATA

ctrlViewData& FmGlobalViewSettings::getCtrlViewData() const
{
  static ctrlViewData cvData;

  cvData.itsCameraTranslation = ctrlTranslation.getValue();
  cvData.itsFocalDistance     = ctrlFocalDistance.getValue();

  cvData.isGridOn             = ctrlGridOn.getValue();
  cvData.itsGridSizeX         = ctrlGridSizeX.getValue();
  cvData.itsGridSizeY         = ctrlGridSizeY.getValue();

  cvData.isSnapOn             = ctrlSnapOn.getValue();
  cvData.itsSnapDistanceX     = ctrlSnapDistanceX.getValue();
  cvData.itsSnapDistanceY     = ctrlSnapDistanceY.getValue();

  return cvData;
}

void FmGlobalViewSettings::setCtrlViewData(const ctrlViewData& cvData,
                                           bool updateDisplay)
{
  ctrlTranslation   = cvData.itsCameraTranslation;
  ctrlFocalDistance = cvData.itsFocalDistance;

  ctrlGridOn    = cvData.isGridOn;
  ctrlGridSizeX = cvData.itsGridSizeX;
  ctrlGridSizeY = cvData.itsGridSizeY;

  ctrlSnapOn        = cvData.isSnapOn;
  ctrlSnapDistanceX = cvData.itsSnapDistanceX;
  ctrlSnapDistanceY = cvData.itsSnapDistanceY;

#ifdef USE_INVENTOR
  if (updateDisplay) FdCtrlDB::setView(cvData);
#else
  if (updateDisplay) std::cout <<"FdCtrlDB::setView() skipped."<< std::endl;
#endif
}

////////////////////////////////////////////////////////


bool FmGlobalViewSettings::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmGlobalViewSettings::getClassTypeID());
}


bool FmGlobalViewSettings::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


/***********************************************************************
*
* Input and output from stream
*
************************************************************************/

std::ostream& FmGlobalViewSettings::writeFMF(std::ostream &os)
{
#ifdef USE_INVENTOR
  if (!FFaAppInfo::isConsole())
  {
    // Get the active camera position
    this->setCameraData(FdDB::getView(),false);
    this->setCtrlViewData(FdCtrlDB::getView(),false);
  }
#endif

  os <<"GLOBAL_VIEW_SETTINGS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmGlobalViewSettings::readAndConnect(std::istream& is, std::ostream&)
{
  FmGlobalViewSettings* obj = new FmGlobalViewSettings();

  // Obsolete fields
  FFaObsoleteField<bool>    tireVisible, rosetteVisible;
  FFaObsoleteField<FmColor> tireColor,   rosetteColor;

  FFA_OBSOLETE_FIELD_INIT(tireVisible,    true, "TIRES_VISIBLE", obj);
  FFA_OBSOLETE_FIELD_INIT(rosetteVisible, true, "STRAIN_ROSETTES_VISIBLE", obj);
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(tireColor,    "DEFAULT_TIRE_COLOR", obj);
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(rosetteColor, "STRAIN_ROSETTE_COLOR", obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("TIRES_VISIBLE", obj);
  FFA_OBSOLETE_FIELD_REMOVE("STRAIN_ROSETTES_VISIBLE", obj);
  FFA_OBSOLETE_FIELD_REMOVE("DEFAULT_TIRE_COLOR", obj);
  FFA_OBSOLETE_FIELD_REMOVE("STRAIN_ROSETTE_COLOR", obj);

  if (tireVisible.wasOnFile())
    obj->itsSpecTires.getValue().visible = tireVisible.getValue();
  if (tireColor.wasOnFile())
    obj->itsSpecTires.getValue().color = tireColor.getValue();

  if (rosetteVisible.wasOnFile())
    obj->itsSpecRosettes.getValue().visible = rosetteVisible.getValue();
  if (rosetteColor.wasOnFile())
    obj->itsSpecRosettes.getValue().color = rosetteColor.getValue();

  return obj->cloneOrConnect();
}


bool FmGlobalViewSettings::localParse(const char* keyWord,
                                      std::istream& activeStatement,
                                      FmGlobalViewSettings* obj)
{
  // Substitute LINK by PART in keyWord
  char* link = strstr(const_cast<char*>(keyWord),"LINK");
  if (link)
  {
    const char* part = "PART";
    for (int i = 0; i < 4; i++)
      link[i] = part[i];
  }

  // Substitute some old key words
  const char* key_word;
  if (!strcmp(keyWord,"JOINT_TRIAD_SCALE"))
    key_word = "SYMBOL_SCALE";
  else if (!strcmp(keyWord,"USE_REF_PLANE"))
    key_word = "VISIBLE_REF_PLANE";
  else if (!strcmp(keyWord,"INTERNAL_PARTCSS_VISIBLE"))
    key_word = "VISIBLE_INTERNAL_PART_CSS";
  else
    key_word = keyWord;

  return parentParse(key_word, activeStatement, obj);
}
