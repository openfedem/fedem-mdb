// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_GLOBAL_VIEW_SETTINGS_H
#define FM_GLOBAL_VIEW_SETTINGS_H

#include "vpmDB/FmBase.H"
#include "vpmDB/FmColor.H"
#include "FFaLib/FFaAlgebra/FFaMat34.H"


struct cameraData
{
  FaMat34 itsCameraOrientation;
  double  itsFocalDistance;
  double  itsHeight;
  bool    itsIsOrthographicFlag;
};


struct ctrlViewData
{
  FaVec3 itsCameraTranslation;
  double itsFocalDistance;

  bool   isGridOn;
  double itsGridSizeX;
  double itsGridSizeY;

  bool   isSnapOn;
  double itsSnapDistanceX;
  double itsSnapDistanceY;
};


class FmGlobalViewSettings : public FmBase
{
private:

  class FmSymbolSpec
  {
  public:
    bool visible;
    FmColor color;
    FmSymbolSpec() : visible(false), color(0,0,0) {}
    FmSymbolSpec(double r, double g, double b) : visible(true), color(r,g,b) {}
    bool operator==(const FmSymbolSpec& obj) const
    { return visible == obj.visible && color == obj.color; }
    friend std::ostream& operator<<(std::ostream& os, const FmSymbolSpec& obj)
    { return os << obj.visible <<" "<< obj.color; }
    friend std::istream& operator>>(std::istream& is, FmSymbolSpec& obj)
    { is >> obj.visible >> obj.color; return is; }
  };

  Fmd_DB_HEADER_INIT();

  FFaField<bool>   itsIsRevoluteJointsVisible;
  FFaField<bool>   itsIsBallJointsVisible;
  FFaField<bool>   itsIsRigidJointsVisible;
  FFaField<bool>   itsIsFreeJointsVisible;
  FFaField<bool>   itsIsPrismaticJointsVisible;
  FFaField<bool>   itsIsCylindricJointsVisible;
  FFaField<bool>   itsIsCamJointsVisible;

  FFaField<bool>   itsIsRefPlaneVisible;
  FFaField<bool>   itsIsSeaStateVisible;
  FFaField<bool>   itsIsWavesVisible;
  FFaField<bool>   itsIsPartsVisible;
  FFaField<bool>   itsIsPartCSVisible;
  FFaField<bool>   itsIsInternalPartCSsVisible;
  FFaField<bool>   itsIsPartCoGCSsVisible;
  FFaField<bool>   itsIsBeamTriadsVisible;
  FFaField<bool>   itsIsBeamsVisible;
  FFaField<bool>   itsIsBeamCSVisible;

  FFaField<FmColor> itsInactiveColor;
  FFaField<FmColor> itsGroundedColor;
  FFaField<FmColor> itsViewerBackgroundColor;

  FFaField<bool>   itsSolidMode;
  FFaField<bool>   itsSolidModeWithEdges;

  FFaField<bool>   itsNiceTransparency;
  FFaField<bool>   itsUseAntialiazingFlag;
  FFaField<bool>   itsUseFogFlag;
  FFaField<double> itsFogVisibility;

  FFaField<FmSymbolSpec> itsSpecFeedbacks;
  FFaField<FmSymbolSpec> itsSpecHPs;
  FFaField<FmSymbolSpec> itsSpecJoints;
  FFaField<FmSymbolSpec> itsSpecLoads;
  FFaField<FmSymbolSpec> itsSpecSprDas;
  FFaField<FmSymbolSpec> itsSpecStickers;
  FFaField<FmSymbolSpec> itsSpecTriads;
  FFaField<FmSymbolSpec> itsSpecTires;
  FFaField<FmSymbolSpec> itsSpecRosettes;

  FFaField<float>   itsSymbolScale;
  FFaField<int>     itsSymbolLineWidth;

  FFaField<FaMat34> cameraOrientation;
  FFaField<double>  cameraFocalDistance;
  FFaField<double>  cameraHeight;
  FFaField<bool>    cameraOrthographicFlag;

  FFaField<FaVec3>  ctrlTranslation;
  FFaField<double>  ctrlFocalDistance;
  FFaField<double>  ctrlHeight;

  FFaField<bool>    ctrlGridOn;
  FFaField<double>  ctrlGridSizeX;
  FFaField<double>  ctrlGridSizeY;

  FFaField<bool>    ctrlSnapOn;
  FFaField<double>  ctrlSnapDistanceX;
  FFaField<double>  ctrlSnapDistanceY;

protected:
  virtual ~FmGlobalViewSettings();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmGlobalViewSettings* obj);

public:
  FmGlobalViewSettings();

  bool getNiceTransparency() const;
  void setNiceTransparency(bool transpMode);

  bool isAntialiazingOn() const;
  void setAntialiazingOn(bool flag);

  bool isFogOn() const;
  void setFogOn(bool flag);

  double getFogVisibility() const;
  void setFogVisibility(double vis);

  void sync() const;

  bool visibleRevoluteJoints() const;
  bool visibleBallJoints() const;
  bool visibleRigidJoints() const;
  bool visibleFreeJoints() const;
  bool visiblePrismaticJoints() const;
  bool visibleCylindricJoints() const;
  bool visibleCamJoints() const;

  bool visibleFeedbacks() const;
  bool visibleHPs() const;
  bool visibleJoints() const;
  bool visibleParts() const;
  bool visiblePartCS() const;
  bool visibleInternalPartCSs() const;
  bool visiblePartCoGCSs() const;
  bool visibleBeams() const;
  bool visibleBeamCS() const;
  bool visibleBeamTriads() const;
  bool visibleLoads() const;
  bool visibleSprDas() const;
  bool visibleStickers() const;
  bool visibleTriads() const;
  bool visibleRefPlanes() const;
  bool visibleSeaStates() const;
  bool visibleWaves() const;
  bool visibleTires() const;
  bool visibleStrainRosettes() const;

  void showRevoluteJoints(bool);
  void showBallJoints(bool);
  void showRigidJoints(bool);
  void showFreeJoints(bool);
  void showPrismaticJoints(bool);
  void showCylindricJoints(bool);
  void showCamJoints(bool);

  void showFeedbacks(bool);
  void showHPs(bool);
  void showJoints(bool);
  void showParts(bool);
  void showPartCS(bool);
  void showBeams(bool);
  void showBeamCS(bool);
  void showInternalPartCSs(bool);
  void showPartCoGCSs(bool);
  void showBeamTriads(bool);
  void showLoads(bool);
  void showSprDas(bool);
  void showStickers(bool);
  void showTriads(bool);
  void showRefPlanes(bool);
  void showSeaStates(bool);
  void showWaves(bool);
  void showTires(bool);
  void showStrainRosettes(bool);

  static const FmColor& getLinkDefaultColor(int index = -1);

  const FmColor& getFeedbackColor() const;
  void setFeedbackColor(const FmColor& color);

  const FmColor& getHPColor() const;
  void setHPColor(const FmColor& color);

  const FmColor& getJointColor() const;
  void setJointColor(const FmColor& color);

  const FmColor& getLoadColor() const;
  void setLoadColor(const FmColor& color);

  const FmColor& getSprDaColor() const;
  void setSprDaColor(const FmColor& color);

  const FmColor& getStickerColor() const;
  void setStickerColor(const FmColor& color);

  const FmColor& getTriadColor() const;
  void setTriadColor(const FmColor& color);

  const FmColor& getGroundedTriadColor() const;
  void setGroundedTriadColor(const FmColor& color);

  const FmColor& getViewerBackgroundColor() const;
  void setViewerBackgroundColor(const FmColor& color);

  const FmColor& getInactiveColor() const;
  void setInactiveColor(const FmColor& color);

  const FmColor& getTireColor() const;
  void setTireColor(const FmColor& color);

  const FmColor& getStrainRosetteColor() const;
  void setStrainRosetteColor(const FmColor& color);

  bool getSolidMode() const;
  bool getSolidModeWithEdges() const;
  void setSolidMode(bool mode, bool showEdgesInSolid = true);

  int getSymbolLineWidth() const;
  void setSymbolLineWidth(int width);

  float getSymbolScale() const;
  void setSymbolScale(float var);

  cameraData& getCameraData() const;
  void setCameraData(const cameraData& cData, bool updateDisplay = true);

  ctrlViewData& getCtrlViewData() const;
  void setCtrlViewData(const ctrlViewData& cvData, bool updateDisplay = true);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Global view settings"; }
};

#endif
