// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_GRAPH_H
#define FM_GRAPH_H

#include "vpmDB/FmArray.H"
#include "vpmDB/FmResultBase.H"

class FmCurveSet;
class FmColor;

typedef std::pair<FmColor,const char*> FmCurveColor;
typedef std::pair<double,double>       FmRange;


class FmGraph : public FmResultBase
{
  Fmd_DB_HEADER_INIT();

  typedef std::array<int,3> GFonts;

public:
  FmGraph(bool beamDiagram = false);

  void setTitle(const std::string& title) { myTitle.setValue(title); }
  const std::string& getTitle() const { return myTitle.getValue(); }

  void setSubTitle(const std::string& title) { mySubTitle.setValue(title); }
  const std::string& getSubTitle() const { return mySubTitle.getValue(); }

  void setXaxisLabel(const std::string& label) { myXaxisLabel.setValue(label); }
  const std::string& getXaxisLabel() const { return myXaxisLabel.getValue(); }

  void setYaxisLabel(const std::string& label) { myYaxisLabel.setValue(label); }
  const std::string& getYaxisLabel() const { return myYaxisLabel.getValue(); }

  void setFontSize(const GFonts& sizes) { myFontSize.setValue(sizes); }
  const GFonts& getFontSize() const { return myFontSize.getValue(); }

  void setGridType(int type) { myGridType.setValue(type); }
  int  getGridType() const { return myGridType.getValue(); }

  void setShowLegend(bool show) { myShowLegend.setValue(show); }
  bool getShowLegend() const { return myShowLegend.getValue(); }

  void setAutoScaleFlag(bool flag) { myAutoScaleFlag.setValue(flag); }
  bool getAutoScaleFlag() const { return myAutoScaleFlag.getValue(); }

  void setUseTimeRange(bool use) { myUseTimeRange.setValue(use); }
  bool getUseTimeRange() const { return myUseTimeRange.getValue(); }

  void setTimeRange(double min, double max);
  void getTimeRange(double& min, double& max) const;
  bool useTimeRange(FmRange& timeRange) const;

  void setXaxisScale(double min, double max);
  void setYaxisScale(double min, double max);

  void getXaxisScale(double& min, double& max) const;
  void getYaxisScale(double& min, double& max) const;

  const FmColor& getCurveDefaultColor() const;
  static std::vector<FmCurveColor>& getCurveDefaultColors();

  bool addCurveSet(FmCurveSet* curve);
  bool addCurveSets(const std::vector<FmCurveSet*>& curves);
  void removeAllCurves();

  void getCurveSets(std::vector<FmCurveSet*>& curves,
                    bool sortOnId = false) const;

  int  getCurveSetCount() const;
  bool hasCurveSets(int mode = -1) const;
  bool hasCurve(FmCurveSet* curve) const;

  bool isFuncPreview() const;
  bool isBeamDiagram() const { return myIsBeamDiagram.getValue(); }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const;

  //! \brief Returns the valid ASCII file extensions.
  static std::vector<std::string> asc() { return { "asc", "txt" }; }
  //! \brief Returns the valid DAC file extensions.
  static std::vector<std::string> dac() { return { "dac" }; }
  //! \brief Returns the valid RPC file extensions.
  static std::vector<std::string> rpc() { return { "rsp", "drv", "tim" }; }

protected:
  virtual ~FmGraph();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmGraph* obj);

  virtual void mainConnectedEvent();

private:
  FFaField<std::string> myTitle;
  FFaField<std::string> mySubTitle;
  FFaField<std::string> myXaxisLabel;
  FFaField<std::string> myYaxisLabel;
  FFaField<GFonts>      myFontSize;
  FFaField<int>         myGridType;
  FFaField<bool>        myShowLegend;
  FFaField<bool>        myAutoScaleFlag;
  FFaField<bool>        myUseTimeRange;
  FFaField<FmRange>     myTimeRange;
  FFaField<FmRange>     myXaxisRange;
  FFaField<FmRange>     myYaxisRange;
  FFaField<bool>        myIsBeamDiagram;
};

#endif
