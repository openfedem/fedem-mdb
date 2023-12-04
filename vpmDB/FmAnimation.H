// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ANIMATION_H
#define FM_ANIMATION_H

#include "vpmDB/FmResultBase.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaString/FFaEnum.H"
#include "FFaLib/FFaContainers/FFaReference.H"

typedef std::pair<double,double> FmRange;


class FmAnimation : public FmResultBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmAnimation();

  bool isHistoryAnimation() const;
  std::string getFringeQuantity() const;

  // User-controlled animation variables

  FFaField<bool>                isModesAnimation;
  FFaField<bool>                isSummaryAnimation;

  FFaField<bool>                loadFringeData;
  FFaField<bool>                loadLineFringeData;
  FFaField<bool>                loadDeformationData;

  FFaField<bool>                autoExport;

  // Fringe setup

  FFaField<std::string>         fringeResultClass;
  FFaField<std::string>         fringeVariableName;
  FFaField<std::string>         fringeToScalarOpName;

  FFaField<bool>                resultSetSelectionByName;
  FFaField<std::string>         resSetSelectionOpName;
  FFaField<std::string>         resSetName;

  FFaField<std::string>         multiFaceAveragingOpName;

  // Fringe averaging setup

  FFaField<std::string>         averagingOpName;
  FFaField<std::string>         averagingItem;
  FFaField<double>              maxMembraneAngleToAverage;
  FFaField<bool>                averagingAcrossMaterials;
  FFaField<bool>                averagingAcrossProperties;
  FFaField<bool>                averagingAcrossElmTypes;

  // Time animation setup

  FFaField<bool>                usingTimeInterval;
  FFaField<FmRange>             timeRange;

  FFaField<bool>                makeFrameForMostFrequentResult;

  // Modes animation setup

  enum EigenModeType {
    SYSTEM_MODES,
    COMPONENT_MODES,
    FREE_FREE_REDUCED,
    NON_REDUCED
  };

private:
  FFaEnumMapping(EigenModeType) {
    FFaEnumEntry(SYSTEM_MODES,"SYSTEM_MODES");
    FFaEnumEntry(COMPONENT_MODES,"COMPONENT_MODES");
    FFaEnumEntry(FREE_FREE_REDUCED,"FREE_FREE_REDUCED");
    FFaEnumEntry(NON_REDUCED,"NON_REDUCED");
    FFaEnumEntryEnd;
  };

public:
  FFaField<EigenModeTypeEnum>   eigenmodeType;
  FFaReference<FmPart>          eigenmodePart;
  FFaField<FFaReferenceBase*>   eigenmodePartField;
  FFaField<double>              eigenmodeTime;
  FFaField<int>                 eigenmodeNr;
  FFaField<double>              eigenmodeAmplitude;
  FFaField<int>                 eigenmodeFramesPrCycle;

  FFaField<bool>                eigenmodeDurationUseTime;
  FFaField<bool>                eigenmodeDurationUseNCycles;
  FFaField<bool>                eigenmodeDurationUseUntilDamped;

  FFaField<double>              eigenmodeDurationTime;
  FFaField<int>                 eigenmodeDurationNCycles;
  FFaField<double>              eigenmodeDurationUntilDamped;

  // Animation control

  FFaReference<FmLink>          linkToFollow;
  FFaField<FFaReferenceBase*>   linkToFollowField;

  FFaField<bool>                showLinkMotion;
  FFaField<bool>                showTriadMotion;
  FFaField<bool>                showDeformation;
  FFaField<double>              deformationScale;

  FFaField<bool>                showFringes;
  FFaField<bool>                showLegend;

  // Fringe legend data

  FFaField<std::string>         legendMappingOpName;
  FFaField<FmRange>             legendRange;
  FFaField<std::string>         colorMappingOpName;
  FFaField<bool>                smoothLegend;

  FFaField<bool>                useLegendTickCount;
  FFaField<int>                 legendTickCount;
  FFaField<double>              legendTickSpacing;
  FFaField<bool>                legendTickSpacingIsPrDecade;

  // Standard stuff

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Animation"; }

protected:
  virtual ~FmAnimation();

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
