// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODES_OPTIONS_H
#define FM_MODES_OPTIONS_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmVTFType.H"

typedef std::pair<int,double>   FmModeType; //!< mode number - time
typedef std::vector<FmModeType> FmModeVec;  //!< vector of mode shapes to expand


class FmModesOptions : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmModesOptions();

  virtual const char* getUITypeName() const { return "ModesOptions"; }

  bool setMaxEigenmode(int modeno);
  bool setMinTime(double minTime);
  bool setMaxTime(double maxTime);

  int addEigenmodeData(int modeNo, double time);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

protected:
  virtual ~FmModesOptions();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& is, FmModesOptions* obj);

private:
  bool convertedFromOldModelFile;

public:
  FFaField<FmModeVec>       modesArray;
  FFaField<bool>            autoVTFExport;
  FFaField<std::string>     vtfFileName;
  FFaField<VTFFileTypeEnum> vtfFileType;
  FFaField<std::string>     addOptions;
};

#endif
