// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODEL_EXP_OPTIONS_H
#define FM_MODEL_EXP_OPTIONS_H

#include "vpmDB/FmModelMemberBase.H"


class FmModelExpOptions : public FmModelMemberBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmModelExpOptions();

  virtual const char* getUITypeName() const { return "ModelExportOptions"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  virtual ~FmModelExpOptions();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  //Stream
  FFaField<std::string> streamFilename;
  FFaField<std::string> streamInputIndGroup;
  FFaField<std::string> streamOutputIndGroup;
  FFaField<double>      streamWindowSize;
  FFaField<bool>        streamTransferState;
  FFaField<bool>        streamAppExport;
  //Batch
  FFaField<std::string> batchFilename;
  FFaField<std::string> batchInputIndGroup;
  FFaField<bool>        batchSurfaceOnly;
  FFaField<bool>        batchStressRecovery;
  FFaField<bool>        batchAllFEParts;
  FFaField<bool>        batchAppExport;
  //FMU
  FFaField<std::string> fmuFilename;
  FFaField<bool>        fmuAppExport;
};

#endif
