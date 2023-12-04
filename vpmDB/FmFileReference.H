// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_FILE_REFERENCE_H
#define FM_FILE_REFERENCE_H

#include "vpmDB/FmSimulationModelBase.H"

typedef std::pair< std::string,std::vector<std::string> > FmFileRefExt;


class FmFileReference : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmFileReference();

  virtual bool isListable() const { return true; }

  virtual const char* getUITypeName() const { return "File reference"; }

  virtual bool clone(FmBase* obj, int depth);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  static const std::vector<FmFileRefExt>& getExtensions();

protected:
  virtual ~FmFileReference();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<std::string> fileName;

private:
  static std::vector<FmFileRefExt> extensions;
};

#endif
