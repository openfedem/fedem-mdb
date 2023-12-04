// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_GENERIC_DB_OBJECT_H
#define FM_GENERIC_DB_OBJECT_H

#include "vpmDB/FmSimulationModelBase.H"


class FmGenericDBObject : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmGenericDBObject();

  virtual bool isListable() const { return true; }

  virtual const char* getUITypeName() const { return "Generic object"; }

  virtual bool clone(FmBase* obj, int depth);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

  virtual int printSolverEntry(FILE* fp);

protected:
  virtual ~FmGenericDBObject();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<std::string> objectType;
  FFaField<std::string> objectDefinition;
};

#endif
