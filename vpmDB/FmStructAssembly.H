// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_STRUCT_ASSEMBLY_H
#define FM_STRUCT_ASSEMBLY_H

#include "vpmDB/FmAssemblyBase.H"


class FmStructAssembly : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  FmStructAssembly() {}
  virtual ~FmStructAssembly() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual const char* getUITypeName() const { return "Structure Assembly"; }
  virtual bool isListable(bool resultView) const { return !resultView; }
};

#endif
