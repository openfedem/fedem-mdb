// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_JACKET_H
#define FM_JACKET_H

#include "vpmDB/FmAssemblyBase.H"


class FmJacket : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  FmJacket(bool isDummy = false);
  virtual ~FmJacket() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Jacket"; }
  virtual const char** getListViewPixmap() const;

public:
  FFaField<bool>   waterFilled;
  FFaField<int>    visualize3Dts;
};

#endif
