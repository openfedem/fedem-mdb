// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SOIL_PILE_H
#define FM_SOIL_PILE_H

#include "vpmDB/FmAssemblyBase.H"


class FmSoilPile : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  typedef std::pair<int,int> Ints;

  FmSoilPile(bool isDummy = false);
  virtual ~FmSoilPile() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Soil Pile"; }
  virtual const char** getListViewPixmap() const;

public:
  FFaField<bool>   internalSoil;
  FFaField<double> soilDensity;
  FFaField<int>    visualize3Dts;
  FFaField<Ints>   visualize3DAngles;
};

#endif
