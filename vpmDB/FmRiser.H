// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RISER_H
#define FM_RISER_H

#include "vpmDB/FmAssemblyBase.H"


class FmRiser : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  typedef std::pair<int,int> Ints;

  FmRiser(bool isDummy = false);
  virtual ~FmRiser() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Beamstring"; }
  virtual const char** getListViewPixmap() const;

  static bool stitch(FmModelMemberBase* beam1,
                     FmModelMemberBase* beam2,
                     FmModelMemberBase* contactSpring,
                     bool interconnectXY, double tolDist = 0.1);
  static bool split(FmModelMemberBase* beam1, FmModelMemberBase* beam2);

public:
  FFaField<bool>   internalMud;
  FFaField<double> mudDensity;
  FFaField<double> mudLevel;
  FFaField<int>    visualize3Dts;
  FFaField<Ints>   visualize3DAngles;
};

#endif
