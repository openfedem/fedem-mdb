// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSoilPile.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <cstdio>


Fmd_SOURCE_INIT(FcSOIL_PILE, FmSoilPile, FmSubAssembly);


FmSoilPile::FmSoilPile(bool isDummy) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(internalSoil, true, "SOIL_FILLED");
  FFA_FIELD_INIT(soilDensity, 0.0, "SOIL_DENSITY");

  FFA_FIELD_INIT(visualize3Dts, 1, "VISUALIZE3D");
  FFA_FIELD_INIT(visualize3DAngles, Ints(0,360), "VISUALIZE3D_ANGLES");
}


const char** FmSoilPile::getListViewPixmap() const
{
  return soilpile_xpm;
}


std::ostream& FmSoilPile::writeFMF(std::ostream& os)
{
  os <<"SOIL_PILE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmSoilPile::readAndConnect(std::istream& is, std::ostream&)
{
  FmSoilPile* obj = new FmSoilPile();

  // Obsolete fields
  FFaObsoleteField<int> startAngle, stopAngle;
  FFA_OBSOLETE_FIELD_INIT(startAngle,0,"VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_INIT(stopAngle,360,"VISUALIZE3D_STOP_ANGLE",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_STOP_ANGLE",obj);

  // Update from old model file
  if (startAngle.wasOnFile())
    obj->visualize3DAngles.getValue().first = startAngle.getValue();
  if (stopAngle.wasOnFile())
    obj->visualize3DAngles.getValue().second = stopAngle.getValue();

  if (!obj->connect())
    // This soil pile assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "
                << obj->getIdString() << std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}
