// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmJacket.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <cstdio>


Fmd_SOURCE_INIT(FcJACKET, FmJacket, FmSubAssembly);


FmJacket::FmJacket(bool isDummy) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(waterFilled, true, "WATER_FILLED");

  FFA_FIELD_INIT(visualize3Dts, 1, "VISUALIZE3D");
}


const char** FmJacket::getListViewPixmap() const
{
  return jacket_xpm;
}


std::ostream& FmJacket::writeFMF(std::ostream& os)
{
  os <<"JACKET\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,*this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmJacket::readAndConnect(std::istream& is, std::ostream&)
{
  FmJacket* obj = new FmJacket();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This jacket assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "
                << obj->getIdString() << std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}
