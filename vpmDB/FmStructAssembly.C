// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmStructAssembly.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <cstdio>


Fmd_SOURCE_INIT(FcSTRUCT_ASSEMBLY, FmStructAssembly, FmSubAssembly);


std::ostream& FmStructAssembly::writeFMF(std::ostream& os)
{
  os <<"STRUCT_ASSEMBLY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,*this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmStructAssembly::readAndConnect(std::istream& is, std::ostream&)
{
  FmStructAssembly* obj = new FmStructAssembly();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "
                << obj->getIdString() << std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}
