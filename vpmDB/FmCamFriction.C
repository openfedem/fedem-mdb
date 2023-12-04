// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>

#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmCamFriction.H"


Fmd_DB_SOURCE_INIT(FcCAM_FRICTION, FmCamFriction, FmTransFriction);


FmCamFriction::FmCamFriction() : FmTransFriction()
{
  Fmd_CONSTRUCTOR_INIT(FmCamFriction);
}


std::ostream& FmCamFriction::writeFMF(std::ostream& os)
{
  os <<"CAM_FRICTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmCamFriction::readAndConnect(std::istream& is, std::ostream&)
{
  FmCamFriction* obj = new FmCamFriction();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmFrictionBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}
