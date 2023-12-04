// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmBearingFriction.H"
#include "vpmDB/FuncPixmaps/revJntFric.xpm"


Fmd_DB_SOURCE_INIT(FcBEARING_FRICTION, FmBearingFriction, FmRotFriction);


FmBearingFriction::FmBearingFriction() : FmRotFriction()
{
  Fmd_CONSTRUCTOR_INIT(FmBearingFriction);

  FFA_FIELD_INIT(myCapacity, 0.1, "BENDING_CAPACITY_DISTANCE");
  FFA_FIELD_INIT(myConstant, 0.0, "BEARING_CONSTANT");
}


const char** FmBearingFriction::getPixmap() const
{
  return revJntFric;
}


void FmBearingFriction::getTypeDepVars(std::vector<double>& vars) const
{
  vars = {
    myRadius.getValue(),
    myCapacity.getValue(),
    myConstant.getValue()
  };
}


void FmBearingFriction::getParameters(std::vector<FmParameter>& retArray) const
{
  M_APPEND_PARAMS("Bearing constant, Y", Constant,
		  FmBearingFriction, retArray);
  M_APPEND_PARAMS("Bending capacity distance, a", Capacity,
		  FmBearingFriction, retArray);

  this->FmRotFriction::getParameters(retArray);
}


std::ostream& FmBearingFriction::writeFMF(std::ostream& os)
{
  os <<"BEARING_FRICTION\n{" << std::endl;
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmBearingFriction::readAndConnect(std::istream& is, std::ostream&)
{
  FmBearingFriction* obj = new FmBearingFriction();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"VAR_1") == 0)
	FmFrictionBase::localParse("CONTACT_RADIUS", activeStatement, obj);
      else if (strcmp(keyWord,"VAR_2") == 0)
	FmFrictionBase::localParse("BENDING_CAPACITY_DISTANCE", activeStatement, obj);
      else if (strcmp(keyWord,"VAR_3") == 0)
	FmFrictionBase::localParse("BEARING_CONSTANT", activeStatement, obj);
      else
	FmFrictionBase::localParse(keyWord, activeStatement, obj);
    }
  }

  obj->connect();
  return true;
}
