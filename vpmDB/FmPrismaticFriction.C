// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmPrismaticFriction.H"
#include "vpmDB/FuncPixmaps/prismJntFric.xpm"


Fmd_DB_SOURCE_INIT(FcPRISMATIC_FRICTION, FmPrismaticFriction, FmTransFriction);


FmPrismaticFriction::FmPrismaticFriction() : FmTransFriction()
{
  Fmd_CONSTRUCTOR_INIT(FmPrismaticFriction);

  FFA_FIELD_INIT(myRadius  , 1.0, "LOCING_DEVICE_RADIUS");
  FFA_FIELD_INIT(myConstant, 0.0, "BEARING_CONSTANT");
}


const char** FmPrismaticFriction::getPixmap() const
{
  return prismJntFric;
}


void FmPrismaticFriction::getParameters(std::vector<FmParameter>& retArray) const
{
  M_APPEND_PARAMS("Distance to locking device, R", Radius,
		  FmPrismaticFriction, retArray);
  M_APPEND_PARAMS("Bearing constant, Y", Constant,
		  FmPrismaticFriction, retArray);

  this->FmTransFriction::getParameters(retArray);
}


void FmPrismaticFriction::getTypeDepVars(std::vector<double>& vars) const
{
  vars = { myRadius.getValue(), myConstant.getValue(), 0.0 };
}


std::ostream& FmPrismaticFriction::writeFMF(std::ostream& os)
{
  os <<"PRISMATIC_FRICTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmPrismaticFriction::readAndConnect(std::istream& is, std::ostream&)
{
  FmPrismaticFriction* obj = new FmPrismaticFriction();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"VAR_1") == 0)
	FmFrictionBase::localParse("LOCING_DEVICE_RADIUS", activeStatement, obj);
      else if (strcmp(keyWord,"VAR_3") == 0)
	FmFrictionBase::localParse("BEARING_CONSTANT", activeStatement, obj);
      else
	FmFrictionBase::localParse(keyWord, activeStatement, obj);
    }
  }

  obj->connect();
  return true;
}
