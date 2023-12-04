// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>

#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmTransFriction.H"


Fmd_DB_SOURCE_INIT(FcTRANS_FRICTION, FmTransFriction, FmFrictionBase);


FmTransFriction::FmTransFriction() : FmFrictionBase()
{
  Fmd_CONSTRUCTOR_INIT(FmTransFriction);
}


void FmTransFriction::getParameters(std::vector<FmParameter>& retArray) const
{
  M_APPEND_PARAMS("Force caused by prestress", PrestressLoad,
		  FmFrictionBase, retArray);

  this->FmFrictionBase::getParameters(retArray);
}


std::ostream& FmTransFriction::writeFMF(std::ostream& os)
{
  os <<"TRANS_FRICTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmTransFriction::readAndConnect(std::istream& is, std::ostream&)
{
  FmTransFriction* obj = new FmTransFriction();

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


bool FmTransFriction::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmTransFriction::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmTransFriction::getClassTypeID());
}
