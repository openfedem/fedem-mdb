// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstring>

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"

#include "vpmDB/FmRotFriction.H"


Fmd_DB_SOURCE_INIT(FcROT_FRICTION, FmRotFriction, FmFrictionBase);


FmRotFriction::FmRotFriction() : FmFrictionBase()
{
  Fmd_CONSTRUCTOR_INIT(FmRotFriction);

  FFA_FIELD_INIT(myRadius, 0.1, "CONTACT_RADIUS");
}


const char* FmRotFriction::getFrictionFsiName() const
{
  // Beta feature: Multi-dof ball-joint friction
  FFaString fDesc = this->getUserDescription();
  if (fDesc.hasSubString("#BALL_FRICTION"))
    if (fDesc.hasSubString("#BALL_FRICTION2"))
      return "BALL_FRICTION2";
    else
      return "BALL_FRICTION";
  else
    return "ROT_FRICTION";
}


void FmRotFriction::getTypeDepVars(std::vector<double>& vars) const
{
  vars.resize(1,myRadius.getValue());
}


void FmRotFriction::getParameters(std::vector<FmParameter>& retArray) const
{
  M_APPEND_PARAMS("Radius of contact surface, R", Radius,
		  FmRotFriction, retArray);
  M_APPEND_PARAMS("Torque caused by prestress", PrestressLoad,
		  FmFrictionBase, retArray);

  this->FmFrictionBase::getParameters(retArray);
}


std::ostream& FmRotFriction::writeFMF(std::ostream& os)
{
  os <<"ROT_FRICTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRotFriction::readAndConnect(std::istream& is, std::ostream&)
{
  FmRotFriction* obj = new FmRotFriction();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"VAR_1") == 0)
	FmFrictionBase::localParse("CONTACT_RADIUS", activeStatement, obj);
      else
	FmFrictionBase::localParse(keyWord, activeStatement, obj);
    }
  }

  obj->connect();
  return true;
}


bool FmRotFriction::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRotFriction::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRotFriction::getClassTypeID());
}
