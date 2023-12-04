// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRigidJoint.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdSimpleJoint.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcRIGID_JOINT, FmRigidJoint, FmSMJointBase);


FmRigidJoint::FmRigidJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmRigidJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdSimpleJoint(this);
#endif
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

std::ostream& FmRigidJoint::writeFMF(std::ostream& os)
{
  os <<"RIGID_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRigidJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmRigidJoint* obj = new FmRigidJoint();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	parentParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmRigidJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRigidJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRigidJoint::getClassTypeID());
}
