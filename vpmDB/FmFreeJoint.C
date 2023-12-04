// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdFreeJoint.H"
#endif

#include "vpmDB/FmRotFriction.H"
#include "vpmDB/FmTransFriction.H"
#include "vpmDB/FmFreeJoint.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcFREE_JOINT, FmFreeJoint, FmSMJointBase);

FmFreeJoint::FmFreeJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmFreeJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdFreeJoint(this);
#endif
  myLegalDOFs[X_ROT] = true;
  myLegalDOFs[Y_ROT] = true;
  myLegalDOFs[Z_ROT] = true;
  myLegalDOFs[X_TRANS] = true;
  myLegalDOFs[Y_TRANS] = true;
  myLegalDOFs[Z_TRANS] = true;
  this->completeInitJVars();

  FFA_FIELD_INIT(myFrictionDof, X_TRANS, "FRICTION_DOF");

  IAmMovingSlaveTriadAlong = false;
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

int FmFreeJoint::getValidFrictionType() const
{
  return FmFrictionBase::getClassTypeID();
}


int FmFreeJoint::getValidFrictionType(int dof) const
{
  if (dof < X_TRANS || dof > Z_ROT) return -1;

  if (dof >= X_ROT)
    return FmRotFriction::getClassTypeID();
  else
    return FmTransFriction::getClassTypeID();
}


bool FmFreeJoint::setFrictionDof(int dof)
{
  if (dof < X_TRANS || dof > Z_ROT) return false;

  myFrictionDof.setValue(dof);
  return true;
}


std::ostream& FmFreeJoint::writeFMF(std::ostream& os)
{
  os <<"FREE_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmFreeJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmFreeJoint* obj = new FmFreeJoint();

  // Old model files without the DOFStatus field
  // should be initialized to SPRING_CONSTRAINED
  for (int i = 0; i < MAX_DOF; i++)
    if (obj->isLegalDOF(i))
      obj->myDofStatus[i] = SPRING_CONSTRAINED;

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


bool FmFreeJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmFreeJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmFreeJoint::getClassTypeID());
}
