// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmRotFriction.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdSimpleJoint.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcBALL_JOINT, FmBallJoint, FmSMJointBase);


FmBallJoint::FmBallJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmBallJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdSimpleJoint(this);
#endif
  myLegalDOFs[X_ROT] = true;
  myLegalDOFs[Y_ROT] = true;
  myLegalDOFs[Z_ROT] = true;
  this->completeInitJVars();

  FFA_FIELD_INIT(myFrictionDof, X_ROT, "FRICTION_DOF");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

int FmBallJoint::getValidFrictionType() const
{
  return FmRotFriction::getClassTypeID();
}


bool FmBallJoint::setFrictionDof(int dof)
{
  if (dof < X_ROT || dof > Z_ROT) return false;

  myFrictionDof.setValue(dof);
  return true;
}


std::ostream& FmBallJoint::writeFMF(std::ostream& os)
{
  os <<"BALL_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmBallJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmBallJoint* obj = new FmBallJoint();

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


bool FmBallJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmBallJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmBallJoint::getClassTypeID());
}
