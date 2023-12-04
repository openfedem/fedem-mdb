// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmBearingFriction.H"
#include "vpmDB/FmJointMotion.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdRevJoint.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcREV_JOINT, FmRevJoint, FmSMJointBase);


FmRevJoint::FmRevJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmRevJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdRevJoint(this);
#endif
  myLegalDOFs[Z_TRANS] = true;
  myLegalDOFs[Z_ROT] = true;
  this->completeInitJVars();

  FFA_FIELD_INIT(hasTzDOF, false, "HAS_Z_TRANS_DOF");
  myLegalDOFs[Z_TRANS] = false;
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmRevJoint::setHasTzDOF(bool yesOrNo)
{
  hasTzDOF = myLegalDOFs[Z_TRANS] = yesOrNo;
}


FmDofMotion* FmRevJoint::getMotionAtDOF(int dof, bool createIfNone)
{
  FmDofMotion* oldMotion = myMotions[dof].getPointer();
  FmDofMotion* motion = this->FmSMJointBase::getMotionAtDOF(dof,createIfNone);
  if (motion != oldMotion && motion != NULL && dof == Z_TRANS)
    // A new motion object for the Z-translation DOF was created.
    // Set initial deflection to zero, in case the master- and slave triads
    // of the owner joint are not co-located
    static_cast<FmJointMotion*>(motion)->setInitLengthOrDefl(0.0,true);

  return motion;
}


int FmRevJoint::getValidFrictionType() const
{
  return FmRotFriction::getClassTypeID();
}


std::ostream& FmRevJoint::writeFMF(std::ostream& os)
{
  os <<"REV_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRevJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmRevJoint* obj = new FmRevJoint();

  // Old model files without the DOFStatus field
  // should be initialized to SPRING_CONSTRAINED
  for (int i = 0; i < MAX_DOF; i++)
    if (obj->isLegalDOF(i))
      obj->myDofStatus[i] = SPRING_CONSTRAINED;

  // Obsolete field
  FFaObsoleteField< std::vector<int> > jointDofs;
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(jointDofs,"JOINT_DOFS",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("JOINT_DOFS",obj);

  if (jointDofs.getValue().size() > Z_TRANS)
    obj->setHasTzDOF(jointDofs.getValue()[Z_TRANS] > 0);
  else if (obj->hasTzDOF.getValue())
    obj->myLegalDOFs[Z_TRANS] = true;

  // Correct friction type when reading old model files
  int fricID = obj->myFriction.getRefID();
  if (fricID > 0 && obj->myFriction.getRefTypeID() < 0)
    obj->myFriction.setRef(fricID,FmBearingFriction::getClassTypeID());

  obj->connect();
  return true;
}


bool FmRevJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRevJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRevJoint::getClassTypeID());
}
