// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmPrismaticFriction.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdPrismJoint.H"
#endif


Fmd_DB_SOURCE_INIT(FcPRISM_JOINT, FmPrismJoint, FmMMJointBase);


FmPrismJoint::FmPrismJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmPrismJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdPrismJoint(this);
#endif
  myLegalDOFs[Z_TRANS] = true;
  this->completeInitJVars();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

int FmPrismJoint::getValidFrictionType() const
{
  return FmTransFriction::getClassTypeID();
}


std::ostream& FmPrismJoint::writeFMF(std::ostream& os)
{
  os <<"PRISM_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmPrismJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmPrismJoint* obj = new FmPrismJoint();

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

  // Correct friction type when reading old model files
  int fricID = obj->myFriction.getRefID();
  if (fricID > 0 && obj->myFriction.getRefTypeID() < 0)
    obj->myFriction.setRef(fricID,FmPrismaticFriction::getClassTypeID());

  obj->connect();
  return true;
}


bool FmPrismJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmPrismJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmPrismJoint::getClassTypeID());
}
