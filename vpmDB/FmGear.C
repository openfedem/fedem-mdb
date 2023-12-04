// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdHP.H"
#endif

#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmGear.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcGEAR, FmGear, FmHPBase);

FmGear::FmGear()
{
  Fmd_CONSTRUCTOR_INIT(FmGear);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdHP(this);
#endif

  FFA_REFERENCE_FIELD_INIT(itsOutputJointField, itsOutputJoint, "OUTPUT_JOINT");
}


FmGear::~FmGear()
{
  this->disconnect();
  this->removeItsOutputJoint();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmGear::connect(FmRevJoint* inputJnt, FmRevJoint* outputJnt)
{
  bool status = this->mainConnect();

  this->setInputJoint(inputJnt);
  this->setOutputJoint(outputJnt);

  return status;
}


FmJointBase* FmGear::getOutputJoint() const
{
  return itsOutputJoint;
}


bool FmGear::setOutputJoint(FmJointBase* jnt)
{
  if (!jnt->isOfType(FmRevJoint::getClassTypeID()))
    return false;

  if (jnt->hasHPConnections())
    return false; // the joint already has a higher pair connection

  itsOutputJoint = jnt;
  return true;
}


void FmGear::removeItsOutputJoint()
{
  itsOutputJoint = 0;
}


std::ostream& FmGear::writeFMF(std::ostream& os)
{
  os <<"GEAR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmGear::readAndConnect(std::istream& is, std::ostream&)
{
  FmGear* obj = new FmGear();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmGear::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmGear::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmGear::getClassTypeID());
}


bool FmGear::localParse(const char* keyWord, std::istream& activeStatement,
                        FmGear* obj)
{
  bool retVal;
  if (strcmp(keyWord,"OUTPUT_REV_JOINT") == 0)
  {
    // For old model file compatibility (R4.2.1 and earlier)
    retVal = parentParse("OUTPUT_JOINT", activeStatement, obj);
    if (obj->itsOutputJoint.getRefID() > 0)
      obj->itsOutputJoint.setRef(obj->itsOutputJoint.getRefID(),
				 FmRevJoint::getClassTypeID());
  }
  else
    retVal = parentParse(keyWord, activeStatement, obj);

  return retVal;
}
