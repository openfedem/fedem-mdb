// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRackPinion.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmRevJoint.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdHP.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcRACK_PINION, FmRackPinion, FmHPBase);

FmRackPinion::FmRackPinion()
{
  Fmd_CONSTRUCTOR_INIT(FmRackPinion);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdHP(this);
#endif

  FFA_REFERENCE_FIELD_INIT(itsOutputJointField, itsOutputJoint, "OUTPUT_JOINT");
}


FmRackPinion::~FmRackPinion()
{
  disconnect();
  removeItsOutputJoint();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmRackPinion::connect(FmRevJoint* inputJnt, FmPrismJoint* outputJnt)
{
  bool status = mainConnect();

  setInputJoint(inputJnt);
  setOutputJoint(outputJnt);

  return status;
}


FmJointBase* FmRackPinion::getOutputJoint() const
{
  return itsOutputJoint;
}


bool FmRackPinion::setOutputJoint(FmJointBase* jnt)
{
  if (!jnt->isOfType(FmPrismJoint::getClassTypeID()))
    return false;

  if (jnt->hasHPConnections())
    return false; // the joint already has a higher pair connection

  itsOutputJoint = jnt;
  return true;
}


void FmRackPinion::removeItsOutputJoint()
{
  itsOutputJoint = 0;
}


std::ostream& FmRackPinion::writeFMF(std::ostream& os)
{
  os <<"RACK_PINION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRackPinion::readAndConnect(std::istream& is, std::ostream&)
{
  FmRackPinion* obj = new FmRackPinion();

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


bool FmRackPinion::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRackPinion::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRackPinion::getClassTypeID());
}


bool FmRackPinion::localParse(const char* keyWord, std::istream& activeStatement,
                              FmRackPinion* obj)
{
  bool retVal;
  if (strcmp(keyWord,"OUTPUT_PRISM_JOINT") == 0)
  {
    // For old model file compatibility (R4.2.1 and earlier)
    retVal = parentParse("OUTPUT_JOINT", activeStatement, obj);
    if (obj->itsOutputJoint.getRefID() > 0)
      obj->itsOutputJoint.setRef(obj->itsOutputJoint.getRefID(),
				 FmPrismJoint::getClassTypeID());
  }
  else
    retVal = parentParse(keyWord, activeStatement, obj);

  return retVal;
}
