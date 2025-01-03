// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmJointMotion.H"
#include "vpmDB/FmJointBase.H"
#include "FFaLib/FFaString/FFaParse.H"


Fmd_DB_SOURCE_INIT(FcJOINT_MOTION, FmJointMotion, FmDofMotion);


FmJointMotion::FmJointMotion()
{
  Fmd_CONSTRUCTOR_INIT(FmJointMotion);

  FFA_FIELD_INIT(useInitDeflection, true, "USE_INIT_DEFLECTION");
}


FmJointBase* FmJointMotion::getOwnerJoint() const
{
  FmJointBase* owner = NULL;
  if (this->hasReferringObjs(owner)) // there should only be one
    return owner;
  else
    return NULL;
}


bool FmJointMotion::getInitLengthOrDefl(double& lenOrDefl) const
{
  lenOrDefl = myMotionVal.getValue();
  if (useInitDeflection.getValue())
  {
    FmJointBase* owner = this->getOwnerJoint();
    lenOrDefl = owner->getJointVariable(owner->atWhatDOF(this)) - lenOrDefl;
  }

  return useInitDeflection.getValue();
}


void FmJointMotion::setInitLengthOrDefl(double L0, bool isDeflection)
{
  // Override useInitDeflection for prescribed velocities and accelerations
  if (myType.getValue() > DEFLECTION) isDeflection = false;

  if (isDeflection)
  {
    FmJointBase* owner = this->getOwnerJoint();
    L0 = owner->getJointVariable(owner->atWhatDOF(this)) - L0;
#ifdef FM_DEBUG
    std::cout <<"FmJointMotion::setInitDeflection("<< this->getIdString()
              <<"): "<< L0 << std::endl;
#endif
  }

  myMotionVal.setValue(L0);
  useInitDeflection.setValue(isDeflection);
}


std::ostream& FmJointMotion::writeFMF(std::ostream& os)
{
  os <<"JOINT_MOTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmJointMotion::readAndConnect(std::istream& is, std::ostream&)
{
  FmJointMotion* obj = new FmJointMotion();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord,is,activeStatement,'=',';'))
      FmJointMotion::parentParse(keyWord,activeStatement,obj);
  }

  obj->connect();
  return true;
}


void FmJointMotion::initAfterResolve()
{
  // Override FmDofMotion::initAfterResolve() which applies to triad DOFs only
  this->FmIsControlledBase::initAfterResolve();
}


bool FmJointMotion::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmJointMotion::getClassTypeID());
}
