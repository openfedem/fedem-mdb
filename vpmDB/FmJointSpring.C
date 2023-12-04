// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(FcJOINT_SPRING, FmJointSpring, FmSpringBase);


FmJointSpring::FmJointSpring()
{
  Fmd_CONSTRUCTOR_INIT(FmJointSpring);

  // Remove the SAVE_VAR field inherited from FmIsPlottedBase,
  // since the joint spring variables are toggled by the owner joint
  this->removeField("SAVE_VAR");
}


FmJointSpring::~FmJointSpring()
{
  this->disconnect();
}


bool FmJointSpring::disconnect()
{
  bool status = this->mainDisconnect();

  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return status;

  ownerJoint->releaseSpringAtDOF(ownerJoint->atWhatDOF(this));
  return status;
}


FmModelMemberBase* FmJointSpring::getActiveOwner() const
{
  FmJointBase* owner = this->getOwnerJoint();
  if (owner && !this->isMeasured())
  {
    int dof = this->getDOF();
    if (owner->getStatusOfDOF(dof) < FmHasDOFsBase::SPRING_CONSTRAINED)
      return NULL;

    if (!this->getStiffFunction() && this->getInitStiff() == 0.0)
      return NULL;
  }

  return owner;
}


bool FmJointSpring::isMeasured() const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return false;

  // Quick exit if the owner joint has no simple sensor
  FmSensorBase* sensor = ownerJoint->getSimpleSensor();
  if (!sensor) return false;

  int thisDof = this->getDOF();

  // Get all engines using this sensor, check if dof selection is to this spring
  std::vector<FmEngine*> engines;
  sensor->getEngines(engines);
  for (FmEngine* engine : engines)
  {
    size_t nArg = engine->getNoArgs();
    FmSensorBase* s = engine->getSensor();
    for (size_t j = 0; j < nArg; s = engine->getSensor(++j))
      if (s == sensor && engine->getDof(j) == thisDof)
        switch (engine->getEntity(j))
	  {
	  case FmIsMeasuredBase::JSPR_ANG:
	  case FmIsMeasuredBase::JSPR_DEFL:
	  case FmIsMeasuredBase::JSPR_FORCE:
	    return true;
	  }
  }

  return false;
}


double FmJointSpring::getModelSpringLength() const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return 0.0;

  return ownerJoint->getJointVariable(this->getDOF());
}


int FmJointSpring::getDOF() const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return 0;

  return ownerJoint->atWhatDOF((FmJointSpring*)this);
}


FmJointBase* FmJointSpring::getOwnerJoint() const
{
  FmJointBase* owner = NULL;
  if (this->hasReferringObjs(owner)) // there should only be one
    return owner;
  else
    return NULL;
}


std::ostream& FmJointSpring::writeFMF(std::ostream& os)
{
  os <<"JOINT_SPRING\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmJointSpring::readAndConnect(std::istream& is, std::ostream&)
{
  FmJointSpring* obj = new FmJointSpring();

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


bool FmJointSpring::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmJointSpring::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmJointSpring::getClassTypeID());
}


int FmJointSpring::checkJointSprings()
{
  int errCount = 0;
  std::vector<FmJointSpring*> allSprings;
  FmDB::getAllJointSprings(allSprings);
  for (FmJointSpring* spring : allSprings)
    if (!spring->getOwnerJoint())
    {
      errCount++;
      ListUI <<"ERROR: "<< spring->getIdString()
             <<" is not attached to a joint.\n";
    }
    else if (!spring->getOwnerJoint()->isLegalDOF(spring->getDOF()))
    {
      errCount++;
      ListUI <<"ERROR: "<< spring->getIdString()
             <<" is attached to an illegal joint DOF.\n";
    }

  return errCount;
}


bool FmJointSpring::getSaveVar(unsigned int& nVar, IntVec& toggles) const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return false;

  size_t nJSVar = ownerJoint->mySaveVar.getValue().size();
  if (nJSVar < 5+nVar)
    nVar = nJSVar > 5 ? nJSVar - 5 : 0;
  for (unsigned int i = 0; i < nVar; i++)
    toggles[i] = ownerJoint->mySaveVar.getValue()[5+i] ? 1 : 0;

  return true;
}
