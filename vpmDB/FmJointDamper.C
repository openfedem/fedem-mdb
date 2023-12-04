// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(FcJOINT_DAMPER, FmJointDamper, FmDamperBase);


FmJointDamper::FmJointDamper()
{
  Fmd_CONSTRUCTOR_INIT(FmJointDamper);

  // Remove the SAVE_VAR field inherited from FmIsPlottedBase,
  // since the joint spring variables are toggled by the owner joint
  this->removeField("SAVE_VAR");
}


FmJointDamper::~FmJointDamper()
{
  this->disconnect();
}


bool FmJointDamper::disconnect()
{
  bool status = this->mainDisconnect();

  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return status;

  ownerJoint->releaseDamperAtDOF(ownerJoint->atWhatDOF(this));
  return status;
}


FmModelMemberBase* FmJointDamper::getActiveOwner() const
{
  FmJointBase* owner = this->getOwnerJoint();
  if (owner && !this->isMeasured())
  {
    int dof = this->getDOF();
    if (owner->getStatusOfDOF(dof) < FmHasDOFsBase::SPRING_CONSTRAINED)
      return NULL;

    if (!this->getFunction() && this->getInitDamp() == 0.0)
      return NULL;

    // If the owner joint is a global spring element this damper is not used
    if (owner->isGlobalSpringElement())
      return NULL;
  }

  return owner;
}


bool FmJointDamper::isMeasured() const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return false;

  // Quick exit if the owner joint has no simple sensor
  FmSensorBase* sensor = ownerJoint->getSimpleSensor();
  if (!sensor) return false;

  int thisDof = this->getDOF();

  // Get all engines using this sensor, check if dof selection is to this damper
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
	  case FmIsMeasuredBase::JDAMP_ANG:
	  case FmIsMeasuredBase::JDAMP_VEL:
	  case FmIsMeasuredBase::JDAMP_FORCE:
	    return true;
	  }
  }

  return false;
}


int FmJointDamper::getDOF() const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return 0;

  return ownerJoint->atWhatDOF((FmJointDamper*)this);
}


FmJointBase* FmJointDamper::getOwnerJoint() const
{
  FmJointBase* owner = NULL;
  if (this->hasReferringObjs(owner)) // there should only be one
    return owner;
  else
    return NULL;
}


std::ostream& FmJointDamper::writeFMF(std::ostream& os)
{
  os <<"JOINT_DAMPER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmJointDamper::readAndConnect(std::istream& is, std::ostream&)
{
  FmJointDamper* obj = new FmJointDamper();

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


bool FmJointDamper::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmJointDamper::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmJointDamper::getClassTypeID());
}


int FmJointDamper::checkJointDampers()
{
  int errCount = 0;
  std::vector<FmJointDamper*> allDampers;
  FmDB::getAllJointDampers(allDampers);
  for (FmJointDamper* damper : allDampers)
    if (!damper->getOwnerJoint())
    {
      errCount++;
      ListUI <<"ERROR: "<< damper->getIdString()
             <<" is not attached to a joint.\n";
    }
    else if (!damper->getOwnerJoint()->isLegalDOF(damper->getDOF()))
    {
      errCount++;
      ListUI <<"ERROR: "<< damper->getIdString()
             <<" is attached to an illegal joint DOF.\n";
    }

  return errCount;
}


int FmJointDamper::printSolverEntry(FILE* fp)
{
  if (!this->getActiveOwner())
    return 0; // joint dof is not SPRING_CONSTRAINED

  FmJointBase* activeJoint = this->getOwnerJoint();
  if (activeJoint->isSuppressed())
    return 0; // slave is suppressed

  if (activeJoint->isContactElement())
  {
    fprintf(fp,"! Contact element damper\n");
    fprintf(fp,"&DAMPER_BASE\n");
  }
  else
  {
    fprintf(fp,"! Joint damper\n");
    fprintf(fp,"&DAMPER\n");
  }
  this->printID(fp);
  return this->FmDamperBase::printSolverEntry(fp);
}


bool FmJointDamper::getSaveVar(unsigned int& nVar, IntVec& toggles) const
{
  FmJointBase* ownerJoint = this->getOwnerJoint();
  if (!ownerJoint) return false;

  size_t nJSVar = ownerJoint->mySaveVar.getValue().size();
  if (nJSVar < 10+nVar)
    nVar = nJSVar > 10 ? nJSVar - 10 : 0;
  for (unsigned int i = 0; i < nVar; i++)
    toggles[i] = ownerJoint->mySaveVar.getValue()[10+i] ? 1 : 0;

  return true;
}
