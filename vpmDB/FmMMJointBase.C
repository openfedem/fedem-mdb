// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmStraightMaster.H"
#include "vpmDB/FmArcSegmentMaster.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcMM_JOINT_BASE, FmMMJointBase, FmJointBase);

Fm1DMaster* FmMMJointBase::editedMaster = NULL;

std::vector<Fm1DMaster*> FmMMJointBase::tmpMasters;


FmMMJointBase::FmMMJointBase()
{
  Fmd_CONSTRUCTOR_INIT(FmMMJointBase);

  FFA_REFERENCE_FIELD_INIT(myMasterField, myMaster, "MASTER");
}


FmMMJointBase::~FmMMJointBase()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmMMJointBase::eraseOptions()
{
  FmTriad* triad = this->getSlaveTriad();
  this->removeItsSlaveTriad();

  if (triad)
  {
    if (!triad->hasReferences())
      triad->erase();
    else
      triad->updateTopologyInViewer();
  }

  Fm1DMaster* line = myMaster.getPointer();
  myMaster.setRef(NULL);

  if (line)
  {
    FmMMJointBase* other = NULL;
    if (line->hasReferringObjs(other))
      line->updateTopologyInViewer();
    else
      line->erase();
  }

  return this->FmJointBase::eraseOptions();
}


bool FmMMJointBase::isLooping() const
{
  FmArcSegmentMaster* master = dynamic_cast<FmArcSegmentMaster*>(myMaster.getPointer());
  return master ? master->isLooping() : false;
}


bool FmMMJointBase::isMasterAttachedToLink(bool allowMulipleLinks) const
{
  return myMaster.isNull() ? false : myMaster->isAttached(allowMulipleLinks);
}


FmLink* FmMMJointBase::getMasterLink() const
{
  return myMaster.isNull() ? NULL : myMaster->getOwnerLink();
}


FmPart* FmMMJointBase::getMasterPart(bool noEarth) const
{
  if (myMaster.isNull()) return NULL;

  FmPart* masterPart = myMaster->getOwnerPart();
  if (!masterPart) return NULL;

  return noEarth && masterPart->isEarthLink() ? NULL : masterPart;
}


FaVec3 FmMMJointBase::getTransJointVariables() const
{
  FmTriad* first = this->getFirstMaster();
  FmTriad* triad = this->getSlaveTriad();
  if (first && triad)
    return first->getGlobalCS().inverse()*triad->getGlobalTranslation();
  else if (first)
    return first->getGlobalCS().inverse().translation();
  else if (triad)
    return triad->getGlobalTranslation();
  else
    return FaVec3();
}


FaVec3 FmMMJointBase::getRotJointVariables() const
{
  FmTriad* first = this->getFirstMaster();
  FmTriad* triad = this->getSlaveTriad();
  if (first && triad)
    return this->getJointRotations(first->getGlobalCS(),triad->getGlobalCS());
  else if (first)
    return this->getJointRotations(first->getGlobalCS(),FaMat34());
  else if (triad)
    return this->getJointRotations(FaMat34(),triad->getGlobalCS());
  else
    return FaVec3();
}


void FmMMJointBase::setRotJointVariables(const FaVec3& rotations)
{
  FmTriad* first = this->getFirstMaster();
  if (first)
    this->setJointRotations(rotations,first->getGlobalCS());
  else
    this->setJointRotations(rotations,FaMat34());
}


bool FmMMJointBase::detach()
{
  // This detaches only the independent joint triads, because it looks best ;-)
  if (!this->isMasterAttachedToLink(true))
    ListUI <<"Error : The independent triads of "<< this->getIdString()
	   <<" are already detached.\n";
  else if (!myMaster->detach())
    ListUI <<"Error : Could not detach "<< this->getIdString() <<".\n";
  else
    return this->draw();

  return false;
}


FmTriad* FmMMJointBase::getFirstMaster() const
{
  return myMaster.isNull() ? NULL : myMaster->getFirstTriad();
}


FmTriad* FmMMJointBase::getLastMaster() const
{
  return myMaster.isNull() ? NULL : myMaster->getLastTriad();
}


bool FmMMJointBase::addMasterTriad(const FaVec3& globPoint)
{
  return editedMaster ? editedMaster->addTriadOnPoint(globPoint) : false;
}


bool FmMMJointBase::addMasterOnPoint(const FaVec3& globPoint)
{
  return myMaster.isNull() ? false : myMaster->addTriadOnPoint(globPoint);
}


bool FmMMJointBase::addAsMasterTriad(FmTriad* triad)
{
  return myMaster.isNull() ? false : myMaster->addTriad(triad);
}


void FmMMJointBase::getMasterTriads(std::vector<FmTriad*>& triads) const
{
  if (!myMaster.isNull()) myMaster->getTriads(triads);
}


bool FmMMJointBase::isMasterTriad(const FmTriad* triad) const
{
  return myMaster.isNull() ? false : myMaster->hasTriad(triad);
}


bool FmMMJointBase::localParse(const char* keyWord, std::istream& activeStatement,
			       FmMMJointBase* obj)
{
  if (strcmp(keyWord,"MASTER_TRIADS"))
    return parentParse(keyWord, activeStatement, obj);

  // Conversion from old model files: Create a line or arc segment object
  // and assign independent triads referred by this joint to it
  Fm1DMaster* line = NULL;
  if (obj->isOfType(FmCamJoint::getClassTypeID()))
    line = new FmArcSegmentMaster();
  else
    line = new FmStraightMaster();

  int tid;
  std::vector<int> triadIDs;
  activeStatement >> tid;
  while (activeStatement)
  {
    triadIDs.push_back(tid);
    activeStatement >> tid;
  }
  line->setTriads(triadIDs);

  // Check if an identical object has been created previously by another joint
  for (Fm1DMaster* tmp : tmpMasters)
    if (*tmp == *line)
    {
      // Use the existing object instead and delete the new one
      line->erase();
      obj->setMaster(tmp);
      return false;
    }

  obj->setMaster(line);
  tmpMasters.push_back(line); // delay connect() to avoid base ID troubles

  return false;
}


void FmMMJointBase::connectTmpMasters()
{
  // Connect all line objects that were created
  // due to parsing of old (R5.0 and older) model file
  for (Fm1DMaster* line : tmpMasters)
    line->connect();

  tmpMasters.clear();
}


bool FmMMJointBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmMMJointBase::getClassTypeID());
}


bool FmMMJointBase::insertAsMaster(FmTriad* triad, size_t pos)
{
  return myMaster.isNull() ? false : myMaster->insertTriad(triad,pos);
}
