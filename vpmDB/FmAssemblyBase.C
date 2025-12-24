// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmAssemblyBase.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmDB.H"


FmAssemblyBase::FmAssemblyBase(bool isDummy) : FmSubAssembly(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_DEFAULT_INIT(myCS,"COORDINATE_SYSTEM");
  FFA_FIELD_DEFAULT_INIT(myLocation,"LOCATION3D_DATA");
}


FmAssemblyBase* FmAssemblyBase::getPositionedParent() const
{
  return dynamic_cast<FmAssemblyBase*>(this->getParentAssembly());
}


void FmAssemblyBase::setLocalCS(const FaMat34& cs, bool updateLoc)
{
  // If the sub-assembly contains grounded Triads, they have to be moved
  // explicitly, since their local CS is relative to the fixed Earth link
  FaMat34 TrMat = cs * myCS.getValue().inverse();
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this);
  for (size_t i = 0; i < triads.size(); i++)
    if (triads[i]->isAttached(FmDB::getEarthLink()))
      triads[i]->setGlobalCS(TrMat*triads[i]->getGlobalCS());

  myCS.setValue(cs);
  if (updateLoc)
    this->updateLocation();

  FmDB::displayAll(this->getHeadMap());
}


void FmAssemblyBase::setGlobalCS(const FaMat34& cs, bool updateLoc)
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
    this->setLocalCS(parent->toLocal(cs),updateLoc);
  else
    this->setLocalCS(cs,updateLoc);
}


FaMat34 FmAssemblyBase::toGlobal(const FaMat34& m) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
    return parent->toGlobal(myCS.getValue()) * m;
  else
    return myCS.getValue() * m;
}


FaMat34 FmAssemblyBase::toLocal(const FaMat34& m) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
    return parent->toGlobal(myCS.getValue()).inverse() * m;
  else
    return myCS.getValue().inverse() * m;
}


FaMat33 FmAssemblyBase::toGlobal(const FaMat33& m) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
    return parent->toGlobal(myCS.getValue().direction()) * m;
  else
    return myCS.getValue().direction() * m;
}


FaMat33 FmAssemblyBase::toLocal(const FaMat33& m) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
    return parent->toGlobal(myCS.getValue().direction()).transpose() * m;
  else
    return myCS.getValue().direction().transpose() * m;
}


FaVec3 FmAssemblyBase::toGlobal(const FaVec3& v, bool directionOnly) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
  {
    if (directionOnly)
      return parent->toGlobal(myCS.getValue().direction()) * v;
    else
      return parent->toGlobal(myCS.getValue()) * v;
  }

  if (directionOnly)
    return myCS.getValue().direction() * v;
  else
    return myCS.getValue() * v;
}


FaVec3 FmAssemblyBase::toLocal(const FaVec3& v, bool directionOnly) const
{
  FmAssemblyBase* parent = this->getPositionedParent();
  if (parent)
  {
    if (directionOnly)
      return parent->toGlobal(myCS.getValue().direction()).transpose() * v;
    else
      return parent->toGlobal(myCS.getValue()).inverse() * v;
  }

  if (directionOnly)
    return myCS.getValue().direction().transpose() * v;
  else
    return myCS.getValue().inverse() * v;
}


void FmAssemblyBase::setLocation(const FFa3DLocation& loc)
{
  myLocation = loc;

  this->setGlobalCS(myLocation.getValue().getMatrix(),false);
  this->updateLocation('M'); // update the members of this assembly
}


void FmAssemblyBase::updateLocation(char updateWhat)
{
  if (updateWhat == 'A' || updateWhat == 'T')
    myLocation.getValue().set(myLocation.getValue().getPosType(),
			      myLocation.getValue().getRotType(),
			      this->toGlobal(FaMat34()));

  if (updateWhat != 'A' && updateWhat != 'M') return;

  // Update the locations of all model members of this assembly
  const std::map<int,FmRingStart*>* root = this->getHeadMap();
  std::map<int,FmRingStart*>::const_iterator it;
  for (it = root->begin(); it != root->end(); it++)
  {
    FmBase* runner = it->second->getNext();
    if (runner->isOfType(FmIsPositionedBase::getClassTypeID()))
      while (runner != it->second && runner != 0)
      {
	static_cast<FmIsPositionedBase*>(runner)->updateLocation();
	runner = runner->getNext();
      }
    else if (runner->isOfType(FmSubAssembly::getClassTypeID()))
      while (runner != it->second && runner != 0)
      {
	static_cast<FmSubAssembly*>(runner)->updateLocation();
	runner = runner->getNext();
      }
  }
}


bool FmAssemblyBase::isMovable() const
{
  // An assembly can be moved only if none of its or Triads or Joints
  // are connected to objects in other sub-assemblies
  // (except for child assemblies of this assembly)

  size_t i, j;
  FmLink* obj;
  std::vector<FmJointBase*> joints;
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this);
  for (i = 0; i < triads.size(); i++)
    for (j = 0; (obj = triads[i]->getOwnerLink(j)); j++)
      if (!obj->isPartOf(this) && !obj->isEarthLink())
	return false;
      else
      {
	std::vector<FmJointBase*> tmp;
	triads[i]->getJointBinding(tmp);
	joints.insert(joints.end(),tmp.begin(),tmp.end());
      }

  for (i = 0; i < joints.size(); i++)
    if (!joints[i]->isOfType(FmFreeJoint::getClassTypeID()) &&
	!joints[i]->isContactElement())
    {
      if ((obj = joints[i]->getSlaveLink()))
	if (!obj->isPartOf(this))
	  return false;
      if ((obj = joints[i]->getMasterLink()))
	if (!obj->isPartOf(this) && !obj->isEarthLink())
	  return false;
    }

  return true;
}


double FmAssemblyBase::getTotalLength() const
{
  std::vector<FmBeam*> beams;
  FmDB::getAllBeams(beams,this);

  // Sum length of all beams
  double length = 0.0;
  for (size_t i = 0; i < beams.size(); i++)
    length += beams[i]->getLength();

  return length;
}


double FmAssemblyBase::getTotalMass(double* pLength) const
{
  size_t i;
  double mass = 0.0;

  // Sum mass of all beams
  std::vector<FmBeam*> beams;
  FmDB::getAllBeams(beams,this);
  for (i = 0; i < beams.size(); i++)
  {
    mass += beams[i]->getMass();
    if (pLength != NULL)
      *pLength += beams[i]->getLength();
  }

  if (pLength)
  {
    // Sum length of all beams
    *pLength = 0.0;
    for (i = 0; i < beams.size(); i++)
      *pLength += beams[i]->getLength();
  }

  // Add mass of all parts
  std::vector<FmPart*> parts;
  FmDB::getAllParts(parts,this);
  for (i = 0; i < parts.size(); i++)
    mass += parts[i]->getMass();

  // Add mass of all triads
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this);
  for (i = 0; i < triads.size(); i++)
    mass += triads[i]->getAddMass(-1);

  return mass;
}


FaVec3 FmAssemblyBase::getGlobalCoG(bool) const
{
  FaVec3 mCoG;
  this->getMass(mCoG);
  return mCoG;
}


double FmAssemblyBase::getMass(FaVec3& mCoG, bool includeSubAss) const
{
  size_t i;
  double mass = 0.0;

  // Sum mass and mass*CoG for all links
  std::vector<FmLink*> links;
  FmDB::getAllLinks(links,this,!includeSubAss);
  for (i = 0; i < links.size(); i++)
  {
    double m = links[i]->getMass();
    mCoG += m*links[i]->getPositionCG().translation();
    mass += m;
  }

  // Add mass and mass*CoG for all triads
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this,!includeSubAss);
  for (i = 0; i < triads.size(); i++)
  {
    double m = triads[i]->getAddMass(-1);
    mCoG += m*triads[i]->getGlobalTranslation();
    mass += m;
  }

  if (mass != 0.0)
    mCoG /= mass;

  return mass;
}


FaVec3 FmAssemblyBase::getExtents() const
{
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this);
  if (triads.size() < 2)
    return FaVec3();

  FaVec3 min(triads.front()->getGlobalTranslation());
  FaVec3 max(min);
  for (size_t t = 1; t < triads.size(); t++)
  {
    FaVec3 pos = triads[t]->getGlobalTranslation();
    for (int i = 0; i < 3; i++)
      if (pos[i] < min[i])
        min[i] = pos[i];
      else if (pos[i] > max[i])
        max[i] = pos[i];
  }

  return max - min;
}


FmBase* FmAssemblyBase::duplicate() const
{
  FmSubAssembly* subass = this->duplicate(NULL);
  FmAssemblyBase* pAss = dynamic_cast<FmAssemblyBase*>(subass);
  if (pAss)
  {
    FaMat34 newCS = pAss->myCS.getValue();
    newCS[VW] += 0.2*pAss->getExtents();
    pAss->setLocalCS(newCS);
  }

  return subass;
}
