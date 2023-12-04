// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmSMJointBase.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmLink.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdRefPlane.H"
#endif


Fmd_DB_SOURCE_INIT(FcREF_PLANE, FmRefPlane, FmIsPositionedBase);

FmRefPlane::FmRefPlane()
{
  Fmd_CONSTRUCTOR_INIT(FmRefPlane);

  FFA_FIELD_INIT(myRGBColor, FmColor(0.3f,0.3f,0.3f), "COLOR");
  FFA_FIELD_INIT(myTransparency, 0.65, "TRANSPARENCY");
  FFA_FIELD_INIT(myWidth, 1.0, "WIDTH");
  FFA_FIELD_INIT(myHeight, 1.0, "HEIGHT");

#ifdef USE_INVENTOR
  itsDisplayPt = new FdRefPlane(this);
#endif
}


FmRefPlane::~FmRefPlane()
{
  this->disconnect();
}


bool FmRefPlane::setRGBColor(const FmColor& col)
{
  if (col == myRGBColor.getValue()) return false;

  myRGBColor.setValue(col);
#ifdef USE_INVENTOR
  itsDisplayPt->updateFdApperance();
#endif
  return true;
}


bool FmRefPlane::setTransparency(double var)
{
  if (var == myTransparency.getValue()) return false;

  myTransparency.setValue(var);
#ifdef USE_INVENTOR
  itsDisplayPt->updateFdApperance();
#endif
  return true;
}


void FmRefPlane::setHeight(double height)
{
  myHeight.setValue(height);
#ifdef USE_INVENTOR
  itsDisplayPt->updateFdDetails();
#endif
}


void FmRefPlane::setWidth(double width)
{
  myWidth.setValue(width);
#ifdef USE_INVENTOR
  itsDisplayPt->updateFdDetails();
#endif
}


std::ostream& FmRefPlane::writeFMF(std::ostream& os)
{
  os <<"REF_PLANE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRefPlane::readAndConnect(std::istream& is, std::ostream&)
{
  FmRefPlane* obj = new FmRefPlane();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  return obj->cloneOrConnect();
}


bool FmRefPlane::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRefPlane::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRefPlane::getClassTypeID());
}


/*!
  Customized attach to ground link.
*/

bool FmRefPlane::attach(FmBase* attachObject)
{
  if (attachObject->isOfType(FmTriad::getClassTypeID()))
  {
    FmTriad* attachTr = static_cast<FmTriad*>(attachObject);
    if (attachTr->isSlaveTriad(true))
    {
      ListUI <<"Error: "<< attachTr->getIdString()
	     <<" is a dependent joint triad, and thus it cannot be attached to ground.\n";
      return false;
    }
    else if (attachTr->isAttached())
    {
      ListUI <<"Error: "<< attachTr->getIdString()
	     <<" is already attached to "
	     << attachTr->getOwnerLink(0)->getIdString()
	     <<", and thus it cannot be attached to ground.\n";
      return false;
    }

    // If the triad an independent triad of a line joint,
    // attach that joint instead to get all other independent triad also
    std::vector<FmJointBase*> joints;
    attachTr->getJointBinding(joints);
    for (FmJointBase* joint : joints)
      if (joint->isOfType(FmMMJointBase::getClassTypeID()))
        return this->attach(joint);

    return this->attachTriad(attachTr);
  }

  else if (attachObject->isOfType(FmSMJointBase::getClassTypeID()))
  {
    FmSMJointBase* attachJt = static_cast<FmSMJointBase*>(attachObject);
    if (attachJt->isMasterAttachedToLink(true))
    {
      if (attachJt->getMasterLink() && attachJt->getMasterLink()->isEarthLink())
	return false; // Already attached to ground, do nothing

      else if (attachJt->isSlaveAttachedToLink(true))
      {
	ListUI <<"Error: "<< attachJt->getIdString()
	       <<" is already attached, and thus it cannot be attached to ground.\n";
	return false;
      }

      // Only the independent joint triad has been attached so far, so swap the triads
      ListUI <<"Note: Swapping triads for "<< attachJt->getIdString() <<".\n";
      FmTriad* triad = attachJt->getSlaveTriad();
      attachJt->setAsSlaveTriad(attachJt->getItsMasterTriad());
      attachJt->setAsMasterTriad(triad);
    }

    return this->attachTriad(attachJt->getItsMasterTriad());
  }

  else if (attachObject->isOfType(FmMMJointBase::getClassTypeID()))
  {
    FmMMJointBase* attachJt = static_cast<FmMMJointBase*>(attachObject);
    if (attachJt->isMasterAttachedToLink(true))
    {
      ListUI <<"Error: "<< attachJt->getIdString()
	     <<" is already attached, and thus it cannot be attached to ground.\n";
      return false;
    }

    // Attach all independent triads
    std::vector<FmTriad*> triads;
    attachJt->getMasterTriads(triads);
    for (FmTriad* triad : triads)
      this->attachTriad(triad,false);

    // Update triad visualizations
    attachJt->getSlaveTriad()->updateTopologyInViewer();
    for (FmTriad* triad : triads)
      triad->updateTopologyInViewer();
  }

  else if (attachObject->isOfType(Fm1DMaster::getClassTypeID()))
  {
    Fm1DMaster* surf = static_cast<Fm1DMaster*>(attachObject);

    // Attach all triads
    std::vector<FmTriad*> triads;
    surf->getTriads(triads);
    for (FmTriad* triad : triads)
      this->attachTriad(triad,false);

    // Update triad visualizations
    for (FmTriad* triad : triads)
      triad->updateTopologyInViewer();
  }

  else
    return false; // Illegal type to attach

  return true;
}


bool FmRefPlane::attachTriad(FmTriad* attachTr, bool updateViz)
{
  // Search for an existing triad at this location
  FmLink*  earth = FmDB::getEarthLink();
  FaVec3   point = earth->getGlobalCS().inverse() * attachTr->getGlobalTranslation();
  FmTriad* oldTr = earth->getTriadAtPoint(point,FmDB::getPositionTolerance());

  if (oldTr) // We have an existing triad at this location
  {
    // Disconnect both triads so that the coordinate systems are right
    oldTr->disconnect();
    attachTr->disconnect();
    // Clone the old triad with values from the new
    attachTr->clone(oldTr,FmBase::DEEP_REPLACE);
    // Set the attachTr ID to the ID of the oldTr
    attachTr->setID(oldTr->getID());
    // Connect the attachTr once again
    attachTr->connect(earth);
    // Remove the cloned triad
    oldTr->erase();
  }
  else
  {
    attachTr->disconnect();
    attachTr->connect(earth);
  }

  if (!updateViz) return true;

  // Update the triad visualization

  std::vector<FmJointBase*> joints;
  attachTr->getJointBinding(joints);
  if (joints.empty())
    attachTr->updateTopologyInViewer();
  else for (FmJointBase* jt : joints)
  {
    jt->getSlaveTriad()->updateTopologyInViewer();
    if (jt->isOfType(FmSMJointBase::getClassTypeID()))
      static_cast<FmSMJointBase*>(jt)->getItsMasterTriad()->updateTopologyInViewer();
  }

  return true;
}
