// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmCamFriction.H"
#include "vpmDB/FmArcSegmentMaster.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdCamJoint.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcCAM_JOINT, FmCamJoint, FmMMJointBase);

FmCamJoint::FmCamJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmCamJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCamJoint(this);
#endif

  myLegalDOFs[X_TRANS] = true;
  myLegalDOFs[Y_TRANS] = true;
  myLegalDOFs[Z_TRANS] = true;
  myLegalDOFs[X_ROT]   = true;
  myLegalDOFs[Y_ROT]   = true;
  myLegalDOFs[Z_ROT]   = true;

  this->completeInitJVars();

  // With the contact element formulation (which now is default)
  // SPRING_CONSTRAINED should be default for the lateral dofs.
  // It is also the only legal choice for the X_TRANS dof.
  // The Y_TRANS, Z_TRANS and the rotational dofs may also be FREE.
  myDofStatus[X_TRANS] = SPRING_CONSTRAINED;
  myDofStatus[Y_TRANS] = SPRING_CONSTRAINED;

  FFA_FIELD_INIT(myCamWidth,        0.1,   "CAM_WIDTH");
  FFA_FIELD_INIT(myCamThickness,    0.1,   "CAM_THICKNESS");
  FFA_FIELD_INIT(IAmUsingCylCoords, false, "RADIAL_CONTACT");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

int FmCamJoint::getValidFrictionType() const
{
  return FmCamFriction::getClassTypeID();
}


FaVec3 FmCamJoint::getTransJointVariables() const
{
  return FaVec3();
}

FaVec3 FmCamJoint::getRotJointVariables() const
{
  return FaVec3();
}


bool FmCamJoint::addMasterOnPoint(const FaVec3& globPoint)
{
  if (myMaster.isNull())
  {
    Fm1DMaster* master = new FmArcSegmentMaster;
    master->setParentAssembly(this->getParentAssembly());
    master->connect();
    myMaster.setRef(master);
  }
  return myMaster->addTriadOnPoint(globPoint);
}


bool FmCamJoint::addAsMasterTriad(FmTriad* triad)
{
  if (myMaster.isNull())
  {
    Fm1DMaster* master = new FmArcSegmentMaster;
    master->setParentAssembly(this->getParentAssembly());
    master->connect();
    myMaster.setRef(master);
  }
  return myMaster->addTriad(triad);
}


std::ostream& FmCamJoint::writeFMF(std::ostream& os)
{
  os <<"CAM_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


static std::set<int> loopingCams;

bool FmCamJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmCamJoint* obj = new FmCamJoint();

  // Obsolete fields
  FFaObsoleteField<bool> loopFlag;
  FFA_OBSOLETE_FIELD_INIT(loopFlag,false,"CAM_LOOP",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"CONTACT_SURFACE") == 0)
        parentParse("MASTER", activeStatement, obj);
      else
        parentParse(keyWord, activeStatement, obj);
    }
  }

  FFA_OBSOLETE_FIELD_REMOVE("CAM_LOOP",obj);
  if (loopFlag.getValue()) loopingCams.insert(obj->getID());

  // Correct friction type when reading old model files
  int fricID = obj->myFriction.getRefID();
  if (fricID > 0 && obj->myFriction.getRefTypeID() < 0)
    obj->myFriction.setRef(fricID,FmCamFriction::getClassTypeID());

  obj->connect();
  return true;
}


void FmCamJoint::initAfterResolve()
{
  this->FmMMJointBase::initAfterResolve();

  std::set<int>::const_iterator cit = loopingCams.find(this->getID());
  if (cit == loopingCams.end()) return;

  FmArcSegmentMaster* master = dynamic_cast<FmArcSegmentMaster*>(this->getMaster());
  if (master) master->setLooping();

  loopingCams.erase(this->getID());
}


bool FmCamJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmCamJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmCamJoint::getClassTypeID());
}


void FmCamJoint::setDefaultRotationOnMasters()
{
  FmArcSegmentMaster* master = dynamic_cast<FmArcSegmentMaster*>(this->getMaster());
  if (master) master->setDefaultOrientation(this->getSlaveTriad());
}


bool FmCamJoint::getSaveVar(unsigned int& nVar, IntVec& toggles) const
{
  this->FmMMJointBase::getSaveVar(nVar,toggles);

  // The 3rd toggle (acceleration) is not used for Cam joints
  for (unsigned int i = 2; i < nVar && i+1 < toggles.size(); i++)
    toggles[i] = toggles[i+1];

  return true;
}
