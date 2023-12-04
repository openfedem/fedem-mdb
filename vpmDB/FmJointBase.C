// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmJointMotion.H"
#include "vpmDB/FmHPBase.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmLink.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdBase.H"
#include "vpmDisplay/FaDOF.H"
#endif


Fmd_DB_SOURCE_INIT(FcJOINT_BASE, FmJointBase, FmHasDOFsBase);


const Strings& FmJointBase::getRotFormulationUINames()
{
  static Strings rotFormulationTypes;
  if (rotFormulationTypes.empty())
  {
    rotFormulationTypes.reserve(3);
    rotFormulationTypes.push_back("Sequential rotation, Follower axis");
    rotFormulationTypes.push_back("Sequential rotation, Orthogonal axis");
    rotFormulationTypes.push_back("Rotational vector");
  }
  return rotFormulationTypes;
}


const Strings& FmJointBase::getRotSequenceUINames()
{
  static Strings rotSequenceTypes;
  if (rotSequenceTypes.empty())
  {
    rotSequenceTypes.reserve(6);
    rotSequenceTypes.push_back("ZYX");
    rotSequenceTypes.push_back("YXZ");
    rotSequenceTypes.push_back("XZY");
    rotSequenceTypes.push_back("XYZ");
    rotSequenceTypes.push_back("YZX");
    rotSequenceTypes.push_back("ZXY");
  }
  return rotSequenceTypes;
}


const char* FmJointBase::getRotExplain(int rotFormulation, int rotSequence)
{
  if ( rotFormulation == ROT_AXIS )
    return "Singularity free rotational formulation (independent of Sequence)";
  else if ( rotSequence == rZYX || rotSequence == rXYZ )
    return "Rotation of +/- 90 degrees about Y-axis gives a singularity";
  else if ( rotSequence == rYXZ || rotSequence == rZXY )
    return "Rotation of +/- 90 degrees about X-axis gives a singularity";
  else if ( rotSequence == rXZY || rotSequence == rYZX )
    return "Rotation of +/- 90 degrees about Z-axis gives a singularity";

  return "";
}


const Strings& FmJointBase::getSpringCplUINames()
{
  static Strings springCplTypes;
  if (springCplTypes.empty())
  {
    springCplTypes.reserve(5);
    springCplTypes.push_back("None");
    springCplTypes.push_back("Cylindrical Z");
    springCplTypes.push_back("Cylindrical X");
    springCplTypes.push_back("Cylindrical Y");
    springCplTypes.push_back("Spherical");
  }
  return springCplTypes;
}


FmJointBase::FmJointBase()
{
  Fmd_CONSTRUCTOR_INIT(FmJointBase);

  for (int i = 0; i < MAX_DOF; i++)
    myLegalDOFs[i] = false;

  FFA_FIELD_INIT(tranSpringCpl,  NONE,             "TRAN_SPRING_CPL");
  FFA_FIELD_INIT(rotSpringCpl,   NONE,             "ROT_SPRING_CPL");
  FFA_FIELD_INIT(rotFormulation, FOLLOWER_AXIS,    "ROT_FORMULATION");
  FFA_FIELD_INIT(rotSequence,    rZYX,             "ROT_SEQUENCE");
  FFA_FIELD_INIT(myDOFQuadrant,  std::vector<int>(3,0), "VAR_QUADRANTS");

  FFA_REFERENCE_FIELD_INIT(itsSlaveTriadField, itsSlaveTriad, "SLAVE_TRIAD");

  FFA_REFERENCE_FIELD_INIT(myFrictionField, myFriction, "FRICTION_OBJECT");
  myFriction.setPrintIfZero(false);
}


static const char* dof[] = { "X_TRANS","Y_TRANS","Z_TRANS","X_ROT","Y_ROT","Z_ROT" };

void FmJointBase::completeInitJVars()
{
  for (int i = 0; i < MAX_DOF; i++)
    if (myLegalDOFs[i])
    {
      FFA_FIELD_INIT(myDofStatus[i], FREE, std::string(dof[i]) + "_STATUS");

      FFA_REFERENCE_FIELD_INIT(mySpringFields[i], mySprings[i], std::string(dof[i]) + "_SPRING");
      FFA_REFERENCE_FIELD_INIT(myDamperFields[i], myDampers[i], std::string(dof[i]) + "_DAMPER");

      mySprings[i].setPrintIfZero(false);
      myDampers[i].setPrintIfZero(false);
    }

  this->completeInitDOFs();
}


FmJointBase::~FmJointBase()
{
  for (int i = 0; i < MAX_DOF; i++)
  {
    if (mySprings[i])
      mySprings[i]->erase();
    if (myDampers[i])
      myDampers[i]->erase();
  }

  std::vector<FmHPBase*> hps;
  this->getReferringObjs(hps);
  for (FmHPBase* hp : hps) hp->erase();
}


void FmJointBase::eraseInternal()
{
  // To get user approval to delete any curve axis definitions using this joint
  this->FmIsPlottedBase::eraseOptions();
  delete this;
}


void FmJointBase::updateChildrenDisplayTopology()
{
  std::vector<FmHPBase*> hps;
  this->getReferringObjs(hps);
  for (FmHPBase* hp : hps) hp->updateTopologyInViewer();

  this->FmHasDOFsBase::updateChildrenDisplayTopology();
}


/*!
  Convenience functions used by FmSolverParser
*/

bool FmJointBase::isAxialJoint(bool useLocalDofsOnly) const
{
  if (!this->isOfType(FmFreeJoint::getClassTypeID()))
    return false;
  else if (FFaString(this->getUserDescription()).hasSubString("#Axial"))
    return true;
  else if (useLocalDofsOnly)
    return FFaString(this->getUserDescription()).hasSubString("#LocalDofs");

  return false;
}


bool FmJointBase::isGlobalSpringElement() const
{
  if (this->isOfType(FmFreeJoint::getClassTypeID()))
    if (FFaString(this->getUserDescription()).hasSubString("#GlobalSpring"))
      return true;

  return false;
}


bool FmJointBase::isContactElement() const
{
  if (this->isOfType(FmCamJoint::getClassTypeID()))
    if (!FFaString(this->getUserDescription()).hasSubString("#MasterSlaveCam"))
      return true;

  return false;
}


bool FmJointBase::isAttachedToLink(const FmLink* thisLink) const
{
  if (this->isMasterAttachedToLink())
    if (!thisLink || this->getMasterLink() != (FmLink*)thisLink)
      return true;

  if (this->isSlaveAttachedToLink())
    if (!thisLink || this->getSlaveLink() != (FmLink*)thisLink)
      return true;

  return false;
}


FmLink* FmJointBase::getOtherLink(const FmLink* thisLink) const
{
  if (!thisLink)
    return NULL;

  if (this->isMasterAttachedToLink())
    if (this->getMasterLink() != (FmLink*)thisLink)
      return this->getMasterLink();

  if (this->isSlaveAttachedToLink())
    if (this->getSlaveLink() != (FmLink*)thisLink)
      return this->getSlaveLink();

  return NULL;
}


FmLink* FmJointBase::getOtherLink(const FmTriad* jointTriad) const
{
  if (!jointTriad)
    return NULL;

  if (this->isSlaveTriad(jointTriad))
    return this->getMasterLink();

  if (this->isMasterTriad(jointTriad))
    return this->getSlaveLink();

  return NULL;
}


bool FmJointBase::isSuppressed() const
{
  FmLink* link = this->getSlaveLink();
  if (link)
    if (link->isSuppressed())
      if ((link = this->getMasterLink()))
	return link->isSuppressed() || link->isEarthLink();

  return false;
}


bool FmJointBase::isSlaveAttachedToLink(bool allowMultipleLinks) const
{
  FmTriad* slaveTriad = this->getSlaveTriad();
  if (slaveTriad)
    if (slaveTriad->isAttached(false,allowMultipleLinks))
      return true;

  return false;
}


FmLink* FmJointBase::getSlaveLink() const
{
  if (this->isSlaveAttachedToLink())
    return this->getSlaveTriad()->getOwnerLink();
  else
    return NULL;
}


#ifdef USE_INVENTOR
FaDOF FmJointBase::getObjDegOfFreedom() const
{
  FaMat34 jCS;
  if (this->isOfType(FmSMJointBase::getClassTypeID()))
    jCS = this->getGlobalCS();
  else
    jCS = this->getSlaveTriad()->getGlobalCS();
  FaVec3 direction(jCS[VZ]);
  int type = FaDOF::FREE;

  bool ballBehaviour = false;
  if (this->isOfType(FmFreeJoint::getClassTypeID()))
  {
    DOFStatus TX = this->getStatusOfDOF(0);
    DOFStatus TY = this->getStatusOfDOF(1);
    DOFStatus TZ = this->getStatusOfDOF(2);
    if (TX > FREE && TY > FREE && TZ > FREE)
      ballBehaviour = true;
    else
    {
      DOFStatus RX = this->getStatusOfDOF(3);
      DOFStatus RY = this->getStatusOfDOF(4);
      DOFStatus RZ = this->getStatusOfDOF(5);
      if (TX > FREE && TY > FREE && RX > FREE && RY > FREE)
	type = RZ > FREE ? FaDOF::PRISM : FaDOF::CYL;
      else if (TX > FREE && TZ > FREE && RX > FREE && RZ > FREE)
      {
	direction = jCS[VY];
	type = RY > FREE ? FaDOF::PRISM : FaDOF::CYL;
      }
      else if (TY > FREE && TZ > FREE && RY > FREE && RZ > FREE)
      {
	direction = jCS[VX];
	type = RX > FREE ? FaDOF::PRISM : FaDOF::CYL;
      }
      // TODO: All other combinations are treated as FREE, support others?
    }
  }
  if (ballBehaviour || this->isOfType(FmBallJoint::getClassTypeID()))
  {
    DOFStatus RX = this->getStatusOfDOF(3);
    DOFStatus RY = this->getStatusOfDOF(4);
    DOFStatus RZ = this->getStatusOfDOF(5);
    if (RX > FREE && RY > FREE && RZ > FREE)
      type = FaDOF::RIGID;
    else if (RX > FREE && RY > FREE)
      type = FaDOF::REV;
    else if (RX > FREE && RZ > FREE)
    {
      direction = jCS[VY];
      type = FaDOF::REV;
    }
    else if (RY > FREE && RZ > FREE)
    {
      direction = jCS[VX];
      type = FaDOF::REV;
    }
    // TODO: When only one DOF is constrained?
    else
      type = FaDOF::BALL;
  }
  else if (this->isOfType(FmRevJoint::getClassTypeID()) ||
	   this->isOfType(FmCylJoint::getClassTypeID()))
  {
    if (this->getStatusOfDOF(5) > FREE)
      type = this->getStatusOfDOF(2) > FREE ? FaDOF::RIGID : FaDOF::PRISM;
    else
      type = this->getStatusOfDOF(2) > FREE ? FaDOF::REV : FaDOF::CYL;
  }

  else if (this->isOfType(FmPrismJoint::getClassTypeID()))
    type = this->getStatusOfDOF(2) > FREE ? FaDOF::RIGID : FaDOF::PRISM;

  else if (this->isOfType(FmRigidJoint::getClassTypeID()))
    type = FaDOF::RIGID;

  return FaDOF(jCS.translation(),direction,type);
}
#endif


bool FmJointBase::isLegalDOF(int DOFno) const
{
  if (DOFno >= 0 && DOFno < MAX_DOF)
    return myLegalDOFs[DOFno];
  else
    return false;
}


int FmJointBase::getLegalDOFCount() const
{
  int legalDOFcount = 0;
  for (int i = 0; i < MAX_DOF; i++)
    if (myLegalDOFs[i])
      legalDOFcount++;

  return legalDOFcount;
}


int FmJointBase::getJointVariableNumber(int dofNo) const
{
  int dofCount = 1;
  for (int i = 0; i < MAX_DOF && i < dofNo; i++)
    if (myLegalDOFs[i])
      dofCount++;

  return dofCount;
}


double FmJointBase::getJointVariable(int var) const
{
  if (!this->isLegalDOF(var))
    return 0.0;

  if (var == X_TRANS || var == Y_TRANS || var == Z_TRANS)
    return this->getTransJointVariables()[var];
  else
    return this->getRotJointVariables()[var-X_ROT];
}


void FmJointBase::getEntities(std::vector<FmSensorChoice>& choicesToFill, int dof)
{
  choicesToFill.clear();

  DOFStatus status = this->getStatusOfDOF(dof);
  if (status != FIXED) {
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::REL_POS]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::VEL]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::ACCEL]);
  }
  if (status == FIXED || status == PRESCRIBED)
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::FORCE]);
  else if (status >= SPRING_CONSTRAINED) {
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JSPR_ANG]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JSPR_DEFL]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JSPR_FORCE]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JDAMP_ANG]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JDAMP_VEL]);
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::JDAMP_FORCE]);
  }
}


void FmJointBase::getDofs(std::vector<FmSensorChoice>& choicesToFill)
{
  choicesToFill.clear();
  for (int i = 0; i < MAX_DOF; i++)
    if (myLegalDOFs[i])
      choicesToFill.push_back(itsDofTable[i]);
}


bool FmJointBase::setSpringAtDOF(int DOFno, FmJointSpring* spr, bool forceReplace)
{
  if (!this->isLegalDOF(DOFno))
    return false;

  else if (!mySprings[DOFno])
    {
      if (spr) {
	spr->disconnect();
	mySprings[DOFno] = spr;
	spr->connect();
      }
    }

  else if (forceReplace)
    {
      if (spr != mySprings[DOFno])
        removeSpringAtDOF(DOFno);

      mySprings[DOFno] = spr;
    }

  else
    return false;

  return true;
}


bool FmJointBase::setDamperAtDOF(int DOFno, FmJointDamper* dmp, bool forceReplace)
{
  if (!this->isLegalDOF(DOFno))
    return false;

  else if (!myDampers[DOFno])
    {
      if (dmp) {
	dmp->disconnect();
	myDampers[DOFno] = dmp;
	dmp->connect();
      }
    }

  else if (forceReplace)
    {
      if (dmp != myDampers[DOFno])
        removeDamperAtDOF(DOFno);

      myDampers[DOFno] = dmp;
    }

  else
    return false;

  return true;
}


FmJointSpring* FmJointBase::getSpringAtDOF(int DOFno, bool createIfNone)
{
  if (!this->isLegalDOF(DOFno)) return NULL;

  if (!mySprings[DOFno] && createIfNone)
  {
    mySprings[DOFno] = new FmJointSpring();
    mySprings[DOFno]->setParentAssembly(this->getParentAssembly());
    mySprings[DOFno]->connect();
  }

  return mySprings[DOFno];
}


int FmJointBase::getSpringBaseID(int DOFno) const
{
  if (this->isLegalDOF(DOFno))
    if (mySprings[DOFno] && mySprings[DOFno]->getActiveOwner())
      return mySprings[DOFno]->getBaseID();

  return 0;
}


FmJointDamper* FmJointBase::getDamperAtDOF(int DOFno, bool createIfNone)
{
  if (!this->isLegalDOF(DOFno)) return NULL;

  if (!myDampers[DOFno] && createIfNone)
  {
    myDampers[DOFno] = new FmJointDamper();
    myDampers[DOFno]->setParentAssembly(this->getParentAssembly());
    myDampers[DOFno]->connect();
  }

  return myDampers[DOFno];
}


int FmJointBase::getDamperBaseID(int DOFno) const
{
  if (this->isLegalDOF(DOFno))
    if (myDampers[DOFno] && myDampers[DOFno]->getActiveOwner())
      return myDampers[DOFno]->getBaseID();

  return 0;
}


FmDofMotion* FmJointBase::getMotionAtDOF(int DOFno, bool createIfNone)
{
  if (!this->isLegalDOF(DOFno)) return NULL;

  if (!myMotions[DOFno] && createIfNone)
  {
    myMotions[DOFno] = new FmJointMotion();
    myMotions[DOFno]->setParentAssembly(this->getParentAssembly());
    myMotions[DOFno]->connect();
  }

  return myMotions[DOFno];
}


void FmJointBase::removeSpringAtDOF(int DOFno)
{
  if (this->isLegalDOF(DOFno))
    if (mySprings[DOFno])
    {
      mySprings[DOFno]->erase();
      mySprings[DOFno] = NULL;
    }
}


void FmJointBase::removeDamperAtDOF(int DOFno)
{
  if (this->isLegalDOF(DOFno))
    if (myDampers[DOFno])
    {
      myDampers[DOFno]->erase();
      myDampers[DOFno] = NULL;
    }
}


void FmJointBase::releaseSpringAtDOF(int DOFno)
{
  if (this->isLegalDOF(DOFno))
    mySprings[DOFno] = NULL;
}


void FmJointBase::releaseDamperAtDOF(int DOFno)
{
  if (this->isLegalDOF(DOFno))
    myDampers[DOFno] = NULL;
}


bool FmJointBase::setStatusForDOF(int dof, DOFStatus dstat)
{
  if (!this->isLegalDOF(dof)) return false;

  return myDofStatus[dof].setValue(dstat);
}


FmJointBase::DOFStatus FmJointBase::getStatusOfDOF(int dof) const
{
  if (!this->isLegalDOF(dof)) return FIXED;

  return myDofStatus[dof].getValue();
}


bool FmJointBase::hasConstraints(bool fixedOnly) const
{
  for (int dof = 0; dof < MAX_DOF; dof++)
    if (this->isLegalDOF(dof))
      switch (myDofStatus[dof].getValue())
	{
	case FIXED:
	case FREE_DYNAMICS:
	case SPRING_DYNAMICS:
	  return true;
	case PRESCRIBED:
	  if (!fixedOnly)
	    return true;
        default:
          break;
	}

  return false;
}


void FmJointBase::setInitVel(int dof, double vel)
{
  if (!this->isLegalDOF(dof)) return;

  if ((size_t)dof < initVel.getValue().size())
    initVel.getValue()[dof] = vel;
  else if (vel != 0.0)
  {
    initVel.getValue().resize(dof+1,0.0);
    initVel.getValue()[dof] = vel;
  }
}


void FmJointBase::setInitAcc(int dof, double acc)
{
  if (!this->isLegalDOF(dof))
    return;

  if ((size_t)dof < initAcc.getValue().size())
    initAcc.getValue()[dof] = acc;
  else if (acc != 0.0)
  {
    initAcc.getValue().resize(dof+1,0.0);
    initAcc.getValue()[dof] = acc;
  }
}


bool FmJointBase::setAsSlaveTriad(FmTriad* slTriad)
{
  this->removeItsSlaveTriad();
  itsSlaveTriad = slTriad;
  return true;
}


bool FmJointBase::removeItsSlaveTriad()
{
  if (!itsSlaveTriad)
    return false;

#ifdef USE_INVENTOR
  FmTriad* oldTr = itsSlaveTriad;
#endif
  itsSlaveTriad = NULL;
#ifdef USE_INVENTOR
  oldTr->getFdPointer()->updateFdDetails();
#endif
  return true;
}


bool FmJointBase::isSlaveTriad(const FmTriad* triad) const
{
  return (itsSlaveTriad == triad);
}


FmTriad* FmJointBase::getSlaveTriad() const
{
  return itsSlaveTriad;
}


bool FmJointBase::isMasterSlaveInOtherJoint() const
{
  std::vector<FmTriad*> masters;
  this->getMasterTriads(masters);

  for (FmTriad* master : masters)
    if (master->isSlaveTriad())
      return true;

  return false;
}



int FmJointBase::atWhatDOF(const FmJointSpring* spr) const
{
  for (int i = 0; i < MAX_DOF; i++)
    if (mySprings[i] == spr)
      return i;

  return -1;
}


int FmJointBase::atWhatDOF(const FmJointDamper* dmp) const
{
  for (int i = 0; i < MAX_DOF; i++)
    if (myDampers[i] == dmp)
      return i;

  return -1;
}


int FmJointBase::atWhatDOF(const FmJointMotion* pm) const
{
  return this->FmHasDOFsBase::atWhatDOF(pm);
}


FaVec3 FmJointBase::getJointRotations(const FaMat34& from,
				      const FaMat34& to) const
{
  FaVec3 rotVars = FaMat34::getEulerZYX(from,to);

  for (int i = 0; i < 3; i++)
    if (myDOFQuadrant.getValue()[i])
      rotVars[i] += myDOFQuadrant.getValue()[i] * 2.0*M_PI;

  return rotVars;
}


void FmJointBase::setJointRotations(const FaVec3& rotations,
				    const FaMat34& masterCS)
{
  FmTriad* slaveTr = this->getSlaveTriad();
  if (!slaveTr) return;

  FaMat34 slaveCS = slaveTr->getGlobalCS();

  // quasi-setting of the DOF quadrant.
  // i=0 means first or second quadrant, 1 means larger angles.
  for (int i = 0; i < 3; i++)
    {
      double val = rotations[i]/M_PI;
      if (val > 1.0 && val < 2.0)
	myDOFQuadrant.getValue()[i] = 1;
      else if (val < -1.0 && val > -2.0)
	myDOFQuadrant.getValue()[i] = -1;
      else
	myDOFQuadrant.getValue()[i] = 0;
    }

  slaveCS.eulerRotateZYX(rotations,masterCS);
  slaveTr->setGlobalCS(slaveCS);

  slaveTr->updateDisplayTopology();
}


bool FmJointBase::hasHPConnections() const
{
  FmHPBase* hp = NULL;
  return this->hasReferringObjs(hp);
}


FmHPBase* FmJointBase::getHPConnection() const
{
  FmHPBase* hp = NULL;
  if (this->hasReferringObjs(hp,"itsOutputJoint"))
    return hp; // There should only be one or none
  else
    return NULL;
}


bool FmJointBase::localParse(const char* keyWord, std::istream& activeStatement,
			     FmJointBase* obj)
{
  // Conversion of some pre R5.1 keywords
  int motionDof = -1;
  for (int i = 0; i < 6; i++)
    if (std::string(dof[i])+"_MOTION_TYPE" == keyWord)
      return parentParse((std::string(dof[i])+"_STATUS").c_str(),activeStatement,obj);
    else if (std::string(dof[i])+"_JVAR_INIT_VEL" == keyWord)
    {
      double initVel;
      activeStatement >> initVel;
      obj->setInitVel(i,initVel);
      return true;
    }
    else if (std::string(dof[i])+"_JVAR_INIT_ACC" == keyWord)
    {
      double initAcc;
      activeStatement >> initAcc;
      obj->setInitAcc(i,initAcc);
      return true;
    }
    else if (std::string(dof[i])+"_MOTION" == keyWord)
    {
      motionDof = i;
      break;
    }

  bool retVal = parentParse(keyWord, activeStatement, obj);

  // Manually fix joint motion reference for old model files
  if (motionDof >= 0 && obj->myMotions[motionDof].getRefTypeID() == 0)
    obj->myMotions[motionDof].setRef(obj->myMotions[motionDof].getRefID(),
				     FmJointMotion::getClassTypeID());
  return retVal;
}


void FmJointBase::initAfterResolve()
{
  this->FmIsPositionedBase::initAfterResolve();

  this->setAsSlaveTriad(itsSlaveTriad);
  for (int i = 0; i < MAX_DOF; i++)
    if (myLegalDOFs[i])
    {
      this->setSpringAtDOF(i, mySprings[i], true);
      this->setDamperAtDOF(i, myDampers[i], true);
      this->setLoadAtDOF  (i, myLoads[i],   true);
      this->setMotionAtDOF(i, myMotions[i], true);
    }
}


bool FmJointBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmJointBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmJointBase* copyObj = static_cast<FmJointBase*>(obj);

  FmTriad* slTr = copyObj->getSlaveTriad();
  if (slTr) {
    if (depth == FmBase::DEEP_REPLACE)
      copyObj->removeItsSlaveTriad();
    this->setAsSlaveTriad(slTr);
  }

  for (int i = 0; i < MAX_DOF; i++)
    if (myLegalDOFs[i]) {
      this->setDamperAtDOF(i, copyObj->getDamperAtDOF(i), true);
      this->setSpringAtDOF(i, copyObj->getSpringAtDOF(i), true);
      this->setLoadAtDOF  (i, copyObj->getLoadAtDOF(i)  , true);
      this->setMotionAtDOF(i, copyObj->getMotionAtDOF(i), true);
    }

  if (depth == FmBase::DEEP_REPLACE) {
    copyObj->releaseReferencesToMe("itsInputJoint", this);
    copyObj->releaseReferencesToMe("itsOutputJoint", this);
  }

  return true;
}


int FmJointBase::checkJoints()
{
  int errorCount = 0;

  std::vector<FmJointBase*> allJoints;
  FmDB::getAllJoints(allJoints);
  for (FmJointBase* joint : allJoints)
    if (joint->isSuppressed() && joint->isMeasured())
    {
      errorCount++;
      ListUI <<"ERROR: "<< joint->getIdString(true)
             <<" is suppressed and used as Function argument.\n";
    }
    else for (int dof = 0; dof < FmHasDOFsBase::MAX_DOF; dof++)
      // Bugfix #380: Ensure DOF motion objects exist for prescribed DOFs
      if (joint->isLegalDOF(dof))
        if (joint->getStatusOfDOF(dof) == FmHasDOFsBase::PRESCRIBED)
          joint->getMotionAtDOF(dof,true);

  return errorCount;
}


bool FmJointBase::getSaveVar(unsigned int& nVar, IntVec& toggles) const
{
  if (nVar < 1) return false;

  if (5*mySaveVar.getValue().size() < nVar)
    nVar = 5*mySaveVar.getValue().size();
  for (unsigned int i = 0; i < nVar; i += 5)
    for (unsigned int j = 0; j < 5 && i+j < toggles.size(); j++)
      toggles[i+j] = mySaveVar.getValue()[j] ? 1 : 0;

  return true;
}
