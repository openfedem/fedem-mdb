// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmDofMotion.H"
#include "vpmDB/FmSMJointBase.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmfDeviceFunction.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(FcDOF_MOTION, FmDofMotion, FmIsControlledBase);


FmDofMotion::FmDofMotion()
{
  Fmd_CONSTRUCTOR_INIT(FmDofMotion);

  FFA_FIELD_INIT(myType, DEFLECTION, "MOTION_TYPE");
  FFA_FIELD_INIT(myMotionVal,   0.0, "VALUE");
  FFA_FIELD_INIT(freqDomain,  false, "FREQUENCY_DOMAIN");
}


FmDofMotion::~FmDofMotion()
{
  this->disconnect();
}


bool FmDofMotion::disconnect()
{
  this->mainDisconnect();

  FmHasDOFsBase* owner = this->getOwner();
  if (!owner) return true;

  owner->releaseMotionAtDOF(owner->atWhatDOF(this));
  return true;
}


FmModelMemberBase* FmDofMotion::getActiveOwner() const
{
  FmHasDOFsBase* owner = this->getOwner();
  if (owner)
  {
    int dof = owner->atWhatDOF(this);
    if (!owner->isLegalDOF(dof))
      return NULL;

    if (owner->getStatusOfDOF(dof) != FmHasDOFsBase::PRESCRIBED)
      return NULL;
  }

  return owner;
}


FmHasDOFsBase* FmDofMotion::getOwner() const
{
  FmHasDOFsBase* owner = NULL;
  if (this->hasReferringObjs(owner)) // there should only be one owner
    return owner;
  else
    return NULL;
}


bool FmDofMotion::setMotionType(int type)
{
  return myType.setValue(MotionTypeMapping::map()[type].first);
}


std::ostream& FmDofMotion::writeFMF(std::ostream& os)
{
  os <<"DOF_MOTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmDofMotion::readAndConnect(std::istream& is, std::ostream&)
{
  FmDofMotion* obj = new FmDofMotion();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord,is,activeStatement,'=',';'))
      localParse(keyWord,activeStatement,obj);
  }

  obj->connect();
  return true;
}


static std::map<int,Ints> ownerMap;

bool FmDofMotion::localParse(const char* keyword, std::istream& statement,
                             FmDofMotion* obj)
{
  if (strcmp(keyword,"OWNER_TRIAD") == 0)
    statement >> ownerMap[obj->getID()].first;
  else if (strcmp(keyword,"LOCAL_DOF") == 0)
    statement >> ownerMap[obj->getID()].second;
  else if (strcmp(keyword,"INIT_MOTION") == 0)
    return parentParse("VALUE",statement,obj);
  else
    return parentParse(keyword,statement,obj);

  return true;
}


void FmDofMotion::initAfterResolve()
{
  this->FmIsControlledBase::initAfterResolve();

  std::map<int,Ints>::const_iterator it = ownerMap.find(this->getID());
  if (it == ownerMap.end()) return;

  FmBase* owner = FmDB::findID(FmTriad::getClassTypeID(),it->second.first);
  if (owner)
  {
    FmTriad* triad = static_cast<FmTriad*>(owner);
    int dof = it->second.second;
    triad->setStatusForDOF(dof,FmHasDOFsBase::PRESCRIBED);
    triad->setMotionAtDOF(dof,this);
  }

  ownerMap.erase(this->getID());
}


bool FmDofMotion::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmDofMotion::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmDofMotion::getClassTypeID());
}


int FmDofMotion::checkMotions()
{
  FmAnalysis* analysis = FmDB::getActiveAnalysis();
  if (!analysis->quasistaticEnable.getValue())
    return 0;

  if (analysis->quasistaticMode.getValue())
    if (analysis->quasistaticUpToTime.getValue() <= analysis->startTime.getValue())
      return 0;

  if (analysis->stopTime.getValue() <= analysis->startTime.getValue())
    return 0;

  // We are doing at least one quasi-static load increment.
  // Check that there are no prescribed velocities in the model, as they would
  // be ignored during the quasi-static stage and yield incorrect results.

  int err = 0;
  FmHasDOFsBase* movedObj = NULL;
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,FmDofMotion::getClassTypeID());
  for (FmModelMemberBase* obj : objs)
    if (obj->hasReferringObjs(movedObj))
    {
      FmDofMotion* motion = static_cast<FmDofMotion*>(obj);
      if (motion->getMotionType() >= VELOCITY) err++;
    }

  if (err > 0)
    FFaMsg::dialog("This model has prescribed velocities and/or accelerations"
                   "\nand has been set up for quasi-static simulation.\n"
                   "This is not supported.\nEither disable the quasi-static "
                   "mode, or convert the prescribed velocity/acceleration "
                   "into a prescribed displacement instead.",FFaMsg::ERROR);

  return err;
}


int FmDofMotion::printSolverEntry(FILE* fp)
{
  if (!this->getActiveOwner())
    return 0; // DOF is not PRESCRIBED

  FmHasDOFsBase* owner = this->getOwner();
  if (owner->isSuppressed())
    return 0; // Owner is suppressed, ignore motion

  bool doFRA = false;
  FmEngine* motionEngine = this->getEngine();
  if (freqDomain.getValue() && motionEngine)
    doFRA = FmDB::getActiveAnalysis()->solveFrequencyDomain.getValue();

  if (doFRA)
    fprintf(fp,"&LOAD\n");
  else
    fprintf(fp,"&MOTION\n");
  this->printID(fp);

  int lDof = 1 + owner->atWhatDOF(this);
  if (owner->isOfType(FmJointBase::getClassTypeID()))
  {
    fprintf(fp,"  jointId = %d\n", owner->getBaseID());

    if (lDof == 3 && owner->isOfType(FmMMJointBase::getClassTypeID()))
      lDof = 7; // Slider dof

    // TEMPORARY FIX,bh:
    // For Axial joints dO is set to distance between master and
    // slave joint minus deflection of the X-translation spring
    if (static_cast<FmJointBase*>(owner)->isAxialJoint())
    {
      FmSMJointBase* joint = static_cast<FmSMJointBase*>(owner);
      // distance between master and slave triad
      double distance = (joint->getSlaveTriad()->getGlobalTranslation() -
                         joint->getItsMasterTriad()->getGlobalTranslation()).length();

      // Subtrack deflection of X-spring, if any
      FmJointSpring* springTX = joint->getSpringAtDOF(0);
      if (springTX) distance -= springTX->getInitDeflection();

      this->setInitMotion(distance);
    }
  }
  else if (owner->isOfType(FmTriad::getClassTypeID()))
  {
    fprintf(fp,"  triadId = %d\n", owner->getBaseID());
    // Beta feature: Output FE node number for prescribed displacements from file
    FmTriad* triad = static_cast<FmTriad*>(owner);
    FmPart* fePart = triad->getOwnerFEPart();
    if (fePart && FFaString(fePart->getUserDescription()).hasSubString("#Displace"))
      fprintf(fp,"  nodeId = %d\n", triad->FENodeNo.getValue());
  }
  fprintf(fp,"  lDof = %d\n", lDof);

  if (doFRA)
    fprintf(fp,"  loadType = %d\n", 2+this->getMotionType());
  else switch (this->getMotionType())
  {
  case DEFLECTION:
    fprintf(fp,"  type = 'deflection'\n");
    break;
  case VELOCITY:
    fprintf(fp,"  type = 'velocity'\n");
    break;
  case ACCELERATION:
    fprintf(fp,"  type = 'acceleration'\n");
    break;
  }

  double d0 = this->getInitMotion(); // Constant motion part
  if (doFRA)
  {
    FmfDeviceFunction* f = dynamic_cast<FmfDeviceFunction*>(motionEngine->getFunction());
    if (f)
    {
      std::string fileName(f->getActualDeviceName(false));
      if (FFaFilePath::isRelativePath(fileName) && !relPathCorrection.empty())
        fileName = relPathCorrection + fileName;
      fprintf(fp,"  fileName = '%s'\n", fileName.c_str());
    }
    else
    {
      if (fabs(d0) > 1.0e-15)
        fprintf(fp,"  f0 = %17.9e,", d0);
      fprintf(fp,"  f1 = 1.0, loadEngineId = %d\n", motionEngine->getBaseID());
    }
  }
  else if (fabs(d0) > 1.0e-15)
  {
    fprintf(fp,"  d0 = %17.9e", d0);
    if (motionEngine)
      fprintf(fp,",");
    else
      fprintf(fp,"\n");
  }
  if (motionEngine && !doFRA)
    fprintf(fp,"  d1 = 1.0,  motionEngineId = %d\n", motionEngine->getBaseID());

  // Variables to be saved:
  // 1 - Motion value
  // 2 - Energies
  if (!doFRA)
    this->writeSaveVar(fp,2);

  fprintf(fp,"/\n\n");
  return 0;
}
