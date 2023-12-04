// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmHasDOFsBase.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmDB.H"


Fmd_DB_SOURCE_INIT(FcHAS_DOFS_BASE, FmHasDOFsBase, FmIsPositionedBase);


FmHasDOFsBase::FmHasDOFsBase()
{
  Fmd_CONSTRUCTOR_INIT(FmHasDOFsBase);

  FFA_FIELD_DEFAULT_INIT(initVel, "INIT_VELOCITY");
  FFA_FIELD_DEFAULT_INIT(initAcc, "INIT_ACCELERATION");
}


void FmHasDOFsBase::completeInitDOFs()
{
  const char* dof[] = { "X_TRANS","Y_TRANS","Z_TRANS","X_ROT","Y_ROT","Z_ROT" };

  for (int i = 0; i < MAX_DOF; i++)
    if (this->isLegalDOF(i))
    {
      FFA_REFERENCE_FIELD_INIT(myLoadFields[i],   myLoads[i],   std::string(dof[i]) + "_LOAD"  );
      FFA_REFERENCE_FIELD_INIT(myMotionFields[i], myMotions[i], std::string(dof[i]) + "_MOTION");

      myLoads[i].setPrintIfZero(false);
      myMotions[i].setPrintIfZero(false);
    }
}


FmHasDOFsBase::~FmHasDOFsBase()
{
  for (int i = 0; i < MAX_DOF; i++)
  {
    if (myLoads[i])
      myLoads[i]->erase();
    if (myMotions[i])
      myMotions[i]->erase();
  }
}


bool FmHasDOFsBase::setLoadAtDOF(int dof, FmDofLoad* load, bool forceReplace)
{
  if (!this->isLegalDOF(dof))
    return false;

  else if (!myLoads[dof])
  {
    if (load) {
      load->disconnect();
      myLoads[dof] = load;
      load->connect();
    }
  }

  else if (forceReplace)
  {
    if (load != myLoads[dof])
      myLoads[dof]->erase();

    myLoads[dof] = load;
  }

  else
    return false;

  return true;
}


bool FmHasDOFsBase::setMotionAtDOF(int dof, FmDofMotion* pm, bool forceReplace)
{
  if (!this->isLegalDOF(dof))
    return false;

  else if (!myMotions[dof])
  {
    if (pm) {
      pm->disconnect();
      myMotions[dof] = pm;
      pm->connect();
    }
  }

  else if (forceReplace)
  {
    if (pm != myMotions[dof])
      myMotions[dof]->erase();

    myMotions[dof] = pm;
  }

  else
    return false;

  return true;
}


FmDofLoad* FmHasDOFsBase::getLoadAtDOF(int dof, bool createIfNone)
{
  if (!this->isLegalDOF(dof)) return 0;

  if (!myLoads[dof] && createIfNone)
  {
    myLoads[dof] = new FmDofLoad();
    myLoads[dof]->setParentAssembly(this->getParentAssembly());
    myLoads[dof]->connect();
  }

  return myLoads[dof];
}


int FmHasDOFsBase::getLoadBaseID(int dof) const
{
  if (this->isLegalDOF(dof))
    if (myLoads[dof] && myLoads[dof]->getActiveOwner())
      return myLoads[dof]->getBaseID();

  return 0;
}


FmDofMotion* FmHasDOFsBase::getMotionAtDOF(int dof, bool createIfNone)
{
  if (!this->isLegalDOF(dof)) return 0;

  if (!myMotions[dof] && createIfNone)
  {
    myMotions[dof] = new FmDofMotion();
    myMotions[dof]->setParentAssembly(this->getParentAssembly());
    myMotions[dof]->connect();
  }

  return myMotions[dof];
}


int FmHasDOFsBase::getMotionBaseID(int dof) const
{
  if (this->isLegalDOF(dof))
    if (myMotions[dof] && myMotions[dof]->getActiveOwner())
      return myMotions[dof]->getBaseID();

  return 0;
}


void FmHasDOFsBase::releaseLoadAtDOF(int dof)
{
  if (dof >= 0) myLoads[dof] = NULL;
}


void FmHasDOFsBase::releaseMotionAtDOF(int dof)
{
  if (dof >= 0) myMotions[dof] = NULL;
}


void FmHasDOFsBase::getDOFs(std::vector<int>& dofs) const
{
  dofs.clear();
  dofs.reserve(MAX_DOF);
  for (int dof = 0; dof < MAX_DOF; dof++)
    if (this->isLegalDOF(dof))
      dofs.push_back(dof);
}


bool FmHasDOFsBase::setStatusForDOF(int dof, int status)
{
  DOFStatus dstat = DOFStatusMapping::map()[status].first;
  return this->setStatusForDOF(dof,dstat);
}


int FmHasDOFsBase::getStatusCode(int dof) const
{
  switch (this->getStatusOfDOF(dof)) {
  case FIXED:
    return 0;
  case FREE_DYNAMICS:
  case SPRING_DYNAMICS:
    return 2;
  default:
    break;
  }
  return 1;
}


int FmHasDOFsBase::atWhatDOF(const FmDofLoad* load) const
{
  for (int i = 0; i < MAX_DOF; i++)
    if (myLoads[i] == load)
      return i;

  return -1;
}


int FmHasDOFsBase::atWhatDOF(const FmDofMotion* pm) const
{
  for (int i = 0; i < MAX_DOF; i++)
    if (myMotions[i] == pm)
      return i;

  return -1;
}



double FmHasDOFsBase::getInitVel(int dof, bool includeFixed) const
{
  if ((size_t)dof < initVel.getValue().size() && this->isLegalDOF(dof))
    if (includeFixed || this->getStatusOfDOF(dof) != FIXED)
      return initVel.getValue()[dof];

  return 0.0;
}


double FmHasDOFsBase::getInitAcc(int dof, bool includeFixed) const
{
  if ((size_t)dof < initAcc.getValue().size() && this->isLegalDOF(dof))
    if (includeFixed || this->getStatusOfDOF(dof) != FIXED)
      return initAcc.getValue()[dof];

  return 0.0;
}


bool FmHasDOFsBase::localParse(const char* keyWord, std::istream& activeStatement,
			       FmHasDOFsBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


void FmHasDOFsBase::initAfterResolve()
{
  this->FmIsPositionedBase::initAfterResolve();

  for (int i = 0; i < MAX_DOF; i++)
  {
    this->setLoadAtDOF  (i, myLoads[i],   true);
    this->setMotionAtDOF(i, myMotions[i], true);
  }
}


bool FmHasDOFsBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmHasDOFsBase::getClassTypeID());
}


std::ostream& operator<<(std::ostream& s, const std::vector<FmHasDOFsBase::DOFStatusEnum>& obj)
{
  if (obj.empty()) return s;

  s << obj.front();
  for (size_t i = 1; i < obj.size(); i++)
    s <<" "<< obj[i];

  return s;
}


std::istream& operator>>(std::istream& s, std::vector<FmHasDOFsBase::DOFStatusEnum>& obj)
{
  obj.clear();

  FmHasDOFsBase::DOFStatusEnum ds;
  while (s)
  {
    s >> ds;
    if (s)
      obj.push_back(ds);
  }

  s.clear();
  return s;
}


void FmHasDOFsBase::updateChildrenDisplayTopology()
{
  // Update sea visualization if this object is connected to it
  FmVesselMotion* raom = FmDB::getActiveRAO();
  if (raom && raom->getVesselTriad() == this)
    FmDB::drawSea();

  this->FmIsPositionedBase::updateChildrenDisplayTopology();
}
