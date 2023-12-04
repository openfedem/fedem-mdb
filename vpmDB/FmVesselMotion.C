// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FiDeviceFunctions/FiRAOTable.H"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmfWaveSinus.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FmfComplSinus.H"
#include "vpmDB/FmfDelayedComplSinus.H"
#include "vpmDB/FmfMathExpr.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDofMotion.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmVesselMotion.H"


Fmd_DB_SOURCE_INIT(FcVESSEL_MOTION, FmVesselMotion, FmStructPropertyBase);


FmVesselMotion::FmVesselMotion()
{
  Fmd_CONSTRUCTOR_INIT(FmVesselMotion);

  FFA_REFERENCE_FIELD_INIT(waveFunctionField, waveFunction, "WAVE_FUNCTION");
  FFA_REFERENCELIST_FIELD_INIT(motionEngineField, motionEngine, "MOTION_ENGINES");
  FFA_REFERENCE_FIELD_INIT(motionScaleField, motionScale, "MOTION_SCALE");

  FFA_REFERENCE_FIELD_INIT(raoFileRefField, raoFileRef, "RAO_FILE_REF");
  raoFileRef.setPrintIfZero(false);

  FFA_FIELD_DEFAULT_INIT(raoFile, "RAO_FILE_NAME");
  FFA_FIELD_DEFAULT_INIT(offSet, "WAVE_ORIGIN_OFFSET");
  FFA_FIELD_INIT(waveDir, 0, "WAVE_DIRECTION");
}


FmVesselMotion::~FmVesselMotion()
{
  this->disconnect();

  // Cannot use motionEngine.getFirstPtr() here in case the references has not
  // been resolved yet (might be the case in some error situations during input
  // file parsing where the model is cleaned up). It will then return NULL for
  // the unresolved references and cause crash.
  std::vector<FmEngine*> engines;
  motionEngine.getPtrs(engines);
  for (FmEngine* engine : engines)
    engine->erase();
}


std::ostream& FmVesselMotion::writeFMF(std::ostream& os)
{
  os <<"VESSEL_MOTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


static std::map<FmVesselMotion*,int> waveEng;

bool FmVesselMotion::readAndConnect(std::istream& is, std::ostream&)
{
  FmVesselMotion* obj = new FmVesselMotion();

  FFaObsoleteField<int> waveEngId;
  FFA_OBSOLETE_FIELD_INIT(waveEngId,0, "WAVE_ENGINE", obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("WAVE_ENGINE", obj);
  if (waveEngId.wasOnFile())
    waveEng[obj] = waveEngId.getValue();

  FFaFilePath::checkName(obj->raoFile.getValue());

  obj->connect();
  return true;
}


bool FmVesselMotion::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmVesselMotion::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmVesselMotion::getClassTypeID());
}


void FmVesselMotion::initAfterResolve()
{
  this->FmStructPropertyBase::initAfterResolve();

  std::map<FmVesselMotion*,int>::const_iterator it = waveEng.find(this);
  if (it != waveEng.end())
  {
    FmMathFuncBase* wfunc = NULL;
    FmBase* found = FmDB::findID(FmEngine::getClassTypeID(),it->second);
    if (found) wfunc = static_cast<FmEngine*>(found)->getFunction();
    if (wfunc)
    {
      wfunc->setUserDescription(found->getUserDescription());
      wfunc->setFunctionUse(FmMathFuncBase::WAVE_FUNCTION);
      waveFunction.setRef(wfunc);
    }
    waveEng.erase(this);
  }

  FmfWaveSinus* motion = NULL;
  for (size_t i = 0; i < motionEngine.size(); i++)
    if ((motion = dynamic_cast<FmfWaveSinus*>(motionEngine[i]->getFunction())))
      motion->lDof.setValue(i+1);

  this->onWaveChanged(false);
}


FmBase* FmVesselMotion::duplicate() const
{
  FmVesselMotion* vm = static_cast<FmVesselMotion*>(this->FmStructPropertyBase::duplicate());
  vm->motionEngine.clear();
  vm->createMotions();
  vm->motionScale.setPointer(NULL);
  vm->scaleMotions(motionScale.getPointer());

  return vm;
}


const std::string& FmVesselMotion::getActualRAOFileName() const
{
  if (!raoFileRef.isNull())
    return raoFileRef->fileName.getValue();
  else
    return raoFile.getValue();
}


void FmVesselMotion::createMotions()
{
  for (size_t idof = motionEngine.size(); idof < 6; idof++)
  {
    std::string dof = std::string(idof < 3 ? "T" : "R") + char('x'+idof%3);

    FmEngine* eng = new FmEngine();
    eng->setParentAssembly(this->getParentAssembly());
    eng->setUserDescription(dof+"-motion");
    eng->connect();

    FmMathFuncBase* func = new FmfWaveSinus(idof+1);
    func->setParentAssembly(this->getParentAssembly());
    func->setFunctionUse(FmMathFuncBase::GENERAL);
    func->connect();

    eng->setFunction(func);
    motionEngine.push_back(eng);
  }
}


bool FmVesselMotion::scaleMotions(FmEngine* scaling)
{
  if (scaling == motionScale.getPointer())
    return false;

  FmVesselMotion* vm = NULL;
  if (scaling && scaling->hasReferringObjs(vm,"motionEngine"))
    return false;

  motionScale.setPointer(scaling);
  if (!scaling)
  {
    while (motionEngine.size() > 6)
    {
      size_t i = motionEngine.size() - 1;
      size_t j = i - 6;
      motionEngine[i]->releaseReferencesToMe("myEngine",motionEngine[j]);
      motionEngine[i]->erase();
    }
    this->onChanged();
    return true;
  }
  else if (motionEngine.size() < 6)
    return false;

  FmSensorBase* s2 = NULL;
  if (motionEngine.size() > 6)
    s2 = motionEngine[6]->getSensor(1);

  size_t oldEng = motionEngine.size();
  while (motionEngine.size() < 12)
  {
    FmEngine* eng = new FmEngine();
    if (motionEngine.size() == 6)
    {
      FmfMathExpr* xy = new FmfMathExpr("x*y");
      xy->setNoArgs(2);
      xy->setParentAssembly(this->getParentAssembly());
      xy->setFunctionUse(FmMathFuncBase::GENERAL);
      xy->connect();
      eng->setFunction(xy);
    }

    FmSimpleSensor* s1 = new FmSimpleSensor();
    s1->setParentAssembly(this->getParentAssembly());
    s1->connect();
    eng->setSensor(s1,0);

    if (motionEngine.size() == 6)
    {
      s2 = new FmSimpleSensor();
      s2->setParentAssembly(this->getParentAssembly());
      s2->connect();
      eng->setSensor(s2,1);
    }
    else
    {
      eng->setEngineToLinkFunctionFrom(motionEngine[6]);
      eng->setSensor(s2,1);
    }

    eng->setParentAssembly(this->getParentAssembly());
    eng->connect();

    motionEngine.push_back(eng);
  }

  static_cast<FmSimpleSensor*>(s2)->setMeasured(motionScale.getPointer());
  for (size_t idof = 6; idof < 12; idof++)
  {
    motionEngine[idof]->setUserDescription("Scaled " + motionEngine[idof-6]->getUserDescription());
    static_cast<FmSimpleSensor*>(motionEngine[idof]->getSensor())->setMeasured(motionEngine[idof-6].getPointer());
    motionEngine[idof]->onChanged();
    if (idof >= oldEng)
      motionEngine[idof-6]->releaseReferencesToMe("myEngine",motionEngine[idof]);
  }

  return motionEngine.size() > oldEng;
}


void FmVesselMotion::onWaveChanged(bool updateSeaViz)
{
  if (waveFunction.isNull()) return;

  waveFunction->getData().clear();
  waveFunction->initGetValue();

  this->onRAOChanged(updateSeaViz);
  waveFunction->FmMathFuncBase::onChanged();
}


void FmVesselMotion::onRAOChanged(bool updateSeaViz)
{
  if (waveFunction.isNull()) return;
  if (motionEngine.size() < 6) return;

  std::string raoFileName = this->getActualRAOFileName();
  if (raoFileName.empty()) return;

  double tmp[6];
  double* wData = NULL;
  int idof, nComp = 0;
  std::vector<double>& waveData = waveFunction->getData();
  if (waveData.empty()) return;

  if (waveFunction->isOfType(FmfWaveSpectrum::getClassTypeID())) {
    nComp = waveData.size()/3;
    wData = &waveData.front();
  }
  else if (waveFunction->isOfType(FmfSinusoidal::getClassTypeID())) {
    nComp = 1;
    wData = tmp;
    tmp[0] = waveData[2];
    tmp[1] = waveData[0]*2.0*M_PI;
    tmp[2] =-waveData[1]*2.0*M_PI; // opposite sign convention on phase delay
  }
  else if (waveFunction->isOfType(FmfComplSinus::getClassTypeID()) ||
	   waveFunction->isOfType(FmfDelayedComplSinus::getClassTypeID())) {
    nComp = 2;
    wData = tmp;
    tmp[0] = waveData[4];
    tmp[1] = waveData[0]*2.0*M_PI;
    tmp[2] =-waveData[2]*2.0*M_PI; // opposite sign convention on phase delay
    tmp[3] = waveData[5];
    tmp[4] = waveData[1]*2.0*M_PI;
    tmp[5] =-waveData[3]*2.0*M_PI; // opposite sign convention on phase delay
  }
  else
    FFaMsg::list("ERROR: Invalid wave function " + waveFunction->getInfoString()
		 + ".\n       Can not calculate RAO motion data.\n",true);

  for (idof = 0; idof < 6; idof++)
    motionEngine[idof]->getFunction()->getData().resize(3*nComp,0.0);

  if (nComp > 0)
  {
    double* motionData[6];
    for (idof = 0; idof < 6; idof++)
      motionData[idof] = &motionEngine[idof]->getFunction()->getData().front();

    const std::string& path = FmDB::getMechanismObject()->getAbsModelFilePath();
    FFaFilePath::makeItAbsolute(raoFileName,path);
    if (FiRAOTable::applyRAO(raoFileName,waveDir.getValue(),3,nComp,
			     wData,motionData))
      ListUI <<"  -> RAO transforming Function "<< waveFunction->getInfoString()
	     <<", wave direction "<< waveDir.getValue() <<", "<< nComp
	     <<" wave components\n";
    else
    {
      FFaMsg::list("ERROR: RAO transformation failed for wave Function " +
		   waveFunction->getInfoString() + ".\n",true);
      for (idof = 0; idof < 6; idof++)
	motionEngine[idof]->getFunction()->getData().clear();
    }
  }

  for (idof = 0; idof < 6; idof++)
    motionEngine[idof]->getFunction()->onChanged();

  // Update the sea surface visualization, if necessary
  if (updateSeaViz) FmDB::drawSea();
}


void FmVesselMotion::onEventSwitched(const FmSimulationEvent* event)
{
  if (event->isModified(waveFunction.getPointer()))
    this->onRAOChanged(true);
  else
    this->onWaveChanged(true);
}


bool FmVesselMotion::getWaveAngles(std::vector<int>& angles) const
{
  std::string fileName = this->getActualRAOFileName();
  if (fileName.empty()) return false;

  const std::string& path = FmDB::getMechanismObject()->getAbsModelFilePath();
  FFaFilePath::makeItAbsolute(fileName,path);
  return FiRAOTable::getDirections(fileName,angles);
}


/*!
  Return the triad (or free joint) that is using all motion functions
  as prescribed displacements.
*/

FmHasDOFsBase* FmVesselMotion::getVesselTriad() const
{
  bool valid = true;
  FmHasDOFsBase* vt = NULL;
  for (size_t idof = 0; idof < motionEngine.size() && valid; idof++)
    if (!motionEngine[idof].isNull())
    {
      std::vector<FmDofMotion*> motions;
      motionEngine[idof]->getReferringObjs(motions);
      for (FmDofMotion* dofm : motions)
        if (!vt)
          vt = dofm->getOwner();
        else if (!(valid = (vt == dofm->getOwner())))
          break;
    }

  if (!vt) return vt; // nothing attached yet

  if (valid)
  {
    if (vt->isOfType(FmTriad::getClassTypeID()))
      return vt;
    else if (vt->isOfType(FmFreeJoint::getClassTypeID()))
    {
      FmTriad* triad = static_cast<FmSMJointBase*>(vt)->getItsMasterTriad();
      if (triad && triad->getNDOFs(true) == 0) return vt;
    }
  }

  FFaMsg::list("ERROR: The RAO motion functions are not properly attached.\n");
  return NULL;
}


/*!
  Return the local coordinate system for the sea wave motions.
  The Z-axis of this system is opposite of the given gravitation vector, \a g,
  and the X- and Y-axes are then to be as close as possible to the system
  directions of the vessel triad (if any), and then rotated the angle
  \a waveDir around the local Z-axis. If no vessel triad is detected,
  the X-axis is instead defined by projecting the user-defined
  wave direction vector, \a x, onto the sea surface.
*/

FaMat34 FmVesselMotion::getWaveCS(const FaVec3& g, const FaVec3& x,
				  double z0) const
{
  FaMat34 cs;
  FmHasDOFsBase* vessel = this->getVesselTriad();
  FmTriad* vesselTriad = dynamic_cast<FmTriad*>(vessel);
  if (!vessel)
    cs[VX] = x; // no RAO-motions attached, use given wave direction vector
  else if (!vesselTriad)
    cs = vessel->getGlobalCS(); // the RAO-motions are used by a free joint
  else if (vesselTriad->itsLocalDir.getValue() > FmTriad::GLOBAL)
    cs = vesselTriad->getGlobalCS(); // the triad uses local system directions

  FaVec3 eZ(-g);
  if (eZ.isZero())
    eZ = cs[VZ];
  else
    eZ.normalize();

  FaVec3 eX = cs[VY] ^ eZ;
  FaVec3 eY = eZ ^ cs[VX];
  if (vessel && eX.normalize() * cs[VX] <= eY.normalize() * cs[VY])
  {
    cs[VX] = eX;
    cs[VY] = eZ ^ eX;
  }
  else
  {
    cs[VY] = eY;
    cs[VX] = eY ^ eZ;
  }

  cs[VZ] = eZ;
  cs[VW] = z0*eZ + offSet.getValue();

  if (waveDir.getValue() == 0) return cs;

  // Rotate waveDir degrees about the local Z-axis
  return cs * FaMat33::makeZrotation(RAD(waveDir.getValue()));
}


bool FmVesselMotion::setWaveDir(int newDir)
{
  if (newDir == waveDir.getValue()) return false;

  waveDir.setValue(newDir);

  // Update the sea surface visualization, if necessary
  if (FmDB::getActiveRAO() == this) FmDB::drawSea();

  return true;
}
