// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "turbineConverter.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmfUserDefined.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"


bool FWP::createRegulationSystem(FmTurbine* turbine, FmRotor* rotor,
				 FmRevJoint* generator)
{
  const int Rz = FmHasDOFsBase::Z_ROT;

  // Check if we already have a regulation system
  bool deleteRSys = false;
  FmDofLoad* load = generator ? generator->getLoadAtDOF(Rz) : NULL;
  if (load && load->getEngine())
  {
    if (turbine->ctrlSys.getValue())
      return true; // yes, leave it untouched
    else
      deleteRSys = true; // yes, remove it
  }
  else if (!turbine->ctrlSys.getValue())
    return true; // no, and we don't want one either

  // Find the pitch joints
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,FmRevJoint::getClassTypeID(),rotor);
  std::vector<FmRevJoint*> pitch(objs.size());
  for (size_t i = 0; i < pitch.size(); i++)
    pitch[i] = static_cast<FmRevJoint*>(objs[i]);

  if (deleteRSys)
  {
    // Delete the existing regulation system
    FmSensorBase* arg = load->getEngine()->getSensor(1);
    FmEngine* filter = arg ? dynamic_cast<FmEngine*>(arg->getMeasured()) : NULL;
    if (filter) filter->erase();
    load->getEngine()->erase();
    generator->setLoadAtDOF(Rz,NULL,true);
    for (FmRevJoint* joint : pitch)
    {
      FmDofMotion* motion = joint->getMotionAtDOF(Rz,false);
      if (motion && motion->getEngine())
      {
	motion->getEngine()->erase();
	joint->setStatusForDOF(Rz,FmHasDOFsBase::FIXED);
	joint->setMotionAtDOF(Rz,NULL,true);
      }
    }
    return true;
  }

  // Create the Low-pass velocity filter
  FmfUserDefined* f = new FmfUserDefined();
  if (!f->setFuncId(101))
  {
    f->erase();
    return false;
  }

  f->setFunctionUse(FmMathFuncBase::GENERAL);
  f->setParentAssembly(turbine);
  f->connect();

  std::vector<FmParameter> params;
  f->getParameters(params);
  params.front().setFcn(f,0.25); // Corner frequency

  FmEngine* filter = new FmEngine();
  filter->setParentAssembly(turbine);
  filter->setUserDescription("Filtered velocity");
  filter->connect();
  filter->setFunction(f);
  filter->setSensor(generator->getSimpleSensor(true),0);
  filter->setEntity(FmIsMeasuredBase::VEL,0);
  filter->setDof(FmIsMeasuredBase::Z_ROT,0);
  filter->setSensor(FmDB::getTimeSensor(),1);

  // Create the Pitch Controller
  f = new FmfUserDefined();
  if (!f->setFuncId(103))
  {
    f->erase();
    return false;
  }

  f->setFunctionUse(FmMathFuncBase::GENERAL);
  f->setParentAssembly(turbine);
  f->connect();

  params.clear();
  f->getParameters(params);
  params[0].setFcn(f,122.9096);    // Reference speed
  params[1].setFcn(f,0.008068634); // Integral gain (Ki)
  params[2].setFcn(f,0.1099965);   // Pitch for doubled power (Kk)
  params[3].setFcn(f,0.01882681);  // Proportional gain (Kp)
  params[4].setFcn(f,0.0);         // Minimum pitch setting
  params[5].setFcn(f,1.570796);    // Maximum pitch setting
  params[6].setFcn(f,0.1396263);   // Maximum pitch rate

  FmEngine* pctrl = new FmEngine();
  pctrl->setParentAssembly(turbine);
  pctrl->setUserDescription("Pitch controller");
  pctrl->connect();
  pctrl->setFunction(f);
  pctrl->setSensor(pitch.front()->getSimpleSensor(true),0);
  pctrl->setEntity(FmIsMeasuredBase::REL_POS,0);
  pctrl->setDof(FmIsMeasuredBase::Z_ROT,0);
  pctrl->setSensor(filter->getSimpleSensor(true),1);
  pctrl->setSensor(FmDB::getTimeSensor(),2);

  // Create the Torque Controller
  f = new FmfUserDefined();
  if (!f->setFuncId(102))
  {
    f->erase();
    return false;
  }

  f->setFunctionUse(FmMathFuncBase::GENERAL);
  f->setParentAssembly(turbine);
  f->connect();

  params.clear();
  f->getParameters(params);
  params[0].setFcn(f,0.0174533); // Minimum pitch, region 3
  params[1].setFcn(f,121.6805);  // Rated speed
  params[2].setFcn(f,5296610.0); // Rated power, region 3
  params[3].setFcn(f,70.16224);  // Transition speed, region 1 & 1.5
  params[4].setFcn(f,91.21091);  // Transition speed, region 1.5 & 2
  params[5].setFcn(f,2.332287);  // Transition speed, region 1
  params[6].setFcn(f,10.0);      // Rated slip percentage, region 2.5
  params[7].setFcn(f,47402.91);  // Maximum torque, region 3
  params[8].setFcn(f,15000.0);   // Maximum torque rate

  FmEngine* tctrl = new FmEngine();
  tctrl->setParentAssembly(turbine);
  tctrl->setUserDescription("Torque controller");
  tctrl->connect();
  tctrl->setFunction(f);
  tctrl->setSensor(pctrl->getSimpleSensor(true),0);
  tctrl->setSensor(filter->getSimpleSensor(true),1);
  tctrl->setSensor(FmDB::getTimeSensor(),2);

  // Insert the Torque and Pitch controllers into the Turbine
  generator->getLoadAtDOF(Rz,true)->setEngine(tctrl);
  for (FmRevJoint* joint : pitch)
  {
    joint->setStatusForDOF(Rz,FmHasDOFsBase::PRESCRIBED);
    joint->getMotionAtDOF(Rz,true)->setEngine(pctrl);
  }

  return true;
}
