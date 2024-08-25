// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "turbineConverter.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmGear.H"
#include "vpmDB/FmFileSys.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"


namespace FWP
{
  bool createTurbine(FmTurbine* theTurbine,
                     FmTower* theTower,
                     FmNacelle* theNacelle,
                     FmGenerator* theGenerator,
                     FmGearBox* theGearBox,
                     FmShaft* theShaft,
                     FmShaft* hsShaft,
                     FmRotor* theRotor);

  bool updateTurbine(FmTurbine* theTurbine,
                     FmTower* theTower,
                     FmNacelle* theNacelle,
                     FmGenerator* theGenerator,
                     FmGearBox* theGearBox,
                     FmShaft* theShaft,
                     FmShaft* hsShaft,
                     FmRotor* theRotor);

  bool updateTower(FmTower* theTower);

  bool updateShaftProps(const FmShaft* shaft, FmBeamProperty* prop);
}


FmBladeDesign* FWP::readBladeDesign (const std::string& bladeDesignFile,
                                     FmBladeDesign* oldBladeDesign)
{
  // Read blade-design from selected file
  FmBladeDesign* newDesign = FmBladeDesign::readFromFMM(bladeDesignFile);
  if (!newDesign) return NULL;

  std::string srcBladePath = newDesign->myModelFile.getValue();
  if (oldBladeDesign)
  {
    const std::string& oldBladePath = oldBladeDesign->myModelFile.getValue();
    oldBladeDesign->erase();
    if (srcBladePath != oldBladePath)
      FmFileSys::removeDir(FFaFilePath::getBaseName(oldBladePath).append("_airfoils"));
  }

  // Copy the blade to the model's blade-folder. Create new folder if necessary
  std::string dstBladePath = bladeDesignFile;
  std::string dstBladeFolder = FmDB::getMechanismObject()->getAbsBladeFolderPath();
  if (FmFileSys::verifyDirectory(dstBladeFolder))
  {
    // Clean the directory for any existing fmm-files
    std::vector<std::string> oldBlades;
    if (FmFileSys::getFiles(oldBlades,dstBladeFolder,"*.fmm"))
      for (const std::string& bladeFile : oldBlades)
        if (!FmFileSys::deleteFile(FFaFilePath::appendFileNameToPath(dstBladeFolder,bladeFile)))
          std::cerr <<"  ** Could not delete file "<< bladeFile
                    <<" from folder "<< dstBladeFolder << std::endl;

    // Get the source blade's path and copy to folder
    dstBladePath = FFaFilePath::appendFileNameToPath(dstBladeFolder,
                                                     FFaFilePath::getFileName(srcBladePath));
    newDesign->myModelFile.setValue(dstBladePath);
    newDesign->writeToFMM(dstBladePath);
  }

  // Get the blade's airfoil paths and copy to this model's airfoil folder
  std::string srcAirfoilFolder = FFaFilePath::getBaseName(srcBladePath).append("_airfoils");
  std::string dstAirfoilFolder = FFaFilePath::getBaseName(dstBladePath).append("_airfoils");
  if (FmFileSys::verifyDirectory(dstAirfoilFolder))
  {
    std::vector<FmBladeProperty*> bprops;
    newDesign->getBladeSegments(bprops);
    for (FmBladeProperty* prop : bprops)
      if (!FmFileSys::copyFile(prop->AirFoil.getValue(),srcAirfoilFolder,dstAirfoilFolder))
        std::cerr <<"  ** Could not copy file "<< prop->AirFoil.getValue()
                  <<"\n     from folder "<< srcAirfoilFolder
                  <<"\n     to folder "<< dstAirfoilFolder << std::endl;
  }

  return newDesign;
}


bool FWP::updateTurbine (int turbineID)
{
  // Start off by finding all the sub-assemblies the turbine consists of.
  // They must already have been defined on entry (except for the blades).
  ////////////////////////////////////////////////////////////////////////

  FmTurbine* turbine = FmDB::getTurbineObject(turbineID);
  if (!turbine)
  {
    ListUI <<" *** Turbine "<< turbineID <<" was not found.\n";
    return false;
  }

  FmTower* tower;
  FmNacelle* nacelle;
  FmGenerator* generator;
  FmGearBox* gearbox;
  FmShaft* lsShaft;
  FmShaft* hsShaft;
  FmRotor* rotor;
  turbine->getParts(tower,nacelle,generator,gearbox,lsShaft,hsShaft,rotor);

  int ok = 0;
  if (nacelle)
    ok++;
  else
    ListUI <<"  ** "<< turbine->getIdString(true)
	   <<" does not have a nacelle assembly.\n";

  if (lsShaft)
    ok++;
  else
    ListUI <<"  ** "<< turbine->getIdString(true)
	   <<" does not have a shaft assembly.\n";

  if (rotor)
    ok++;
  else
    ListUI <<"  ** "<< turbine->getIdString(true)
	   <<" does not have a rotor assembly.\n";

  // Check some key geometry parameters for consistency
  //////////////////////////////////////////////////////

  if (nacelle && nacelle->M3.getValue() < 0.0)
  {
    FFaMsg::dialog("A negative value on the D2 parameter is not allowed.\n"
		   "To generate a down-stream turbine, set D2 to a positive value\n"
		   "and rotate the Turbine assembly 180 degrees about the global Z-axis.",
		   FFaMsg::ERROR);
    return false;
  }
  else if (lsShaft && lsShaft->Length.getValue() <= 0.0)
  {
    FFaMsg::dialog("The main shaft must have positive length (D3).",FFaMsg::ERROR);
    return false;
  }

  ListUI <<"===> Updating mechanims model of "
	 << turbine->getIdString(true) <<"\n";

  if (!nacelle)
  {
    generator = NULL;
    gearbox = NULL;
    lsShaft = NULL;
    hsShaft = NULL;
  }
  else if (gearbox && gearbox->Ratio.getValue() == 0.0)
    gearbox = NULL;

  if (hsShaft)
    if (!gearbox || hsShaft->Length.getValue() <= 0.0)
      if (!hsShaft->hasObjects(FmLink::getClassTypeID()))
	hsShaft = NULL;

  if (hsShaft && !generator)
    hsShaft = NULL;

  if (turbine->hasObjects(FmLink::getClassTypeID()))
    return updateTurbine(turbine,tower,nacelle,
			 generator,gearbox,lsShaft,hsShaft,rotor);
  else if (ok == 3)
    return createTurbine(turbine,tower,nacelle,
			 generator,gearbox,lsShaft,hsShaft,rotor);

  ListUI <<" *** The turbine can not be updated.\n";
  return false;
}


bool FWP::updateTurbineTower (int turbineID)
{
  FmTurbine* turbine = FmDB::getTurbineObject(turbineID);
  if (!turbine)
  {
    ListUI <<" *** Turbine "<< turbineID <<" was not found.\n";
    return false;
  }

  FmTower* tower = turbine->getTower();
  if (tower)
    return updateTower(tower);

  ListUI <<" *** "<< turbine->getIdString(true)
	 <<" does not have a tower assembly.\n";
  return false;
}


static FmBeam* newShaft (FmBase* parent, FmModelMemberBase* property = NULL)
{
  FmBeam* shaft = new FmBeam();
  shaft->setParentAssembly(parent);
  shaft->setProperty(property);
  shaft->setUserDescription("Shaft");
  shaft->connect();
  shaft->alpha2.setValue(0.005); // Default stiffness-proportional damping
  return shaft;
}


bool FWP::createTurbine (FmTurbine* theTurbine,
			 FmTower* theTower,
			 FmNacelle* theNacelle,
			 FmGenerator* theGenerator,
			 FmGearBox* theGearBox,
			 FmShaft* theShaft,
			 FmShaft* hsShaft,
			 FmRotor* theRotor)
{
  if (theTower)
    theTower->setUserDescription("Tower");

  if (theNacelle)
    theNacelle->setUserDescription("Nacelle");
  else
    return false; // A nacelle is mandatory

  if (theGenerator)
    theGenerator->setUserDescription("Generator");
  else
    return false; // A generator is mandatory

  if (theGearBox)
    theGearBox->setUserDescription("Gearbox");

  if (theShaft)
    theShaft->setUserDescription("Shaft");
  else
    return false; // A (low-speed) shaft is mandatory

  if (hsShaft) {
    theShaft->setUserDescription("Main shaft");
    hsShaft->setUserDescription("Secondary shaft");
  }

  if (theRotor)
    theRotor->setUserDescription("Rotor");
  else
    return false; // A rotor is mandatory

  // Coordinate system calculations
  //////////////////////////////////

  // Define the direction vector of the low-speed shaft
  FaVec3 rotAxis = FaMat33::makeYrotation(RAD(theShaft->Tilt.getValue()))[VX];

  // Define the hub coordinate system:
  // X-axis = {rotAxis}
  // Y-axis = {yawAxis} x {rotAxis}
  // Z-axis = {rotAxis} x {Y-axis}
  FaVec3 leftAxis(-rotAxis[1],rotAxis[0],0.0);
  leftAxis.normalize();
  FaMat33 hubCS(rotAxis,leftAxis,rotAxis^leftAxis);

  // Define the local systems for the shaft parts, relative to the hub system.
  // They must have their local Z-axis aligned with the rotation axis.
  FaMat33 shaftCS;
  shaftCS.shift(-1);

  // Find the end point of the low-speed shaft
  FaVec3 shaftEnd(0.0,0.0,theNacelle->M2.getValue());
  shaftEnd -= theNacelle->M3.getValue()*rotAxis;

  // Update the local coordinate systems for the nacelle assemblies
  FaVec3 tower(0.0,0.0, theTower ? theTower->Height.getValue() : 0.0);
  theNacelle->setLocalCS(FaMat34(tower));
  theShaft->setLocalCS(FaMat34(hubCS,shaftEnd));
  shaftEnd += theShaft->Length.getValue()*rotAxis;
  if (theGearBox)
    theGearBox->setLocalCS(FaMat34(hubCS,shaftEnd));
  else
    theGenerator->setLocalCS(FaMat34(hubCS,shaftEnd));

  FmTriad* tBase = NULL;
  FmTriad* tTop = NULL;
  FmPart*  part = NULL;
  if (theTower)
  {
    // Create the tower part
    /////////////////////////

    ListUI <<"  -> Tower base : "<< theTower->toGlobal(FaVec3())
           <<"\n     Tower height : "<< tower.z() <<" [m]\n";

    // Setting the default tower centre of gravity to 40% of the total length
    theTower->CoG.setValue(0.4*tower);

    part = new FmPart();
    part->setParentAssembly(theTower);
    part->setUserDescription("Tower");
    part->useGenericProperties.setValue(true);
    part->myGenericPartStiffType.setValue(FmPart::NODE_STIFFNESS);
    part->kt.setValue(theTower->Stiff.getValue().first);
    part->kr.setValue(theTower->Stiff.getValue().second);
    part->mass.setValue(theTower->Mass.getValue());
    part->inertia.setValue(FFaTensor3(theTower->Inertia.getValue()));
    part->setLocationCG(theTower->CoG.getValue(),theTower->Iaxes.getValue());
    part->updateLocation();
    part->connect();

    // Ground triad
    FmTriad* gnd = new FmTriad();
    gnd->setParentAssembly(theTurbine);
    gnd->setUserDescription("Ground");
    gnd->connect(FmDB::getEarthLink());

    // Triad connecting the tower to ground
    tBase = new FmTriad();
    tBase->setParentAssembly(theTower);
    tBase->setUserDescription("Tower-base");
    tBase->connect(part);

    // Joint connecting the tower to ground.
    // Using a free joint here with all DOFs fixed instead of a rigid joint.
    // This is to facilitate easy access of the reaction forces (Bug #122).
    FmFreeJoint* twrGnd = new FmFreeJoint();
    twrGnd->setParentAssembly(theTurbine);
    twrGnd->setUserDescription("Tower base");
    twrGnd->setAsMasterTriad(gnd);
    twrGnd->setAsSlaveTriad(tBase);
    twrGnd->updateLocation();
    twrGnd->connect();
    for (int dof = 0; dof < 6; dof++)
      twrGnd->setStatusForDOF(dof,FmHasDOFsBase::FIXED);

    // Master triad connecting the tower to the yaw joint
    tTop = new FmTriad(tower);
    tTop->setParentAssembly(theTower);
    tTop->setUserDescription("Tower-top");
    tTop->connect(part);
  }
  else
  {
    // No tower assembly, just create a triad that will represent the tower top
    tTop = new FmTriad();
    tTop->setParentAssembly(theTurbine);
    tTop->setUserDescription("Tower-top");
    tTop->connect();
  }

  // Create the nacelle part
  ///////////////////////////

  ListUI <<"  -> Nacelle : "<< theNacelle->toGlobal(shaftEnd) <<"\n";
  FmPart* nacelle = new FmPart();
  nacelle->setParentAssembly(theNacelle);
  nacelle->setUserDescription("Nacelle");
  nacelle->useGenericProperties.setValue(true);
  nacelle->myGenericPartStiffType.setValue(FmPart::NODE_STIFFNESS);
  nacelle->kt.setValue(theNacelle->Stiff.getValue().first);
  nacelle->kr.setValue(theNacelle->Stiff.getValue().second);
  nacelle->mass.setValue(theNacelle->Mass.getValue());
  nacelle->inertia.setValue(FFaTensor3(theNacelle->Inertia.getValue()));
  nacelle->setLocationCG(theNacelle->CoG.getValue(),theNacelle->Iaxes.getValue());
  nacelle->updateLocation();
  nacelle->connect();

  // Slave triad connecting the yaw joint to the nacelle
  FmTriad* nYaw = new FmTriad();
  nYaw->setParentAssembly(theNacelle);
  nYaw->setUserDescription("Nacelle-yaw");
  nYaw->connect(nacelle);

  // Create the yaw joint. Its DOF status is set as FIXED here.
  // The user has to assign other conditions manually, if desired.
  ////////////////////////////////////////////////////////////////

  FmRevJoint* yaw = new FmRevJoint();
  yaw->setParentAssembly(theTurbine);
  yaw->setUserDescription("Yaw");
  yaw->setAsMasterTriad(tTop);
  yaw->setAsSlaveTriad(nYaw);
  yaw->setStatusForDOF(5,FmHasDOFsBase::FIXED);
  yaw->updateLocation();
  yaw->connect();

  // Create the (low-speed) shaft part(s)
  ////////////////////////////////////////

  // Create material property object
  FmMaterialProperty* elmMat = new FmMaterialProperty();
  elmMat->setParentAssembly(theTurbine);
  elmMat->setUserDescription("Main shaft");
  elmMat->connect();

  // Create beam cross section object
  FmBeamProperty* elmProp = new FmBeamProperty();
  elmProp->setUserDescription("Main shaft");
  elmProp->setParentAssembly(theTurbine);
  elmProp->connect();

  elmProp->material.setRef(elmMat);
  if (!updateShaftProps(theShaft,elmProp))
    return false;

  FmBeam* axle = newShaft(theShaft,elmProp);

  // Slave triad connecting the shaft to the hub via a Rigid joint
  FmTriad* shaft = new FmTriad();
  shaft->setParentAssembly(theShaft);
  shaft->setUserDescription("Hub");
  shaft->setOrientation(shaftCS);
  shaft->connect();

  FmTriad* shaftStart = shaft;
  axle->setTriad(shaftStart,0);

  if (theNacelle->B1.getValue() != 0.0)
  {
    // Create the main bearing
    FaVec3 pos1(theNacelle->B1.getValue(),0.0,0.0);
    FmTriad* sl = new FmTriad(pos1);
    sl->setParentAssembly(theShaft);
    sl->setUserDescription("Shaft");
    sl->setOrientation(shaftCS);
    sl->connect();
    axle->setTriad(sl,1);

    FmTriad* ms = new FmTriad();
    ms->setParentAssembly(theNacelle);
    ms->setUserDescription("Main frame");
    ms->setGlobalCS(theShaft->toGlobal(FaMat34(shaftCS,pos1)));
    ms->connect(nacelle);

    FmRevJoint* bearing = new FmRevJoint();
    bearing->setParentAssembly(theNacelle);
    bearing->setUserDescription("Main bearing");
    bearing->setAsMasterTriad(ms);
    bearing->setAsSlaveTriad(sl);
    bearing->updateLocation();
    bearing->connect();

    axle = newShaft(theShaft,elmProp);
    axle->setTriad(sl,0);

    if (theNacelle->B2.getValue() != 0.0)
    {
      // Create second bearing
      FaVec3 pos2 = pos1 + FaVec3(theNacelle->B2.getValue(),0.0,0.0);
      FmTriad* sl = new FmTriad(pos2);
      sl->setParentAssembly(theShaft);
      sl->setUserDescription("Shaft");
      sl->setOrientation(shaftCS);
      sl->connect();
      axle->setTriad(sl,1);

      FmTriad* ms = new FmTriad();
      ms->setParentAssembly(theNacelle);
      ms->setUserDescription("Main frame");
      ms->setGlobalCS(theShaft->toGlobal(FaMat34(shaftCS,pos2)));
      ms->connect(nacelle);

      FmRevJoint* bearing = new FmRevJoint();
      bearing->setParentAssembly(theNacelle);
      bearing->setUserDescription("Bearing 2");
      bearing->setAsMasterTriad(ms);
      bearing->setAsSlaveTriad(sl);
      bearing->updateLocation();
      bearing->connect();

      axle = newShaft(theShaft,elmProp);
      axle->setTriad(sl,0);
    }
  }

  // Slave triad connecting the shaft to the generator joint,
  // or the gear input joint, if a gear box is present
  FmTriad* azimuth = new FmTriad(FaVec3(theShaft->Length.getValue(),0.0,0.0));
  azimuth->setParentAssembly(theShaft);
  azimuth->setUserDescription("Azimuth");
  azimuth->setOrientation(shaftCS);
  azimuth->connect();
  axle->setTriad(azimuth,1);

  // Master triad connecting the nacelle to the generator joint,
  // or the gear input joint, if a gear box is present
  shaft = new FmTriad();
  shaft->setParentAssembly(theGearBox ? (FmBase*)theGearBox : (FmBase*)theGenerator);
  shaft->setUserDescription("Shaft");
  shaft->setOrientation(shaftCS);
  shaft->connect(nacelle);

  // Now create the generator joint. If a gear box exists, this will be the
  // gear input joint instead, and the actual generator joint is created later.
  /////////////////////////////////////////////////////////////////////////////

  FmRevJoint* generator = new FmRevJoint();
  FmModelMemberBase* rotorDOF = generator;
  generator->setParentAssembly(theGearBox ? (FmBase*)theGearBox : (FmBase*)theGenerator);
  generator->setUserDescription(theGearBox ? "Gear input" : "Generator");
  generator->setAsSlaveTriad(azimuth);
  generator->setAsMasterTriad(shaft);
  generator->updateLocation();
  generator->connect();

  if (theGearBox)
  {
    FaVec3 hs1(theGearBox->Length.getValue(),
	       theGearBox->O1.getValue(),
	       theGearBox->O2.getValue());
    FaVec3 hsPos(theNacelle->toLocal(theGearBox->toGlobal(hs1)));

    // Create the gear box
    FmRevJoint* gearOut = new FmRevJoint();
    gearOut->setParentAssembly(theGearBox);
    gearOut->setUserDescription("Gear output");
    gearOut->connect();

    // Master triad connecting the gear box to the gear output joint
    shaft = new FmTriad(hs1);
    shaft->setParentAssembly(theGearBox);
    shaft->setUserDescription("Gear");
    shaft->connect(nacelle);
    gearOut->setAsMasterTriad(shaft);

    if (hsShaft)
    {
      // Define the local coordinate system of the high-speed shaft
      double hTilt = hsShaft->Tilt.getValue() - theShaft->Tilt.getValue();
      if (hTilt == 0.0)
	hsShaft->setLocalCS(FaMat34(hubCS,hsPos));
      else
	hsShaft->setLocalCS(FaMat34(hubCS*FaMat33::makeYrotation(RAD(hTilt)),hsPos));

      // Create material property object
      elmMat = new FmMaterialProperty();
      elmMat->setParentAssembly(theTurbine);
      elmMat->setUserDescription("Secondary shaft");
      elmMat->connect();

      // Create beam cross section object
      elmProp = new FmBeamProperty();
      elmProp->setParentAssembly(theTurbine);
      elmProp->setUserDescription("Secondary shaft");
      elmProp->connect();

      elmProp->material.setRef(elmMat);
      if (!updateShaftProps(hsShaft,elmProp))
	return false;

      // Create the high-speed shaft
      axle = newShaft(hsShaft,elmProp);

      // Slave triad connecting the high-speed shaft to the gear output joint
      FmTriad* slave = new FmTriad();
      slave->setParentAssembly(hsShaft);
      slave->setUserDescription("Gear");
      slave->setOrientation(shaftCS);
      slave->connect();
      axle->setTriad(slave,0);
      gearOut->setAsSlaveTriad(slave);
      shaft->setOrientation(theNacelle->toLocal(slave->getGlobalCS()).direction());

      FaVec3 hs2(hsShaft->Length.getValue(),0.0,0.0);
      theGenerator->setLocalCS(theNacelle->toLocal(hsShaft->toGlobal(FaMat34(hs2))));
    }
    else
    {
      shaft->setOrientation(theNacelle->toLocal(theGearBox->toGlobal(shaftCS)));

      // Disconnected slave triad of the gear output joint
      shaft = new FmTriad(hs1);
      shaft->setParentAssembly(theGearBox);
      shaft->setOrientation(shaftCS);
      shaft->setUserDescription("Gear");
      shaft->connect();
      gearOut->setAsSlaveTriad(shaft);

      // Add some mass to make the model solvable
      shaft->setAddedMass(0.1);
      FFaMsg::dialog("The secondary shaft was assigned zero length (D5)\n"
                     "and has therefore been eliminated from the model.\n"
                     "The slave triad of the generator, " +
                     gearOut->getIdString(true) + ", has been assigned a small "
                     "mass (0.1) to ensure that the model will be solvable.\n"
                     "\nPlease verify that this mass value is appropriate for "
                     "the current design.",FFaMsg::WARNING);
    }

    gearOut->updateLocation();

    FmGear* gear = new FmGear();
    gear->setParentAssembly(theGearBox);
    gear->setTransmissionRatio(theGearBox->Ratio.getValue());
    gear->connect(generator,gearOut);

    if (hsShaft)
    {
      // Master triad connecting the nacelle to the generator joint
      shaft = new FmTriad();
      shaft->setParentAssembly(theGenerator);
      shaft->setUserDescription("Generator");
      shaft->connect(nacelle);

      // Slave triad connecting the high-speed shaft to the generator joint
      FmTriad* slave = new FmTriad(FaVec3(hsShaft->Length.getValue(),0.0,0.0));
      slave->setParentAssembly(hsShaft);
      slave->setOrientation(shaftCS);
      slave->setUserDescription("Generator");
      slave->connect();
      axle->setTriad(slave,1);
      shaft->setOrientation(theNacelle->toLocal(slave->getGlobalCS()).direction());

      // Create the generator joint
      generator = new FmRevJoint();
      generator->setParentAssembly(theGenerator);
      generator->setUserDescription("Generator");
      generator->setAsMasterTriad(shaft);
      generator->setAsSlaveTriad(slave);
      generator->updateLocation();
      generator->connect();
    }
    else
      generator = gearOut;
  }

  // The generator DOF is locked in eigenvalue analysis and initial equilibrium
  generator->setStatusForDOF(5,FmHasDOFsBase::FREE_DYNAMICS);

  // Create the hub
  //////////////////

  ListUI <<"  -> Platform reference height : "<< theTurbine->ptfmRef.getValue()
	 <<" [m]\n     Hub radius : "<< theRotor->HubDiam.getValue()*0.5
	 <<" [m]\n     Precone angle : "<< theRotor->PreCone.getValue()
	 <<" [deg]\n";

  FaVec3 apex(theShaft->toGlobal(FaVec3(-theRotor->HubApex.getValue(),0.0,0.0)));
  theRotor->setLocalCS(FaMat34(hubCS,theTurbine->toLocal(apex)));
  ListUI <<"  -> Hub apex : "<< apex <<"\n";

  FmPart* hub = new FmPart();
  hub->setParentAssembly(theRotor);
  hub->setUserDescription("Hub");
  hub->useGenericProperties.setValue(true);
  hub->myGenericPartStiffType.setValue(FmPart::NODE_STIFFNESS);
  hub->kt.setValue(theRotor->Stiff.getValue().first);
  hub->kr.setValue(theRotor->Stiff.getValue().second);
  hub->mass.setValue(theRotor->Mass.getValue());
  hub->inertia.setValue(FFaTensor3(theRotor->Inertia.getValue()));
  hub->setLocationCG(theRotor->CoG.getValue(),theRotor->Iaxes.getValue());
  hub->updateLocation();
  hub->connect();

  // Master triad at the hub apex
  FmTriad* hubApex = new FmTriad(FaVec3(theRotor->HubApex.getValue(),0.0,0.0));
  hubApex->setParentAssembly(theRotor);
  hubApex->setUserDescription("Hub");
  hubApex->setOrientation(shaftCS);
  hubApex->connect(hub);

  // Rigid joint connecting the hub and shaft parts
  FmRigidJoint* hubShaft = new FmRigidJoint();
  hubShaft->setParentAssembly(theRotor);
  hubShaft->setUserDescription("Hub-Shaft");
  hubShaft->setAsMasterTriad(hubApex);
  hubShaft->setAsSlaveTriad(shaftStart);
  hubShaft->updateLocation();
  hubShaft->connect();

  // Store away some topology data needed by the Dynamics Solver
  std::vector<FmModelMemberBase*> top(5);
  top[0] = tBase;
  top[1] = nYaw;
  top[2] = hubApex;
  top[3] = hub;
  top[4] = rotorDOF;
  theTurbine->topology.setPtrs(top);

  // Now generate the blades
  ///////////////////////////

  int nBlade = theTurbine->nBlade.getValue();
  for (int b = 0; b < nBlade; b++)
  {
    // Create a new sub-assembly for each blade
    FmBlade* blade = new FmBlade();
    blade->setParentAssembly(theRotor);
    blade->setUserDescription(FFaNumStr("Blade %d",b+1));
    blade->connect();
  }

  std::vector<FmBladeProperty*> blSeg;
  FmBladeDesign* blDef = theTurbine->getBladeProperties(blSeg);
  ListUI <<"  -> Number of blades : "<< nBlade
	 <<" ("<< (int)blSeg.size() <<" segments)\n";

  if (!createBladeElements(theRotor,hub,blDef,blSeg))
    return false;

  // Create regulation system, if wanted
  ///////////////////////////////////////

  if (!createRegulationSystem(theTurbine,theRotor,generator))
    FFaMsg::dialog("Could not create wind turbine control system.\n\n"
                   "Please check that you have enabled the plug-in containing\n"
                   "the pitch- and torque controllers, and try again:\n"
                   "1. Choose 'Plug-Ins..' from the 'Tools' menu.\n"
                   "2. Check WTCtrl.dll.\n"
                   "3. Uncheck any other plug-in containing user-defined functions."
                   "4. Click 'OK' and then 'Update turbine' in the 'Turbine Definition' dialog."
                   "\n\nPlease note that only one user-defined functions plug-in"
                   " can be loaded at the same time.",
                   FFaMsg::ERROR);

  // Finally, generate the predefined graphs
  ///////////////////////////////////////////

  return createGraphs(hubApex,tTop,generator);
}


bool FWP::updateTurbine (FmTurbine* theTurbine,
			 FmTower* theTower,
			 FmNacelle* theNacelle,
			 FmGenerator* theGenerator,
			 FmGearBox* theGearBox,
			 FmShaft* theShaft,
			 FmShaft* hsShaft,
			 FmRotor* theRotor)
{
  // Coordinate system calculations
  //////////////////////////////////

  // Define the direction vector of the low-speed shaft
  FaVec3 rotAxis(1.0,0.0,0.0);
  if (theShaft)
    rotAxis = FaMat33::makeYrotation(RAD(theShaft->Tilt.getValue()))[VX];

  // Define the hub coordinate system:
  // X-axis = {rotAxis}
  // Y-axis = {yawAxis} x {rotAxis}
  // Z-axis = {rotAxis} x {Y-axis}
  FaVec3 leftAxis(-rotAxis[1],rotAxis[0],0.0);
  leftAxis.normalize();
  FaMat33 hubCS(rotAxis,leftAxis,rotAxis^leftAxis);

  // Define the local systems for the shaft parts, relative to the hub system.
  // They must have their local Z-axis aligned with the rotation axis.
  FaMat33 shaftCS;
  shaftCS.shift(-1);

  const char* towerMsg = "The tower height has been set to zero.\n"
    "That implies that the tower assembly will be erased from the model,\n"
    "and it cannot be restored by reentering a non-zero tower height later.\n"
    "\nProceed ?";

  FaVec3 ttop;
  bool erasedTower = false;
  if (theTower)
  {
    if (theTower->Height.getValue() > 0.0)
      ttop.z(theTower->Height.getValue());
    else if (FFaMsg::dialog(towerMsg,FFaMsg::OK_CANCEL))
    {
      erasedTower = true;
      theTower->erase();
      theTower = NULL;

      // Update the location of the newly created yaw master triad
      std::vector<FmModelMemberBase*> objs;
      FmDB::getAllOfType(objs,FmRevJoint::getClassTypeID(),theTurbine);
      if (objs.size() > 0)
      {
	FmTriad* top = static_cast<FmRevJoint*>(objs[0])->getItsMasterTriad();
	if (top) top->setLocalCS(FaMat34());
      }
    }
    else
      return false; // Cancel
  }

  if (theNacelle)
  {
    // Find the end point of the low-speed shaft
    FaVec3 shaftEnd;
    shaftEnd.z(theNacelle->M2.getValue());
    shaftEnd -= theNacelle->M3.getValue()*rotAxis;

    // Update the local coordinate systems for the nacelle assemblies
    if (theTower || erasedTower)
      theNacelle->setLocalCS(FaMat34(ttop));

    if (theShaft)
    {
      theShaft->setLocalCS(FaMat34(hubCS,shaftEnd));
      shaftEnd += theShaft->Length.getValue()*rotAxis;
    }

    if (theGearBox)
      theGearBox->setLocalCS(FaMat34(hubCS,shaftEnd));
    else if (theGenerator)
      theGenerator->setLocalCS(FaMat34(hubCS,shaftEnd));
  }

  // Geometry and property update
  ////////////////////////////////

  std::vector<FmLink*>  parts;
  std::vector<FmTriad*> triads;
  if (theTower)
  {
    FmDB::getAllLinks(parts,theTower);
    if (parts.empty())
    {
      ListUI <<" *** Empty tower assembly. Cannot update\n";
      return false;
    }

    // Update the tower properties (but only if it still is a Generic Part)
    FmPart* tower = dynamic_cast<FmPart*>(parts.front());
    if (parts.size() == 1 && tower && tower->isGenericPart())
    {
      // Setting the default tower centre of gravity to 40% of the total length
      theTower->CoG.setValue(0.4*ttop);

      /* Bugfix #199: Do not update these properties as the user may have edited
	 the mass and stiffness properties of the tower manually in the meantime
      tower->kt.setValue(theTower->Stiff.getValue().first);
      tower->kr.setValue(theTower->Stiff.getValue().second);
      tower->mass.setValue(theTower->Mass.getValue());
      tower->inertia.setValue(FFaTensor3(theTower->Inertia.getValue()));
      tower->setLocationCG(theTower->CoG.getValue(),theTower->Iaxes.getValue());
      */
    }
    else if (parts.size() > 1)
    {
      // If the tower already consists of several beam elements, we must
      // update the tower if its height has been changed (by more than 1%)
      double deltaH = theTower->Height.getValue() - theTower->getTotalLength();
      if (fabs(deltaH) > 0.01*theTower->Height.getValue())
	if (!updateTower(theTower))
	  return false;
    }

    // Master triad connecting the tower to the yaw joint
    FmDB::getAllTriads(triads,theTower);
    if (triads.size() > 1 && triads[1]->isMasterTriad(true))
    {
      FaMat34 topCS = triads[1]->getGlobalCS();
      topCS[VW] = theTower->toGlobal(ttop);
      triads[1]->setGlobalCS(topCS);
      triads[1]->onChanged();
    }
  }

  FmLink* nacelle = NULL;
  if (theNacelle)
  {
    FmDB::getAllLinks(parts,theNacelle);
    if (parts.empty())
    {
      ListUI <<" *** Empty nacelle assembly. Cannot update.\n";
      return false;
    }

    nacelle = parts.front();
    /* Bugfix #199: Do not update these properties as the user may have edited
       the mass and stiffness properties of the nacelle manually in the meantime
    if (nacelle->useGenericProperties.getValue())
    {
      // Update the nacelle properties
      nacelle->kt.setValue(theNacelle->Stiff.getValue().first);
      nacelle->kr.setValue(theNacelle->Stiff.getValue().second);
      nacelle->mass.setValue(theNacelle->Mass.getValue());
      nacelle->inertia.setValue(FFaTensor3(theNacelle->Inertia.getValue()));
      nacelle->setLocationCG(theNacelle->CoG.getValue(),theNacelle->Iaxes.getValue());
    }
    */
  }

  triads.clear();
  if (theShaft)
  {
    // Main shaft and associated triads
    FmDB::getAllLinks(parts,theShaft);
    if (!parts.empty())
      parts[0]->getTriads(triads);
  }

  FmSMJointBase* joint = NULL;
  if (theNacelle && theNacelle->B1.getValue() != 0.0)
  {
    // Update location of the main bearing
    FaVec3 pos1(theNacelle->B1.getValue(),0.0,0.0);
    if (triads.size() > 1)
    {
      triads[1]->setTranslation(pos1);
      triads[1]->onChanged();
      joint = dynamic_cast<FmSMJointBase*>(triads[1]->getJointWhereSlave());
      if (joint && theShaft)
      {
	FmTriad* master = joint->getItsMasterTriad();
	FaMat34 msPos(theShaft->toGlobal(FaMat34(shaftCS,pos1)));
	master->setLocalCS(nacelle->getGlobalCS().inverse()*msPos);
	master->onChanged();
      }
    }
    if (parts.size() > 1)
    {
      parts[1]->getTriads(triads);
      parts.erase(parts.begin()+1);
    }

    if (theNacelle->B2.getValue() != 0.0)
    {
      // Update location of the second bearing
      FaVec3 pos2 = FaVec3(theNacelle->B2.getValue(),0.0,0.0);
      if (triads.size() > 1)
      {
	triads[1]->setTranslation(pos1+pos2);
	triads[1]->onChanged();
	joint = dynamic_cast<FmSMJointBase*>(triads[1]->getJointWhereSlave());
	if (joint && theShaft)
	{
	  FmTriad* master = joint->getItsMasterTriad();
	  FaMat34 msPos(theShaft->toGlobal(FaMat34(shaftCS,pos1+pos2)));
	  master->setLocalCS(nacelle->getGlobalCS().inverse()*msPos);
	  master->onChanged();
	}
      }
      if (parts.size() > 1)
	parts[1]->getTriads(triads);
    }
  }

  // Slave triad connecting the shaft to the generator joint,
  // or the gear input joint, if a gear box is present
  FmRevJoint* generator = NULL;
  if (triads.size() > 1)
  {
    if (theShaft)
    {
      triads[1]->setTranslation(FaVec3(theShaft->Length.getValue(),0.0,0.0));
      triads[1]->onChanged();
    }
    generator = dynamic_cast<FmRevJoint*>(triads[1]->getJointWhereSlave());
    if (generator)
    {
      // Master triad connecting the nacelle to the generator joint,
      // or the gear input joint, if a gear box is present
      FmTriad* master = generator->getItsMasterTriad();
      FaMat34 msPos(shaftCS,FaVec3());
      if (theGearBox)
	msPos = theGearBox->toGlobal(msPos);
      else if (theGenerator)
	msPos = theGenerator->toGlobal(msPos);
      master->setLocalCS(nacelle->getGlobalCS().inverse()*msPos);
      master->onChanged();
    }
  }

  if (theGearBox)
  {
    // Update the gear box location
    std::vector<FmModelMemberBase*> allObjs;
    FmDB::getAllOfType(allObjs,FmRevJoint::getClassTypeID(),theGearBox);
    if (allObjs.size() > 1)
    {
      FaVec3 hs1(theGearBox->Length.getValue(),
		 theGearBox->O1.getValue(),
		 theGearBox->O2.getValue());
      FaVec3 hsPos = theNacelle->toLocal(theGearBox->toGlobal(hs1));

      FmRevJoint* gearOut = static_cast<FmRevJoint*>(allObjs[1]);
      // Master triad connecting the gear box to the gear output joint
      FmTriad* master = gearOut->getItsMasterTriad();
      master->setTranslation(theNacelle->toLocal(theGearBox->toGlobal(hs1)) -
			     nacelle->getTranslation());

      FmTriad* slave = gearOut->getSlaveTriad();
      if (hsShaft)
      {
	// Define the local coordinate system of the high-speed shaft
	double hTilt = hsShaft->Tilt.getValue();
	if (theShaft)
	  hTilt -= theShaft->Tilt.getValue();
	if (hTilt == 0.0)
	  hsShaft->setLocalCS(FaMat34(hubCS,hsPos));
	else
 	  hsShaft->setLocalCS(FaMat34(hubCS*FaMat33::makeYrotation(RAD(hTilt)),hsPos));

	master->setOrientation(theNacelle->toLocal(slave->getGlobalCS()).direction());
	master->onChanged();

	FaVec3 hs2(hsShaft->Length.getValue(),0.0,0.0);
	if (hs2.x() <= 0.0) hs2.x(0.1); // Cannot "switch off" the hs-shaft
	FmDB::getAllOfType(allObjs,FmRevJoint::getClassTypeID(),theGenerator);
	if (allObjs.size() > 0)
	{
	  // Slave triad connecting the high-speed shaft to the generator joint
	  generator = static_cast<FmRevJoint*>(allObjs[0]);
	  slave = generator->getSlaveTriad();
	  slave->setTranslation(hs2);
	  slave->onChanged();

	  // Master triad connecting the nacelle to the generator joint
	  master = generator->getItsMasterTriad();
	  master->setTranslation(theNacelle->toLocal(hsShaft->toGlobal(hs2)) -
				 nacelle->getTranslation());
	  master->setOrientation(theNacelle->toLocal(slave->getGlobalCS()).direction());
	  master->onChanged();

	  generator->updateLocation();
	}
	theGenerator->setLocalCS(theNacelle->toLocal(hsShaft->toGlobal(FaMat34(hs2))));
      }
      else
      {
	master->setOrientation(theNacelle->toLocal(theGearBox->toGlobal(shaftCS)));
	// Disconnected slave triad of the gear output joint
	slave->setGlobalCS(master->getGlobalCS());
      }

      gearOut->updateLocation();
    }
  }

  // Update the hub
  //////////////////

  if (!theRotor)
  {
    ListUI <<" *** Cannot update a turbine with no rotor.\n";
    return false;
  }

  if (theShaft)
  {
    FaVec3 apex(theShaft->toGlobal(FaVec3(-theRotor->HubApex.getValue(),0.0,0.0)));
    theRotor->setLocalCS(FaMat34(hubCS,theTurbine->toLocal(apex)));
  }

  // Update the hub triad location in case the HubApex distance has been changed
  FmDB::getAllTriads(triads,theRotor);
  if (!triads.empty())
    triads[0]->setTranslation(FaVec3(theRotor->HubApex.getValue(),0.0,0.0));

  /* Bugfix #199: Do not update the hub properties, as the user may have edited
     the mass and stiffness properties of the hub manually in the meantime
  FmLink* hub = NULL;
  FmDB::getAllLinks(parts,theRotor);
  if (!parts.empty())
  {
    hub = parts.front();
    if (hub->useGenericProperties.getValue())
    {
      hub->kt.setValue(theRotor->Stiff.getValue().first);
      hub->kr.setValue(theRotor->Stiff.getValue().second);
      hub->mass.setValue(theRotor->Mass.getValue());
      hub->inertia.setValue(FFaTensor3(theRotor->Inertia.getValue()));
      hub->setLocationCG(theRotor->CoG.getValue(),theRotor->Iaxes.getValue());
    }
  }
  */

  theTurbine->onChanged(); // Update object list view in case name has changed

  // Now update the blades
  /////////////////////////

  std::vector<FmBladeProperty*> blSeg;
  FmBladeDesign* blDef = theTurbine->getBladeProperties(blSeg);
  ListUI <<"  -> Number of blade segments : "<< (int)blSeg.size() <<"\n";

  if (!updateBladeElements(theRotor,blDef,blSeg))
    return false;

  // Create regulation system, if wanted, or delete it if not wanted
  ///////////////////////////////////////////////////////////////////

  return createRegulationSystem(theTurbine,theRotor,generator);
}


bool FWP::updateTower (FmTower* tower)
{
  // Fetch the current tower elements, there should be at least one
  std::vector<FmLink*> elms;
  FmDB::getAllLinks(elms,tower);

  // Fetch the two end triads, it should always be the first two
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,tower);
  if (elms.size() < 1 || triads.size() < 2)
  {
    ListUI <<" *** Empty tower assembly. Cannot update.\n";
    return false;
  }

  FmTriad* tbot = triads[0];
  FmTriad* ttop = triads[1];
  if (!tbot->isSlaveTriad(true) || !ttop->isMasterTriad(true))
  {
    ListUI <<" *** The turbine tower topology is corrupt. Cannot update.\n"
           <<"     You have to delete the turbine assembly and start over.\n";
    return false;
  }

  // Erase the current tower if it was a generic part
  if (dynamic_cast<FmPart*>(elms.front()))
  {
    elms.front()->erase();
    elms.erase(elms.begin());
  }

  // Check if the total number of elements is to be changed
  size_t totElms = 0, nElm = elms.size();
  for (const FmTwrSegment& seg : tower->segments.getValue())
    if (seg.H > 0.0) totElms += seg.N;

  FmBeam* beam = NULL;
  if (totElms != nElm)
  {
    // Detach the current top triad. This must be done here before
    // changing the the length of the element it was attached to.
    ttop->disconnect();
    ttop->connect();
    if (!elms.empty() && (beam = dynamic_cast<FmBeam*>(elms.back())))
      beam->setTriad(NULL,1);
  }

  // Now create/update the beam elements of the tower
  ////////////////////////////////////////////////////

  FmTriad* node = tbot;
  double  wth2 = 2.0*tower->Thick.getValue();
  FaMat33 elmCS;
  FaVec3  Rnode;
  size_t  iel = 0, t = 1;
  int updElms = 0, newElms = 0;
  FmBeamProperty* prop = NULL;

  // Loop over tower segments
  const FmTwrSegmentVec& twrSeg = tower->segments.getValue();
  FmTwrSegmentVec::const_iterator tit, pit = twrSeg.end();
  for (tit = twrSeg.begin(); tit != twrSeg.end(); pit = tit, ++tit, t++)
    for (int e = 0; e < tit->N && tit->H > 0.0; e++, iel++)
    {
      double Length = tit->H/(double)tit->N;
      double Rnext  = Rnode.z() + Length;
      if (elms.size() <= iel)
      {
        newElms++; // Increasing the number of tower elements
        FmTriad* node0 = node;
        node = new FmTriad(FaVec3(0.0,0.0,Rnext));
        node->setParentAssembly(tower);
        node->connect();
        beam = newShaft(tower);
        beam->connect(node0,node);
        elms.push_back(beam);
      }
      else
      {
	updElms++; // Update the existing beam element
	beam = static_cast<FmBeam*>(elms[iel]);
	node = beam->getSecondTriad();
	if (!node)
	{
	  // This is the current top element - its second triad (the top triad)
	  // was disconnected initially, so add a new one at this point
	  node = new FmTriad(FaVec3(0.0,0.0,Rnext));
	  node->setParentAssembly(tower);
	  node->connect();
	  beam->setTriad(node,1);
	}
	else
	{
	  // Update location of second triad
	  node->setLocalCS(FaMat34(FaVec3(0.0,0.0,Rnext)));
	  ListUI <<"     Tower "<< node->getIdString(true)
		 <<" relocated to "<< node->getGlobalTranslation() <<"\n";
	}
      }
      beam->setUserDescription(FFaNumStr("Tower section %d",t));

      // Define or update the properties of this beam
      if (e == 0 || tit != twrSeg.begin())
	prop = dynamic_cast<FmBeamProperty*>(beam->getProperty());
      if (prop && tit != twrSeg.begin())
      {
	std::vector<FmBeam*> beams;
	prop->getReferringObjs(beams,"myProp");
	if (beams.size() > 1) prop = NULL;
      }
      if (!prop)
      {
	prop = new FmBeamProperty();
	prop->setParentAssembly(tower->getParentAssembly());
	prop->connect();
      }
      prop->setUserDescription(FFaNumStr("Tower section %d",t));

      if (tit != twrSeg.begin())
      {
	// Interpolate the diameter based on mid-point height of current element
	double xi = (double)(2*e+1)/(double)(2*tit->N);
	prop->crossSectionType.setValue(FmBeamProperty::PIPE);
	prop->Do.setValue(tit->D*xi + pit->D*(1.0-xi));
	prop->Di.setValue(tit->D*xi + pit->D*(1.0-xi) - wth2);
	prop->material.setRef(tower->material.getPointer());
	prop->updateDependentValues();
	prop->onChanged();
      }
      else if (e == 0)
      {
	// The first segment is assumed to have constant diameter,
	// so let all elements refer to the same property object
	prop->crossSectionType.setValue(FmBeamProperty::PIPE);
	prop->Do.setValue(tit->D);
	prop->Di.setValue(tit->D - wth2);
	prop->material.setRef(tower->material.getPointer());
	prop->updateDependentValues();
	prop->onChanged();
      }
      beam->setProperty(prop);
      beam->onChanged();

      Rnode.z(Rnext);
    }

  if (iel != nElm)
  {
    // The number of tower elements has changed, reconnect the top triad
    beam->getSecondTriad()->erase();
    ttop->connect();
    beam->setTriad(ttop,1);
    beam->onChanged();
  }

  // Erase superfluous tower elements in case the number has been reduced
  for (; elms.size() > iel; --newElms)
  {
    beam = static_cast<FmBeam*>(elms.back());
    prop = dynamic_cast<FmBeamProperty*>(beam->getProperty());
    if (prop)
    {
      std::vector<FmBeam*> beams;
      beam->getProperty()->getReferringObjs(beams,"myProp");
      if (beams.size() > 1) prop = NULL;
    }
    beam->getTriads(triads);
    beam->erase();
    elms.pop_back();
    if (prop)
      prop->erase();
    if (triads.size() > 1)
      triads.back()->erase();
    if (elms.size()+1 == iel)
      triads.front()->erase();
  }

  ListUI <<"  -> Updated "<< updElms <<" tower elements.\n";
  if (newElms > 0)
    ListUI <<"     Created "<< newElms <<" tower elements.\n";
  else if (newElms < 0)
    ListUI <<"     Removed "<< -newElms <<" old tower elements.\n";

  return true;
}


bool FWP::updateShaftProps (const FmShaft* shaft, FmBeamProperty* prop)
{
  if (!prop) return false;

  prop->crossSectionType.setValue(FmBeamProperty::PIPE);
  prop->Do.setValue(shaft->Do.getValue());
  prop->Di.setValue(shaft->Di.getValue());
  prop->updateDependentValues();

  ListUI <<"  -> "<< prop->getIdString(true)
	 <<" :\n\tA\t= "<< prop->A.getValue()
	 <<" [m^2]\n\tIy = Iz\t= "<< prop->Iy.getValue()
	 <<" [m^4]\n\tIp\t= "<< prop->Ip.getValue() <<"\n";

  if (prop->material.isNull()) return false;

  prop->material->updateProperties(shaft->Rho.getValue(),
				   shaft->E.getValue(),
				   shaft->G.getValue());

  ListUI <<"\tRho\t= "<< shaft->Rho.getValue()
	 <<" [kg/m^3]\n\tE\t= "<< shaft->E.getValue()
	 <<" [N/m^2]\n\tG\t= "<< shaft->G.getValue()
	 <<" [N/m^2]\n";

  return true;
}
