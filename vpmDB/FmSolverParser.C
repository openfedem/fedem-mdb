// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSolverParser.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmUserDefinedElement.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmHPBase.H"
#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmDofMotion.H"
#include "vpmDB/FmDofLoad.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmTire.H"
#include "vpmDB/FmRoad.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmGenericDBObject.H"
#ifdef FT_HAS_EXTCTRL
#include "vpmDB/FmExternalCtrlSys.H"
#endif
#include "vpmDB/FmControlAdmin.H"
#include "vpmDB/FmfSpline.H"
#include "vpmDB/FmfDeviceFunction.H"
#include "vpmDB/FmFrictionBase.H"
#include "FFlLib/FFlUtils.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


FmSolverParser::FmSolverParser(const char* fileName)
{
  myFile = fopen(fileName,"w");
}


FmSolverParser::~FmSolverParser()
{
  if (myFile) fclose(myFile);

  FmSimulationModelBase::relPathCorrection.clear();
}


void FmSolverParser::setRelPathCorrection(const std::string& path)
{
  FmSimulationModelBase::relPathCorrection = myRelPathCorrection = path;
}


bool FmSolverParser::preSimuleCheck()
{
  int errorCount = 0;

  errorCount += FmJointBase::checkJoints();
  errorCount += FmPart::checkParts();
  errorCount += FmTriad::checkTriads();
  errorCount += FmAxialSpring::checkAxialSprings();
  errorCount += FmAxialDamper::checkAxialDampers();
  errorCount += FmJointSpring::checkJointSprings();
  errorCount += FmJointDamper::checkJointDampers();
  errorCount += FmDofMotion::checkMotions();
  errorCount += FmfSpline::checkSplines();
  errorCount += FmControlAdmin::checkControl();

  std::vector<FmfDeviceFunction*> allDevFuncs;
  FmDB::getAllDeviceFunctions(allDevFuncs);
  for (FmfDeviceFunction* func : allDevFuncs)
    if (!func->checkFileValidity())
      errorCount++;

  if (errorCount > 0)
  {
    ListUI <<"\n---> A total of "<< errorCount <<" errors were found.\n";
    return false;
  }

  FmfSpline::setAllSplineICODE();
  return true;
}


int FmSolverParser::writeFullFile()
{
  if (!myFile) return 999;

  FmEngine::betaFeatureEngines.clear();

  FmTurbine* turbine = FmDB::getTurbineObject();

  std::vector<FmPart*> gageParts;
  int nextBaseId = FmDB::getFreeBaseID();
  int err = writeHeading();
  err += writeEnvironment();
  err += FmDB::getMechanismObject()->printSolverEntry(myFile);
  err += writeAllOfType(FmTriad::getClassTypeID());
  err += writeParts(gageParts);
  err += writeBeams(nextBaseId);
  err += writeAllOfType(FmUserDefinedElement::getClassTypeID());
  err += writeAllOfType(FmTire::getClassTypeID());
  err += writeAllOfType(FmRoad::getClassTypeID());
  err += writeSprings();
  err += writeAllOfType(FmAxialSpring::getClassTypeID());
  err += writeAllOfType(FmAxialDamper::getClassTypeID());
  err += writeAllOfType(FmJointDamper::getClassTypeID());
  err += writeJoints();
  err += writeAllOfType(Fm1DMaster::getClassTypeID());
  err += writeAllOfType(FmHPBase::getClassTypeID());
  err += writeAllOfType(FmGenericDBObject::getClassTypeID());
  err += writeAllOfType(FmBeamProperty::getClassTypeID());
  if (turbine) // This allows for only one turbine in the model
    err += turbine->printSolverEntry(myFile);
  err += writeAllOfType(FmLoad::getClassTypeID());
  err += writeAllOfType(FmDofLoad::getClassTypeID());
  err += writeAllOfType(FmDofMotion::getClassTypeID());
  err += writeAdditionalMasses();
  err += FmControlAdmin::printControl(myFile,nextBaseId);
#ifdef FT_HAS_EXTCTRL
  err += writeAllOfType(FmExternalCtrlSys::getClassTypeID());
#endif
  err += writeSensors();
  err += writeAllOfType(FmEngine::getClassTypeID());
  err += writeAllOfType(FmParamObjectBase::getClassTypeID());
  err += writeAllOfType(FmSpringChar::getClassTypeID());
  int nros = gageParts.empty() ? 0 : writeRosettes(gageParts);

  fclose(myFile);
  myFile = NULL;

  if (turbine)
    err += turbine->writeAeroDynFile(FFaFilePath::appendFileNameToPath(myRDBPath,"fedem_aerodyn.ipt"));

  return err > 0 ? -err : nros;
}


int FmSolverParser::writeHeading()
{
  fprintf(myFile,"&HEADING\n");
  fprintf(myFile,"  modelFile = '%s'\n",
	  FmDB::getMechanismObject()->getModelFileName().c_str());
  fprintf(myFile,"  version = 3.0\n");
  fprintf(myFile,"/\n\n");
  return 0;
}


int FmSolverParser::writeEnvironment()
{
  const FaVec3& grav = FmDB::getGrav();
  FmSeaState* seastate = FmDB::getSeaStateObject(false);
  if (grav.isZero() && !seastate) return 0; // No environment in this model

  fprintf(myFile,"&ENVIRONMENT\n");
  fprintf(myFile,"  gravity  =%17.9e %17.9e %17.9e\n",grav[0],grav[1],grav[2]);
  if (seastate)
  {
    FFaString mDesc = FmDB::getMechanismObject()->getUserDescription();
    fprintf(myFile,"  rhoWater =%17.9e\n",seastate->waterDensity.getValue());
    if (mDesc.hasSubString("#MudDensity"))
      ListUI <<"\n---> WARNING: Ignoring #MudDensity in the"
	     <<" Model description field of the Model Preferences dialog box."
	     <<"\n     Set this in the Riser property view instead.\n";
    if (mDesc.hasSubString("#MarineGrowth"))
      ListUI <<"\n---> WARNING: Ignoring #MarineGrowth in the"
	     <<" Model description field of the Model Preferences dialog box."
	     <<"\n     Set this in the Sea Environment dialog box instead.\n";
    double rhoMG = seastate->growthDensity.getValue();
    double thickMG = seastate->growthThickness.getValue();
    const std::pair<double,double>& limMG = seastate->growthLimit.getValue();
    if (rhoMG > 0.0 && thickMG > 0.0 && limMG.first > limMG.second)
    {
      fprintf(myFile,"  rhoGrowth =%17.9e\n",rhoMG);
      fprintf(myFile,"  tGrowth  =%17.9e\n",thickMG);
      fprintf(myFile,"  zGrowth  =%17.9e %17.9e\n",limMG.second,limMG.first);
    }
    fprintf(myFile,"  sea0     =%17.9e\n",seastate->meanSeaLevel.getValue());
    fprintf(myFile,"  seaDepth =%17.9e\n",seastate->seaDepth.getValue());

    if (seastate->waterDensity.getValue() > 0.0)
    {
      FaMat34 seaCS = FmDB::getSeaCS();
      fprintf(myFile,"  seaCS    =%17.9e %17.9e %17.9e %17.9e\n",
	      seaCS[0][0],seaCS[1][0],seaCS[2][0],seaCS[3][0]);
      fprintf(myFile,"            %17.9e %17.9e %17.9e %17.9e\n",
	      seaCS[0][1],seaCS[1][1],seaCS[2][1],seaCS[3][1]);
      fprintf(myFile,"            %17.9e %17.9e %17.9e %17.9e\n",
	      seaCS[0][2],seaCS[1][2],seaCS[2][2],seaCS[3][2]);
    }

    // Beta feature: Wave theory option
    if (mDesc.hasSubString("#waveTheory"))
      fprintf(myFile,"  waveTheory = %d\n",mDesc.getIntAfter("waveTheory"));
    FmMathFuncBase* wFunc = seastate->waveFunction.getPointer();
    if (wFunc) fprintf(myFile,"  waveFunction = %d\n",wFunc->getBaseID());
    FmMathFuncBase* cFunc = seastate->currFunction.getPointer();
    if (cFunc) fprintf(myFile,"  currFunction = %d\n",cFunc->getBaseID());
    FmMathFuncBase* dFunc = cFunc ? seastate->currentDir.getPointer() : NULL;
    if (dFunc) fprintf(myFile,"  currDirFunction = %d\n",dFunc->getBaseID());
    FmEngine* scale = cFunc ? seastate->currScale.getPointer() : NULL;
    if (scale) fprintf(myFile,"  currScaleEngine = %d\n",scale->getBaseID());

    if (wFunc && (scale = seastate->hdfScale.getPointer()))
      fprintf(myFile,"  hdfScaleEngine = %d\n",scale->getBaseID());
  }
  fprintf(myFile,"/\n\n");
  return 0;
}


int FmSolverParser::writeAdditionalMasses()
{
  int err = 0;

  std::vector<FmTriad*> allTriads;
  FmDB::getAllTriads(allTriads);

  for (FmTriad* activeTriad : allTriads)
    if (activeTriad->hasAddMass())
      err += activeTriad->printAdditionalMass(myFile);

  return err;
}


int FmSolverParser::writeParts(std::vector<FmPart*>& gageParts)
{
  int err = 0;

  // Global initial velocity that should apply to all triads
  // that don't have their own initial velocity
  FaVec3 globVel = FmDB::getMechanismObject()->initVel.getValue();
  bool hasGlbVel = !globVel.isZero();

  std::vector<FmPart*> allParts;
  FmDB::getAllParts(allParts);

  for (FmPart* activePart : allParts)
  {
    // Do not write anything for suppressed parts
    if (activePart->suppressInSolver.getValue()) continue;

    FFaString lDesc = activePart->getUserDescription();

    fprintf(myFile,"&SUP_EL\n");
    activePart->printID(myFile);

    if (!activePart->useGenericProperties.getValue())
    {
      // This is a FE part with reduced matrix files
      std::string partPath;
      if (activePart->externalSource.getValue())
	// Assume external part file names are relative to model file location
	partPath = myRelPathCorrection;
      else
      {
	// Create relative path to the part file repository
	partPath = activePart->myRSD.getValue().getCurrentTaskDirName();
	FFaFilePath::makeItAbsolute(partPath,activePart->getAbsFilePath());
	partPath = FFaFilePath::getRelativeFilename(myRDBPath,partPath);
      }

      // Stress and/or gage recovery during dynamics simulation
      int recover = activePart->recoveryDuringSolve.getValue();
      if (recover > 1 &&  activePart->hasStrainRosettes())
        gageParts.push_back(activePart);

      // Beta feature: Specify element groups for stress recovery
      std::string elmGroups = lDesc.getTextAfter("#recover-stress","#");

      if (recover%2 < 1 && lDesc.hasSubString("#recover-stress"))
        ListUI <<"\n---> WARNING: Ignoring #recover-stress"
               <<" in the description field for "<< activePart->getIdString()
               <<".\n     Set this in the \"Advanced\" tab"
               <<" of the part property panel instead.\n";
      if (recover < 2 && lDesc.hasSubString("#recover-gage"))
        ListUI <<"\n---> WARNING: Ignoring #recover-gage"
               <<" in the description field for "<< activePart->getIdString()
               <<".\n     Set this in the \"Advanced\" tab"
               <<" of the part property panel instead.\n";

      std::vector<std::string> fmxFiles;
      fmxFiles.reserve(7);

      // Lambda function checking for non-empty fmx-file names
      auto&& addFile = [&fmxFiles,partPath](size_t idx, std::string& file)
      {
        if (file.empty()) return;

        fmxFiles.resize(idx,"");
        fmxFiles.push_back(file);
        FFaFilePath::makeItAbsolute(fmxFiles.back(),partPath);
      };

      addFile(0,activePart->SMatFile.getValue());
      addFile(1,activePart->MMatFile.getValue());
      addFile(2,activePart->GMatFile.getValue());
      if (recover > 0)
      {
        activePart->saveFEData();
        addFile(3,activePart->BMatFile.getValue());
        addFile(4,activePart->EMatFile.getValue());
        addFile(5,activePart->SAMdataFile.getValue());
        fmxFiles.resize(6,"");
        fmxFiles.push_back(FFaFilePath::getRelativeFilename(myRDBPath,activePart->getBaseFTLFile()));
        fprintf(myFile,"  recoveryFlag = %d\n",recover);
        if (!elmGroups.empty())
          fprintf(myFile,"  elmGroups  = '%s'\n",elmGroups.c_str());
      }
      else if (activePart->hasLoads())
        addFile(3,activePart->LMatFile.getValue());
      else if (activePart->useNonlinearReduction.getValue())
      {
        addFile(3,activePart->DMatFile.getValue());
        addFile(4,activePart->FMatFile.getValue());
        fprintf(myFile,"  numStates  = %d\n",activePart->nonLinStates.getValue());
      }

      if (fmxFiles.empty())
      {
        ListUI <<"\n---> ERROR: "<< activePart->getIdString(true)
               <<" has no reduced matrix files.\n";
        err++;
      }
      else
      {
        fprintf(myFile,"  inputFiles = '%s'",fmxFiles.front().c_str());
        for (size_t i = 1; i < fmxFiles.size(); i++)
          if (fmxFiles[i].empty())
            fprintf(myFile,", ''");
          else
            fprintf(myFile,",\n               '%s'",fmxFiles[i].c_str());
        fprintf(myFile,"\n");
      }

      int ngen = activePart->nGenModes.getValue();
      if (ngen > 0)
        fprintf(myFile,"  numGenDOFs = %d\n",ngen);

      if (activePart->hasLoads())
        fprintf(myFile,"  numLoadCase = %u\n",(unsigned int)activePart->myLoadCases.getValue().size());

      IntVec genDOFsBC;
      if (activePart->getCompModesFlags(genDOFsBC))
      {
	// Suppression of specified component modes
	fprintf(myFile,"  BC = %d",genDOFsBC.front());
	for (size_t j = 1; j < genDOFsBC.size(); j++) {
	  if (j%10 == 0) fprintf(myFile,"\n      ");
	  fprintf(myFile," %d",genDOFsBC[j]);
	}
	fprintf(myFile,"\n");
      }
    }

    // Beta feature: Drag and slam calculation
    bool drag = lDesc.hasSubString("#Drag");
    bool slam = lDesc.hasSubString("#Slam");
    if (activePart->hasBuoyancy.getValue() || drag || slam)
    {
      // Geometry file for buoyancy calculation
      std::string bodyFile(activePart->getGeometryFile());
      if (bodyFile.empty() && activePart->getLinkHandler()) {
	// Convert the FE data if a CAD/visualization file is not provided
	// Beta feature: Specify a subset of elements for body-file extraction
	std::string elmGroups = lDesc.getTextAfter("#Bodygroup","#");
	if (!elmGroups.empty()) // Export only a subset of the elements
	  FFl::activateElmGroups(activePart->getLinkHandler(),elmGroups);
	if (activePart->baseFTLFile.getValue().empty())
	  activePart->setValidBaseFTLFile();
	bodyFile = FFaFilePath::getBaseName(activePart->getBaseFTLFile(true)) + ".ftc";
	FFl::extractBodyFromShell(activePart->getLinkHandler(),
				  activePart->getGlobalCS(),bodyFile);
	if (!elmGroups.empty()) // Reactivate all elements in the link
	  FFl::activateElmGroups(activePart->getLinkHandler());
      }
      if (bodyFile.empty())
	fprintf(myFile,"  bodyFile = 'NONE'\n");
      else
	fprintf(myFile,"  bodyFile = '%s'\n",
                FFaFilePath::getRelativeFilename(myRDBPath,bodyFile).c_str());
    }
    if (drag)
    {
      // Beta feature: Parameters for simplified drag calculations
      double dragParams[18];
      lDesc.getDoublesAfter("#DragTX",3,dragParams);
      lDesc.getDoublesAfter("#DragTY",3,dragParams+3);
      lDesc.getDoublesAfter("#DragTZ",3,dragParams+6);
      lDesc.getDoublesAfter("#DragRX",3,dragParams+9);
      lDesc.getDoublesAfter("#DragRY",3,dragParams+12);
      lDesc.getDoublesAfter("#DragRZ",3,dragParams+15);
      fprintf(myFile,"  dragParams =");
      for (int i = 0; i < 18; i++)
        if (i%3 == 0 && i > 0)
          fprintf(myFile,"\n               %17.9e",dragParams[i]);
        else
          fprintf(myFile," %17.9e",dragParams[i]);
      fprintf(myFile,"\n");
    }
    if (slam)
    {
      // Beta feature: Slamming parameters
      double slamPar[3];
      lDesc.getDoublesAfter("#Slam",3,slamPar);
      fprintf(myFile,"  slamParams = %17.9e %17.9e %17.9e\n",
	      slamPar[0], slamPar[1], slamPar[2]);
    }

    // Part triads
    int cgTriadId = 0;
    std::map<unsigned int,int> nodeNos;
    std::vector<FmTriad*> localTriads;

    if (activePart->useGenericProperties.getValue())
    {
      // Sort the triads on user ID
      cgTriadId = -activePart->getBaseID();
      activePart->getTriads(localTriads,true);
      if (localTriads.size() > 1)
      {
        // Check if there already is a triad at the part CoG
	FaVec3 cog = activePart->getPositionCG().translation();
	double tol = FmDB::getPositionTolerance();
        for (FmTriad* triad : localTriads)
          if (triad->getGlobalTranslation().equals(cog,tol))
            if ((cgTriadId = triad->getBaseID()) > 0)
              break; // we found a triad at the CoG
      }

      unsigned int nodeNum = 1; // Setting up CoG first
      nodeNos[nodeNum] = cgTriadId;

      // Use the current ordering of the localTriads array
      for (FmTriad* triad : localTriads)
        if (triad->getBaseID() != cgTriadId)
          nodeNos[++nodeNum] = triad->getBaseID();
    }
    else
    {
      // Use the FE nodal ordering
      activePart->getTriads(localTriads);
      for (FmTriad* triad : localTriads)
        nodeNos[triad->FENodeNo.getValue()] = triad->getBaseID();
    }

    // Write out the triad array
    fprintf(myFile,"  numTriads = %u\n",(unsigned int)nodeNos.size());
    fprintf(myFile,"  triadIds =");
    size_t j = 0;
    for (const std::pair<const unsigned int,int>& node : nodeNos)
      if ((++j)%10 == 1 && j > 1)
	fprintf(myFile,"\n             %d",node.second);
      else
	fprintf(myFile," %d",node.second);
    fprintf(myFile,"\n");

    // Beta feature: Write out the associated FE node numbers
    if (lDesc.hasSubString("#PrintSupelDef"))
    {
      fprintf(myFile,"  nodeIds =");
      size_t j = 0;
      for (const std::pair<const unsigned int,int>& node : nodeNos)
        if ((++j)%10 == 1 && j > 1)
	  fprintf(myFile,"\n            %u",node.first);
	else
	  fprintf(myFile," %u",node.first);
      fprintf(myFile,"\n");
    }

    // Corotated reference coordinate system positioning
    int shadowPosAlg = 0;
    // Beta feature: Fixed part (use only when fixed internal nodes exist)
    if (lDesc.hasSubString("#Fixed"))
      shadowPosAlg = -1;
    else switch (activePart->myCSOption.getValue())
      {
      case FmPart::MAX_TRI_UNIT_OFFSET:
      case FmPart::MAX_TRI_LINK_SCALE_OFFSET:
	shadowPosAlg = 1; // Triangle fit based on selected triads (and offsets)
	break;
      case FmPart::MASS_BASED_AVERAGE:
	shadowPosAlg = 2; // Mass based average
	break;
      default:
	shadowPosAlg = FmDB::getActiveAnalysis()->defaultShadowPosAlg.getValue();
	if (shadowPosAlg == 0 || shadowPosAlg == 3)
	  shadowPosAlg = 1;
	else if (shadowPosAlg == 4)
	  shadowPosAlg = 2;
	break;
      }
    fprintf(myFile,"  shadowPosAlg = %d\n", shadowPosAlg);

    if (lDesc.hasSubString("#ShadowPosAlg"))
      ListUI <<"\n---> WARNING: Ignoring #ShadowPosAlg <num>"
	     <<" in the description field for "<< activePart->getIdString()
	     <<".\n     Set this in the \"Advanced\" tab"
	     <<" of the part property window instead.\n";

    if (shadowPosAlg == 1)
    {
      // Corotated coordinate system reference triads
      // Get triads and offsets based on chosen offset algorithm
      FmTriad* ref[3];
      FaVec3   off[3];
      if (!activePart->getRefPoints(ref[0],ref[1],ref[2],off[0],off[1],off[2]))
      {
	ListUI <<"\n---> ERROR: "<< activePart->getIdString(true)
	       <<" has no triads.\n";
	err++;
      }
      else for (int i = 0; i < 3; i++)
	fprintf(myFile,"  refTriad%dId = %d, offset%d =%17.9e %17.9e %17.9e\n",
		i+1, ref[i]->getBaseID(), i+1, off[i][0], off[i][1], off[i][2]);
    }

    // Beta feature: Part-level stress stiffening flag
    if (lDesc.hasSubString("#DynStressStiffening"))
      fprintf(myFile,"  stressStiffFlag = 1\n");
    else if (lDesc.hasSubString("#NoDynStressStiffening"))
      fprintf(myFile,"  stressStiffFlag = 0\n");

    // Beta feature: Projection of internal forces
    int projFlag = lDesc.getIntAfter("#Projection");
    if (projFlag > 0)
      fprintf(myFile,"  projDefFlag = %d\n",projFlag);

    // Centripetal force correction
    switch (activePart->myCentripOption.getValue())
      {
      case FmPart::NO_CENTRIP_CORRECTION:
	fprintf(myFile,"  massCorrFlag = 0\n");
	break;
      case FmPart::USE_CENTRIP_CORRECTION:
	fprintf(myFile,"  massCorrFlag = 1\n");
	break;
      default:
	break;
      }

    if (lDesc.hasSubString("#MassCorrection"))
      ListUI <<"\n---> WARNING: Ignoring #MassCorrection"
	     <<" in the description field for "<< activePart->getIdString()
	     <<".\n     Set this in the \"Advanced\" tab"
	     <<" of the part property window instead.\n";
    if (lDesc.hasSubString("#NoMassCorrection"))
      ListUI <<"\n---> WARNING: Ignoring #NoMassCorrection"
	     <<" in the description field for "<< activePart->getIdString()
	     <<".\n     Set this in the \"Advanced\" tab"
	     <<" of the part property window instead.\n";
    if (lDesc.hasSubString("#MassCorrFlag"))
      ListUI <<"\n---> WARNING: Ignoring #MassCorrFlag <num>"
	     <<" in the description field for "<< activePart->getIdString()
	     <<".\n     Set this in the \"Advanced\" tab"
	     <<" of the part property window instead.\n";

    // Scaling of dynamic properties
    double stiffScale = activePart->stiffnessScale.getValue();
    fprintf(myFile,"  stiffScale =%17.9e\n",stiffScale);
    double massScale = activePart->massScale.getValue();
    fprintf(myFile,"  massScale  =%17.9e\n",massScale);

    // Beta feature: Time-dependent stiffness scaling
    int stifSclEngine = lDesc.getIntAfter("#StiffScaleEngine");
    if (stifSclEngine > 0) {
      fprintf(myFile,"  stiffEngineId = %d\n", stifSclEngine);
      FmEngine::betaFeatureEngines.insert(stifSclEngine);
    }

    // Structural damping coefficients
    fprintf(myFile,"  alpha1 =%17.9e," ,activePart->alpha1.getValue());
    fprintf(myFile,"  alpha2 =%17.9e\n",activePart->alpha2.getValue());

    DoubleVec alpha3, alpha4;
    if (activePart->getCompModesAlpha(alpha3,1))
    {
      fprintf(myFile,"  alpha3 =%17.9e",alpha3.front());
      for (j = 1; j < alpha3.size(); j++) {
	if (j%6 == 0) fprintf(myFile,"\n         ");
	fprintf(myFile," %17.9e",alpha3[j]);
      }
      fprintf(myFile,"\n");
    }
    if (activePart->getCompModesAlpha(alpha4,2))
    {
      fprintf(myFile,"  alpha4 =%17.9e",alpha4.front());
      for (j = 1; j < alpha4.size(); j++) {
	if (j%6 == 0) fprintf(myFile,"\n         ");
	fprintf(myFile," %17.9e",alpha4[j]);
      }
      fprintf(myFile,"\n");
    }

    // Possibly time-dependent structural damping
    int structDmpEngine = activePart->getStructDmpEngineId();
    if (structDmpEngine > 0)
      fprintf(myFile,"  strDmpEngineId = %d\n", structDmpEngine);

    // Part position
    FaMat34 lCS = activePart->getGlobalCS();
    fprintf(myFile,"  supPos =%17.9e %17.9e %17.9e %17.9e\n",
	    lCS[0][0],lCS[1][0],lCS[2][0],lCS[3][0]);
    fprintf(myFile,"          %17.9e %17.9e %17.9e %17.9e\n",
	    lCS[0][1],lCS[1][1],lCS[2][1],lCS[3][1]);
    fprintf(myFile,"          %17.9e %17.9e %17.9e %17.9e\n",
	    lCS[0][2],lCS[1][2],lCS[2][2],lCS[3][2]);

    // Beta feature: Output of position matrices for specified parts
    if (lDesc.hasSubString("#savePos"))
      fprintf(myFile,"  savePos = 1\n");

    // Variables to be saved
    // 1 - Center of gravity
    // 2 - Generalized DOF components (dis,vel,acc)
    // 3 - Energies
    activePart->writeSaveVar(myFile,3);

    fprintf(myFile,"/\n");

    if (cgTriadId > 0)
    {
      FmModelMemberBase* cgTriad = FmDB::findObject(cgTriadId);
      if (cgTriad)
        ListUI <<"  -> Detected center of gravity at "<< cgTriad->getIdString()
	       <<" for "<< activePart->getIdString() <<"\n";
    }
    else if (cgTriadId < 0)
    {
      // Create a dummy triad at the CoG of the generic part
      fprintf(myFile,"\n! Center of Gravity solver triad\n");
      fprintf(myFile,"&TRIAD\n");
      fprintf(myFile,"  id = %d\n", cgTriadId);
      int nDOFs = activePart->condenseOutCoG.getValue() ? 0 : 6;
      fprintf(myFile,"  nDOFs = %d\n",nDOFs);

      ListUI <<"  -> Using dummy center of gravity triad for "
	     << activePart->getIdString()
	     << (nDOFs == 0 ? " (condensed out)\n" : "]\n");

      // Global position matrix
      FaMat34 ur = activePart->getPositionCG();
      fprintf(myFile,"  ur  =%17.9e %17.9e %17.9e %17.9e\n",
	      ur[0][0],ur[1][0],ur[2][0],ur[3][0]);
      fprintf(myFile,"       %17.9e %17.9e %17.9e %17.9e\n",
 	      ur[0][1],ur[1][1],ur[2][1],ur[3][1]);
      fprintf(myFile,"       %17.9e %17.9e %17.9e %17.9e\n",
 	      ur[0][2],ur[1][2],ur[2][2],ur[3][2]);

      if (nDOFs > 0)
      {
	// Beta feature: Initial velocity in description field
	double triadVel[6];
	bool hasTraVel = lDesc.hasSubString("#InitTransVel");
	bool hasRotVel = lDesc.hasSubString("#InitRotVel");
	if (hasTraVel)
	  lDesc.getDoublesAfter("#InitTransVel",3,triadVel);
	else if (hasGlbVel)
	  memcpy(triadVel,globVel.getPt(),3*sizeof(double));
	else
	  triadVel[0] = triadVel[1] = triadVel[2] = 0.0;
	if (hasRotVel)
	  lDesc.getDoublesAfter("#InitRotVel",3,triadVel+3);
	else
	  triadVel[3] = triadVel[4] = triadVel[5] = 0.0;

	if (hasTraVel || hasRotVel || hasGlbVel)
	{
	  fprintf(myFile,"  urd =%17.9e",triadVel[0]);
	  for (int k = 1; k < nDOFs; k++)
	    fprintf(myFile," %17.9e",triadVel[k]);
	  fprintf(myFile,"\n");
	}
      }
      fprintf(myFile,"/\n\n");

      // Initial position of CoG triad in part coordinate system
      localTriads.front()->printLocalPos(myFile,activePart,cgTriadId);
    }

    // Initial triad positions in part coordinate system
    for (FmTriad* triad : localTriads)
      triad->printLocalPos(myFile,activePart);
    fprintf(myFile,"\n");

    if (activePart->useGenericProperties.getValue())
    {
      // Use generic mass- and stiffness properties for this part
      fprintf(myFile,"&GENERIC_PART\n");
      fprintf(myFile,"  supElId =  %d\n", activePart->getBaseID());
      fprintf(myFile,"  mass    = %17.9e\n", activePart->mass.getValue());

      // Transform inertia to part orientation if given in CoG orientation
      FFaTensor3 inertia = activePart->inertia.getValue();
      if (activePart->myInertiaRef.getValue() == FmPart::POS_CG_ROT_CG)
	inertia.rotate(activePart->getPositionCG(false).direction().transpose());

      fprintf(myFile,"  inertia =");
      for (int k = 0; k < 6; k++) fprintf(myFile," %17.9e", inertia[k]);

      if (activePart->myGenericPartStiffType.getValue() == FmPart::NODE_STIFFNESS)
      {
	fprintf(myFile,"\n  kt      = %17.9e", activePart->kt.getValue());
	fprintf(myFile,"\n  kr      = %17.9e", activePart->kr.getValue());
      }
      else
	fprintf(myFile,"\n  isRigid =  1");

      fprintf(myFile,"\n/\n\n");
    }
    else if (activePart->hasLoads())
      activePart->printSolverLoads(myFile);
  }

  return err;
}


int FmSolverParser::writeBeams(int& nextBaseId)
{
  int err = 0;

  std::vector<FmBeam*> allBeams;
  FmDB::getAllBeams(allBeams);

  for (FmBeam* beam : allBeams)
  {
    // Check if this is a wind turbine blade element.
    // The solver input is then handled by a specialized function.
    if (FmTurbine::writeBladeElement(myFile,beam,nextBaseId)) continue;

    err += beam->printSolverEntry(myFile,0,beam->getBeamProperty(),&myRDBPath);

    // Initial triad positions in beam coordinate system
    std::vector<FmTriad*> triads;
    beam->getTriads(triads);
    for (FmTriad* triad : triads)
      triad->printLocalPos(myFile,beam);
    fprintf(myFile,"\n");
  }

  return err;
}


int FmSolverParser::writeJoints()
{
  int err = 0;

  std::vector<FmJointBase*> allJoints;
  std::vector<FmJointBase*> chainJoints;
  std::vector<FmJointBase*> otherJoints;
  FmDB::getAllJoints(allJoints);

  for (FmJointBase* joint : allJoints)
    if (joint->isMasterSlaveInOtherJoint())
      chainJoints.push_back(joint);
    else
      otherJoints.push_back(joint);

  allJoints = otherJoints;
  allJoints.insert(allJoints.end(), chainJoints.begin(), chainJoints.end());

  for (FmJointBase* activeJoint : allJoints)
  {
    // Do not write anything for suppressed joints
    if (activeJoint->isSuppressed()) continue;

    if (activeJoint->isContactElement())
      err += writeContactElement((FmCamJoint*)activeJoint);

    else if (activeJoint->isGlobalSpringElement())
    {
      // Beta feature: Global springs
      fprintf(myFile,"! Global spring\n");
      fprintf(myFile,"&SPRING_ELEMENT\n");
      activeJoint->printID(myFile);
      fprintf(myFile,"  springBaseId =");
      for (int d = 0; d < 6; d++)
	fprintf(myFile," %d",activeJoint->getSpringBaseID(d));
      fprintf(myFile," 1"); // Flagging this is a global spring (instead of axial)

      FFaString jDesc = activeJoint->getUserDescription();
      if (jDesc.hasSubString("#K"))
      {
	// Beta feature: Explicit coupling stiffness
	fprintf(myFile,"\n  couplStiff =");
	std::string keyWord("#K00");
	for (char i = '1'; i <= '6'; i++)
	  for (char j = i+1; j <= '6'; j++)
	  {
	    keyWord[2] = i;
	    keyWord[3] = j;
	    fprintf(myFile," %g",jDesc.getDoubleAfter(keyWord.c_str()));
	  }
      }

      std::vector<FmTriad*> triads;
      activeJoint->getMasterTriads(triads);
      triads.push_back(activeJoint->getSlaveTriad());
      fprintf(myFile,"\n  triadIDs =");
      for (FmTriad* triad : triads)
	if (triad->getNDOFs() > 0)
	  fprintf(myFile," %d",triad->getBaseID());
      fprintf(myFile,"\n/\n\n");
    }
    else
    {
      FFaString jDesc = activeJoint->getUserDescription();

      fprintf(myFile,"&MASTERSLAVEJOINT\n");
      activeJoint->printID(myFile);

      // Write initial joint position
      double slideValue = 0.0;
      FaMat34 ur, urSlave, urSlider;
      if (activeJoint->isOfType(FmCamJoint::getClassTypeID()))
      {
	urSlave = activeJoint->getSlaveTriad()->getGlobalCS();
	Fm1DMaster* master = ((FmMMJointBase*)activeJoint)->getMaster();
	if (master)
	{
	  slideValue = master->getSliderPosition(urSlider,urSlave.translation());
	  ur = urSlider;
	}
	else
	  ur = urSlave;
      }
      else if (activeJoint->isAxialJoint(true)) // Beta feature: Axial joint
      {
	urSlider = ((FmSMJointBase*)activeJoint)->getItsMasterTriad()->getGlobalCS();
	urSlave = activeJoint->getSlaveTriad()->getGlobalCS();
	ur.makeGlobalizedCS(urSlider.translation(),urSlave.translation());
      }
      else if (activeJoint->isOfType(FmSMJointBase::getClassTypeID()))
	ur = activeJoint->getGlobalCS();
      else
	ur = urSlave = activeJoint->getSlaveTriad()->getGlobalCS();

      fprintf(myFile,"  InitPosInGlobal =%17.9e %17.9e %17.9e %17.9e\n",
	      ur[0][0],ur[1][0],ur[2][0],ur[3][0]);
      fprintf(myFile,"                   %17.9e %17.9e %17.9e %17.9e\n",
	      ur[0][1],ur[1][1],ur[2][1],ur[3][1]);
      fprintf(myFile,"                   %17.9e %17.9e %17.9e %17.9e\n",
	      ur[0][2],ur[1][2],ur[2][2],ur[3][2]);

      std::string ignored;
      if (jDesc.hasSubString("#InitTXvel")) ignored.append(" #InitTXvel");
      if (jDesc.hasSubString("#InitTYvel")) ignored.append(" #InitTYvel");
      if (jDesc.hasSubString("#InitTZvel")) ignored.append(" #InitTZvel");
      if (jDesc.hasSubString("#InitRXvel")) ignored.append(" #InitRXvel");
      if (jDesc.hasSubString("#InitRYvel")) ignored.append(" #InitRYvel");
      if (jDesc.hasSubString("#InitRZvel")) ignored.append(" #InitRZvel");
      if (!ignored.empty())
	ListUI <<"\n---> WARNING: Ignoring"<< ignored
	       <<" in the description field for "<< activeJoint->getIdString()
	       <<".\n     Use the \"Initial velocity\" field in the"
	       <<" joint property window instead.\n";

      FmCylJoint* screwJoint = NULL;
      int CVJointID = 0;
      int nJVar = 0;
      IntVec iDof;
      iDof.reserve(6);

      // Beta feature: Separate version type flag
      int version = jDesc.getIntAfter("#Version");
      if (version != 0)
        fprintf(myFile,"  version      = %d\n", version);

      if (activeJoint->isOfType(FmRevJoint::getClassTypeID()))
      {
	// *** REVOLUTE JOINT ***

	fprintf(myFile,"  type         = 1\n");
	if (jDesc.hasSubString("#FreeZ"))
	  ListUI <<"\n---> WARNING: Ignoring #FreeZ"
		 <<" in the description field for "<< activeJoint->getIdString()
		 <<".\n     Use the \"Z translation DOF\" toggle"
		 <<" in the joint property window instead.\n";

	iDof.push_back(5);
	if (activeJoint->isLegalDOF(2))
	{
	  nJVar = 2;
	  iDof.push_back(2);
	  fprintf(myFile,"  nJointVars   = 2\n");
	  fprintf(myFile,"  JointVarDefs = 6 1   3 1\n");
	  fprintf(myFile,"  JVarInitVal  = %17.9e %17.9e\n",
		  activeJoint->getJointVariable(5),
		  activeJoint->getJointVariable(2));
	  fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e\n",
		  activeJoint->getInitVel(5),
		  activeJoint->getInitVel(2));
	  fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e\n",
		  activeJoint->getInitAcc(5),
		  activeJoint->getInitAcc(2));
	  fprintf(myFile,"  springId     = %d %d\n",
		  activeJoint->getSpringBaseID(5),
		  activeJoint->getSpringBaseID(2));
	  fprintf(myFile,"  damperId     = %d %d\n",
		  activeJoint->getDamperBaseID(5),
		  activeJoint->getDamperBaseID(2));
	}
	else
	{
	  nJVar = 1;
	  fprintf(myFile,"  nJointVars   = 1\n");
	  fprintf(myFile,"  JointVarDefs = 6 1\n");

	  fprintf(myFile,"  JVarInitVal  = %17.9e\n",
		  activeJoint->getJointVariable(5));
	  fprintf(myFile,"  JVarInitVel  = %17.9e\n",
		  activeJoint->getInitVel(5));
	  fprintf(myFile,"  JVarInitAcc  = %17.9e\n",
		  activeJoint->getInitAcc(5));

	  fprintf(myFile,"  springId     = %d\n",
		  activeJoint->getSpringBaseID(5));
	  fprintf(myFile,"  damperId     = %d\n",
		  activeJoint->getDamperBaseID(5));
	}

	writeFriction(activeJoint,iDof);
      }

      else if (activeJoint->isOfType(FmBallJoint::getClassTypeID()))
      {
	fprintf(myFile,"  type         = 2\n");
	if (jDesc.hasSubString("#UniversalJoint"))
	{
	  // Beta feature:
	  // *** UNIVERSAL JOINT ***

	  nJVar = 2;
	  fprintf(myFile,"  nJointVars   = 2\n");
	  fprintf(myFile,"  JointVarDefs = 6 2   5 1\n"); // Z-Y follower

	  fprintf(myFile,"  JVarInitVal  = %17.9e %17.9e\n",
		  activeJoint->getJointVariable(5),
		  activeJoint->getJointVariable(4));
	  fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e\n",
		  activeJoint->getInitVel(5),
		  activeJoint->getInitVel(4));
	  fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e\n",
		  activeJoint->getInitAcc(5),
		  activeJoint->getInitAcc(4));
	  fprintf(myFile,"  springId     = %d %d\n",
		  activeJoint->getSpringBaseID(5),
		  activeJoint->getSpringBaseID(4));
	  fprintf(myFile,"  damperId     = %d %d\n",
		  activeJoint->getDamperBaseID(5),
		  activeJoint->getDamperBaseID(4));
	}

	else if (jDesc.hasSubString("#CVJoint"))
	{
	  // Beta feature:
	  // *** CONSTANT VELOCITY JOINT ***

	  CVJointID = activeJoint->getBaseID();
	  nJVar = 4;
	  fprintf(myFile,"  nJointVars   = 4\n");
	  fprintf(myFile,"  JointVarDefs = 6 4   5 3   5 2   6 1\n");

	  double RZ = jDesc.getDoubleAfter("#RZ")*0.5;
	  double RY = jDesc.getDoubleAfter("#RY")*0.5;
	  fprintf(myFile,"  JVarInitVal  = %17.9e %17.9e %17.9e %17.9e\n",
		  RZ, RY, RY, RZ);
	  //TODO,kmo: Check if this is correct
	  fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e 0.0 0.0\n",
		  activeJoint->getInitVel(5),
		  activeJoint->getInitVel(4));
	  fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e 0.0 0.0\n",
		  activeJoint->getInitAcc(5),
		  activeJoint->getInitAcc(4));
	  //TODO,kmo: Check if this is correct
	  fprintf(myFile,"  springId     = %d %d 0 0\n",
		  activeJoint->getSpringBaseID(5),
		  activeJoint->getSpringBaseID(4));
	  fprintf(myFile,"  damperId     = %d %d 0 0\n",
		  activeJoint->getDamperBaseID(5),
		  activeJoint->getDamperBaseID(4));
	}

	else
	{
	  // *** BALL JOINT ***

	  nJVar = 3;
	  fprintf(myFile,"  nJointVars   = 3\n");
	  err += writeRotationJointVars("JointVarDefs =",activeJoint,iDof);

	  fprintf(myFile,"  JVarInitVal  = %17.9e %17.9e %17.9e\n",
		  activeJoint->getJointVariable(iDof[0]),
		  activeJoint->getJointVariable(iDof[1]),
		  activeJoint->getJointVariable(iDof[2]));
	  fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e %17.9e\n",
		  activeJoint->getInitVel(iDof[0]),
		  activeJoint->getInitVel(iDof[1]),
		  activeJoint->getInitVel(iDof[2]));
	  fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e %17.9e\n",
		  activeJoint->getInitAcc(iDof[0]),
		  activeJoint->getInitAcc(iDof[1]),
		  activeJoint->getInitAcc(iDof[2]));
	  fprintf(myFile,"  rotSpringCpl = '%s'\n",
		  activeJoint->rotSpringCpl.getValue().getText());

	  fprintf(myFile,"  springId     = %d %d %d\n",
		  activeJoint->getSpringBaseID(iDof[0]),
		  activeJoint->getSpringBaseID(iDof[1]),
		  activeJoint->getSpringBaseID(iDof[2]));
	  fprintf(myFile,"  damperId     = %d %d %d\n",
		  activeJoint->getDamperBaseID(iDof[0]),
		  activeJoint->getDamperBaseID(iDof[1]),
		  activeJoint->getDamperBaseID(iDof[2]));

	  if (!writeFriction(activeJoint,iDof))
	    if (jDesc.hasSubString("#BallFriction")) {
	      // Beta feature: Multi-dof ball joint friction
	      int fId = jDesc.getIntAfter("#BallFriction");
	      fprintf(myFile,"  frictionSetId = %d %d %d\n", fId, fId, fId);
	    }
	}
      }

      else if (activeJoint->isOfType(FmRigidJoint::getClassTypeID()))
      {
	// *** RIGID JOINT ***

	// Beta feature: Release specified joint DOFs
	fprintf(myFile,"  type         = 3\n");
	fprintf(myFile,"  JointVarDefs =");
	if (jDesc.hasSubString("#FreeX"))
        {
	  nJVar++;
	  fprintf(myFile,"   1 1");
	}
	if (jDesc.hasSubString("#FreeY"))
        {
	  nJVar++;
	  fprintf(myFile,"   2 1");
	}
	if (jDesc.hasSubString("#FreeZ"))
        {
	  nJVar++;
	  fprintf(myFile,"   3 1");
	}
	if (jDesc.hasSubString("#FreeRX"))
        {
	  nJVar++;
	  fprintf(myFile,"   4 1");
	}
	if (jDesc.hasSubString("#FreeRY"))
        {
	  nJVar++;
	  fprintf(myFile,"   5 1");
	}
	if (jDesc.hasSubString("#FreeRZ"))
        {
	  nJVar++;
	  fprintf(myFile,"   6 1");
	}

	fprintf(myFile,"   0 0\n");
	fprintf(myFile,"  nJointVars   = %d\n",nJVar);

	if (nJVar > 0)
	{
	  fprintf(myFile,"  JVarInitVel  =");
	  if (jDesc.hasSubString("#FreeX"))  fprintf(myFile," %17.9e",activeJoint->getInitVel(0));
	  if (jDesc.hasSubString("#FreeY"))  fprintf(myFile," %17.9e",activeJoint->getInitVel(1));
	  if (jDesc.hasSubString("#FreeZ"))  fprintf(myFile," %17.9e",activeJoint->getInitVel(2));
	  if (jDesc.hasSubString("#FreeRX")) fprintf(myFile," %17.9e",activeJoint->getInitVel(3));
	  if (jDesc.hasSubString("#FreeRY")) fprintf(myFile," %17.9e",activeJoint->getInitVel(4));
	  if (jDesc.hasSubString("#FreeRZ")) fprintf(myFile," %17.9e",activeJoint->getInitVel(5));
	  fprintf(myFile,"\n  JVarInitAcc  =");
	  if (jDesc.hasSubString("#FreeX"))  fprintf(myFile," %17.9e",activeJoint->getInitAcc(0));
	  if (jDesc.hasSubString("#FreeY"))  fprintf(myFile," %17.9e",activeJoint->getInitAcc(1));
	  if (jDesc.hasSubString("#FreeZ"))  fprintf(myFile," %17.9e",activeJoint->getInitAcc(2));
	  if (jDesc.hasSubString("#FreeRX")) fprintf(myFile," %17.9e",activeJoint->getInitAcc(3));
	  if (jDesc.hasSubString("#FreeRY")) fprintf(myFile," %17.9e",activeJoint->getInitAcc(4));
	  if (jDesc.hasSubString("#FreeRZ")) fprintf(myFile," %17.9e",activeJoint->getInitAcc(5));
	  fprintf(myFile,"\n");
	}
      }

      else if (activeJoint->isAxialJoint()) // Beta feature: 1-DOF Axial joint
      {
	// *** AXIAL JOINT ***

	nJVar = 1;
	iDof.push_back(0);
	double jLength = (urSlave.translation()-ur.translation()).length();
	fprintf(myFile,"  type         = 8\n");
	fprintf(myFile,"  nJointVars   = 1\n");
	fprintf(myFile,"  JointVarDefs = 1 1\n");
	fprintf(myFile,"  JVarInitVal  = %17.9e\n", jLength);
	fprintf(myFile,"  JVarInitVel  = %17.9e\n", activeJoint->getInitVel(0));
	fprintf(myFile,"  JVarInitAcc  = %17.9e\n", activeJoint->getInitAcc(0));
	fprintf(myFile,"  springId     = %d\n", activeJoint->getSpringBaseID(0));
	fprintf(myFile,"  damperId     = %d\n", activeJoint->getDamperBaseID(0));
      }

      else if (activeJoint->isOfType(FmFreeJoint::getClassTypeID()))
      {
	// *** FREE JOINT ***

	nJVar = 6;
	fprintf(myFile,"  type         = 4\n");
	// Beta feature: Deactivation of rotation-translation coupling
	if (version == 0 && jDesc.hasSubString("#noRotTransCoupling"))
	  fprintf(myFile,"  version      = -1\n");
	fprintf(myFile,"  nJointVars   = 6\n");
	err += writeRotationJointVars("JointVarDefs = 1 3   2 3   3 3  ",activeJoint,iDof);

	fprintf(myFile,"  JVarInitVal  = %17.9e %17.9e %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getJointVariable(0),
		activeJoint->getJointVariable(1),
		activeJoint->getJointVariable(2),
		activeJoint->getJointVariable(iDof[0]),
		activeJoint->getJointVariable(iDof[1]),
		activeJoint->getJointVariable(iDof[2]));
	fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitVel(0),
		activeJoint->getInitVel(1),
		activeJoint->getInitVel(2),
		activeJoint->getInitVel(iDof[0]),
		activeJoint->getInitVel(iDof[1]),
		activeJoint->getInitVel(iDof[2]));
	fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitAcc(0),
		activeJoint->getInitAcc(1),
		activeJoint->getInitAcc(2),
		activeJoint->getInitAcc(iDof[0]),
		activeJoint->getInitAcc(iDof[1]),
		activeJoint->getInitAcc(iDof[2]));

	fprintf(myFile,"  rotSpringCpl = '%s'\n",
		activeJoint->rotSpringCpl.getValue().getText());
	fprintf(myFile,"  tranSpringCpl = '%s'\n",
		activeJoint->tranSpringCpl.getValue().getText());

	fprintf(myFile,"  springId     = %d %d %d %d %d %d\n",
		activeJoint->getSpringBaseID(0),
		activeJoint->getSpringBaseID(1),
		activeJoint->getSpringBaseID(2),
		activeJoint->getSpringBaseID(iDof[0]),
		activeJoint->getSpringBaseID(iDof[1]),
		activeJoint->getSpringBaseID(iDof[2]));
	fprintf(myFile,"  damperId     = %d %d %d %d %d %d\n",
		activeJoint->getDamperBaseID(0),
		activeJoint->getDamperBaseID(1),
		activeJoint->getDamperBaseID(2),
		activeJoint->getDamperBaseID(iDof[0]),
		activeJoint->getDamperBaseID(iDof[1]),
		activeJoint->getDamperBaseID(iDof[2]));

	iDof.insert(iDof.begin(),3,0);
	iDof[1] = 1; iDof[2] = 2;
	writeFriction(activeJoint,iDof);
      }

      else if (activeJoint->isOfType(FmPrismJoint::getClassTypeID()))
      {
	// *** PRISMATIC JOINT ***

	fprintf(myFile,"  type         = 5\n");
	// Beta feature: Cubic interpolation over the independent triads
	if (version == 0 && jDesc.hasSubString("#Cubic"))
	  fprintf(myFile,"  version      = 3\n");

	nJVar = 3; // Account for the slider DOF
	fprintf(myFile,"  nJointVars   = 2\n");
	fprintf(myFile,"  JointVarDefs = 5 2   4 1\n"); // Y-X follower axis

	fprintf(myFile,"  JVarInitVal  = 0.0 0.0 %17.9e\n",
		activeJoint->getJointVariable(2));
	fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e %17.9e\n",
		activeJoint->getInitVel(4),
		activeJoint->getInitVel(3),
		activeJoint->getInitVel(2));
	fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e %17.9e\n",
		activeJoint->getInitAcc(4),
		activeJoint->getInitAcc(3),
		activeJoint->getInitAcc(2));
	fprintf(myFile,"  springId     = 0 0 %d\n",
		activeJoint->getSpringBaseID(2));
	fprintf(myFile,"  damperId     = 0 0 %d\n",
		activeJoint->getDamperBaseID(2));

	iDof.resize(3,0); iDof[2] = 6;
	writeFriction(activeJoint,iDof);
	iDof[0] = 4; iDof[1] = 3; iDof[2] = 2;
      }

      else if (activeJoint->isOfType(FmCylJoint::getClassTypeID()))
      {
	// *** CYLINDRIC JOINT ***

	screwJoint = static_cast<FmCylJoint*>(activeJoint);
	if (!screwJoint->isScrewTransmission()) screwJoint = 0;

	fprintf(myFile,"  type         = 6\n");
	// Beta feature: Cubic interpolation over the independent triads
	if (jDesc.hasSubString("#Cubic"))
	  fprintf(myFile,"  version      = 3\n");

	nJVar = 4; // Account for the slider DOF
	fprintf(myFile,"  nJointVars   = 3\n");
	// Beta feature: Rotation axis parameterization
	if (jDesc.hasSubString("#RotAxisParam"))
	  fprintf(myFile,"  JointVarDefs = 6 1   5 1   4 1\n");
	else // Z-Y-X follower axis
	  fprintf(myFile,"  JointVarDefs = 6 3   5 2   4 1\n");

	fprintf(myFile,"  JVarInitVal  = %17.9e 0.0 0.0 %17.9e\n",
		activeJoint->getJointVariable(5),
		activeJoint->getJointVariable(2));
	fprintf(myFile,"  JVarInitVel  = %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitVel(5),
		activeJoint->getInitVel(4),
		activeJoint->getInitVel(3),
		activeJoint->getInitVel(2));
	fprintf(myFile,"  JVarInitAcc  = %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitAcc(5),
		activeJoint->getInitAcc(4),
		activeJoint->getInitAcc(3),
		activeJoint->getInitAcc(2));
	fprintf(myFile,"  springId     = %d 0 0 %d\n",
		activeJoint->getSpringBaseID(5),
		activeJoint->getSpringBaseID(2));
	fprintf(myFile,"  damperId     = %d 0 0 %d\n",
		activeJoint->getDamperBaseID(5),
		activeJoint->getDamperBaseID(2));
	iDof.push_back(5);
	iDof.push_back(4);
	iDof.push_back(3);
	iDof.push_back(2);
      }

      else if (activeJoint->isOfType(FmCamJoint::getClassTypeID()))
      {
	// *** CAM JOINT ***

	fprintf(myFile,"  type         = 7\n");
	if (jDesc.hasSubString("#SpringActiveRadius"))
	  ListUI <<"\n---> WARNING: Ignoring #SpringActiveRadius"
		 <<" in the description field for "<< activeJoint->getIdString()
		 <<".\n     Use the \"Thickness\" field"
		 <<" in the joint property window instead.\n";
	fprintf(myFile,"  camThickness = %17.9e\n",
		((FmCamJoint*)activeJoint)->getThickness());

	// Beta feature: Fix the lateral translation DOFs
	bool freeX = !jDesc.hasSubString("#FixX");
	bool freeY = !jDesc.hasSubString("#FixY");

	nJVar = 3;
	if (freeX) nJVar++;
	if (freeY) nJVar++;
	fprintf(myFile,"  nJointVars   = %d\n", nJVar);
	nJVar++; // Account for the slider DOF

	fprintf(myFile,"  JointVarDefs = ");
	// Beta feature: Rotation axis parameterization
	if (jDesc.hasSubString("#RotAxisParam"))
	{
	  if (freeX) fprintf(myFile,"1 1  ");
	  if (freeY) fprintf(myFile,"2 1  ");
	  fprintf(myFile,"6 1   5 1   4 1\n");
	}
	else // Z-Y-X follower axis
	{
	  if (freeX) fprintf(myFile,"1 3  ");
	  if (freeY) fprintf(myFile,"2 3  ");
	  fprintf(myFile,"6 3   5 2   4 1\n");
	}

	// TODO: update with correct initial values when initially open cam
	FaVec3 dist = urSlave.translation() - urSlider.translation();
	fprintf(myFile,"  JVarInitVal  =");
	if (freeX) fprintf(myFile," %17.9e", ur[0]*dist);
	if (freeY) fprintf(myFile," %17.9e", ur[1]*dist);
	fprintf(myFile," 0.0 0.0 0.0 %17.9e\n", slideValue);
	fprintf(myFile,"  JVarInitVel  =");
	if (freeX) fprintf(myFile," %17.9e", activeJoint->getInitVel(0));
	if (freeY) fprintf(myFile," %17.9e", activeJoint->getInitVel(1));
	fprintf(myFile," %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitVel(5),
		activeJoint->getInitVel(4),
		activeJoint->getInitVel(3),
		activeJoint->getInitVel(2));
	fprintf(myFile,"  JVarInitAcc  =");
	if (freeX) fprintf(myFile," %17.9e", activeJoint->getInitAcc(0));
	if (freeY) fprintf(myFile," %17.9e", activeJoint->getInitAcc(1));
	fprintf(myFile," %17.9e %17.9e %17.9e %17.9e\n",
		activeJoint->getInitAcc(5),
		activeJoint->getInitAcc(4),
		activeJoint->getInitAcc(3),
		activeJoint->getInitAcc(2));

	// Write springs
	fprintf(myFile,"  springId     =");
	if (freeX) fprintf(myFile," %d", activeJoint->getSpringBaseID(0));
	if (freeY) fprintf(myFile," %d", activeJoint->getSpringBaseID(1));
	fprintf(myFile," 0 0 0 0\n");

	// Write dampers
	fprintf(myFile,"  damperId     =");
	if (freeX) fprintf(myFile," %d", activeJoint->getDamperBaseID(0));
	if (freeY) fprintf(myFile," %d", activeJoint->getDamperBaseID(1));
	fprintf(myFile," 0 0 0 0\n");

	// Write friction
	iDof.resize(nJVar,0); iDof.back() = 6;
	writeFriction(activeJoint,iDof);
	iDof[0] = 5; iDof[1] = 4; iDof[2] = 3;
	iDof.back() = 2;
	if (freeX) iDof[3] = 0;
	if (freeY) iDof[freeX ? 4 : 3] = 1;
      }

      if (!iDof.empty() && activeJoint->hasConstraints(true))
      {
	// Additional BCs for static equilibrium and eigenvalue analysis
	fprintf(myFile,"  BC =");
        for (int d : iDof)
          fprintf(myFile," %d", activeJoint->getStatusCode(d));
	fprintf(myFile,"\n");
      }

      // Joint DOF variables to be saved:
      // 1 - Deflection
      // 2 - Velocity
      // 3 - Acceleration
      // 4 - Friction force
      // 5 - Friction energies
      activeJoint->writeSaveVar(myFile,5*nJVar);

      int slaveId = activeJoint->getSlaveTriad()->getBaseID();
      int masterId = 0;
      if (activeJoint->isOfType(FmSMJointBase::getClassTypeID()))
	masterId = ((FmSMJointBase*)activeJoint)->getItsMasterTriad()->getBaseID();
      else if (activeJoint->isOfType(FmMMJointBase::getClassTypeID()))
        if (((FmMMJointBase*)activeJoint)->getMaster())
          masterId = ((FmMMJointBase*)activeJoint)->getMaster()->getBaseID();

      fprintf(myFile,"  slaveId  = %d\n", slaveId);
      fprintf(myFile,"  masterId = %d\n", masterId);
      fprintf(myFile,"/\n\n");

      if (CVJointID)
      {
	// Write Higher Pair connection for this CVJoint
	fprintf(myFile,"! Constant velocity joint internal connections\n");
	fprintf(myFile,"&HIGHER_PAIR\n");
	activeJoint->printID(myFile);
	fprintf(myFile,"  slaveJoint     = %d\n", CVJointID);
	fprintf(myFile,"  slaveJointDof  = 4\n"); // Z-rotation of ball joint
	fprintf(myFile,"  masterJoint    = %d\n", CVJointID);
	fprintf(myFile,"  masterJointDof = 1\n"); // Z-rotation of ball joint
	fprintf(myFile,"  coeff          = 1.0\n/\n");
	fprintf(myFile,"&HIGHER_PAIR\n");
	activeJoint->printID(myFile);
	fprintf(myFile,"  slaveJoint     = %d\n", CVJointID);
	fprintf(myFile,"  slaveJointDof  = 3\n"); // Y-rotation of ball joint
	fprintf(myFile,"  masterJoint    = %d\n", CVJointID);
	fprintf(myFile,"  masterJointDof = 2\n"); // Y-rotation of ball joint
	fprintf(myFile,"  coeff          = 1.0\n/\n\n");
      }
      else if (screwJoint)
      {
	// Write Higher Pair connection representing the screw transmission
	fprintf(myFile,"! Screw transmission internal connection\n");
	fprintf(myFile,"&HIGHER_PAIR\n");
	screwJoint->printID(myFile);
	fprintf(myFile,"  slaveJoint     = %d\n", screwJoint->getBaseID());
	fprintf(myFile,"  slaveJointDof  = 4\n"); // Slider dof of cylindric joint
	fprintf(myFile,"  masterJoint    = %d\n", screwJoint->getBaseID());
	fprintf(myFile,"  masterJointDof = 1\n"); // Z-rotation of cylindric joint
	fprintf(myFile,"  coeff          = %17.9e\n", screwJoint->getScrewRatio());
	fprintf(myFile,"/\n\n");
      }

      if (jDesc.find("#JointLoadEngine") != std::string::npos)
	ListUI <<"\n---> WARNING: Ignoring #JointLoadEngine"
	       <<" in the description field for "<< activeJoint->getIdString()
	       <<".\n     Use the \"Load magnitude\" field in the"
	       <<" joint property window instead.\n";
    }

    // Write base springs and yield for all frictions that have stiffness
    FmFrictionBase* activeFriction = activeJoint->getFriction();
    if (activeFriction)
      if (activeFriction->getStickStiffness() > 0.0)
      {
	fprintf(myFile,"! Friction limit used by joint friction spring\n");
	fprintf(myFile,"&SPRING_YIELD\n");
	activeJoint->printID(myFile);
	fprintf(myFile,"/\n");

	fprintf(myFile,"! Friction spring for joint or contact element\n");
	fprintf(myFile,"&SPRING_BASE\n");
	activeJoint->printID(myFile);
	fprintf(myFile,"  s0 = %17.9e\n", activeFriction->getStickStiffness());
	fprintf(myFile,"  springYieldId = %d\n", activeJoint->getBaseID());
	fprintf(myFile,"/\n\n");
      }
  }

  return err;
}


int FmSolverParser::writeRotationJointVars(const char* jointVarDefs,
					   FmJointBase* aJoint, IntVec& iDof)
{
  iDof.resize(3,0);
  switch (aJoint->rotSequence.getValue())
    {
    case FmJointBase::rZYX: iDof[0]=5; iDof[1]=4; iDof[2]=3; break;
    case FmJointBase::rYXZ: iDof[0]=4; iDof[1]=3; iDof[2]=5; break;
    case FmJointBase::rXZY: iDof[0]=3; iDof[1]=5; iDof[2]=4; break;
    case FmJointBase::rXYZ: iDof[0]=3; iDof[1]=4; iDof[2]=5; break;
    case FmJointBase::rYZX: iDof[0]=4; iDof[1]=5; iDof[2]=3; break;
    case FmJointBase::rZXY: iDof[0]=5; iDof[1]=3; iDof[2]=4; break;
    default: return 1;
    }

  if (aJoint->getUserDescription().find("#RotAxisParam") != std::string::npos)
    ListUI <<"\n---> WARNING: Ignoring #RotAxisParam"
	   <<" in the description field for "<< aJoint->getIdString()
	   <<".\n     Use the \"Rotation formulation\" menu in the"
	   <<" \"Advanced\" tab of the joint property window instead.\n";

  int iMat[3];
  switch (aJoint->rotFormulation.getValue())
    {
    case FmJointBase::ROT_AXIS:
      iDof[0]=3; iDof[1]=4; iDof[2]=5; // ignore rotation sequence settings
      iMat[0]=1; iMat[1]=1; iMat[2]=1;
      break;
    case FmJointBase::FOLLOWER_AXIS:
      iMat[0]=3; iMat[1]=2; iMat[2]=1;
      break;
    case FmJointBase::ORTHOGONAL_AXIS:
      iMat[0]=1; iMat[1]=2; iMat[2]=3;
      break;
    default:
      return 1;
    }

  fprintf(myFile,"  %s %d %d   %d %d   %d %d\n", jointVarDefs,
	  iDof[0]+1,iMat[0], iDof[1]+1,iMat[1], iDof[2]+1,iMat[2]);
  return 0;
}


int FmSolverParser::writeContactElement(FmCamJoint* activeJoint)
{
  fprintf(myFile,"&CONTACT_ELEMENT\n");
  activeJoint->printID(myFile);

  // Thickness and width of contact surface
  if (activeJoint->getUserDescription().find("#Width") != std::string::npos)
    ListUI <<"\n---> WARNING: Ignoring #Width <w>"
	   <<" in the description field for "<< activeJoint->getIdString()
	   <<".\n     Use the \"Width\" field"
	   <<" in the joint property window instead.\n";

  if (activeJoint->isUsingRadialContact())
    fprintf(myFile,"  radius =%17.9e\n", activeJoint->getThickness());
  else
    fprintf(myFile,"  thickness =%17.9e\n", activeJoint->getThickness());
  fprintf(myFile,"  width =%17.9e\n", activeJoint->getWidth());

  // Write springs
  fprintf(myFile,"  springId = %d %d %d %d %d %d\n",
	  activeJoint->getSpringBaseID(0),
	  activeJoint->getSpringBaseID(1),
	  activeJoint->getSpringBaseID(2),
	  activeJoint->getSpringBaseID(3),
	  activeJoint->getSpringBaseID(4),
	  activeJoint->getSpringBaseID(5));

  // Write dampers
  fprintf(myFile,"  damperId = %d %d %d %d %d %d\n",
	  activeJoint->getDamperBaseID(0),
	  activeJoint->getDamperBaseID(1),
	  activeJoint->getDamperBaseID(2),
	  activeJoint->getDamperBaseID(3),
	  activeJoint->getDamperBaseID(4),
	  activeJoint->getDamperBaseID(5));

  int nVar = writeFriction(activeJoint,IntVec(1,6)) ? 4 : 2;

  // Contact variables to be saved:
  // 1 - Deflection
  // 2 - Velocity
  // 3 - Friction force
  // 4 - Friction energies
  activeJoint->writeSaveVar(myFile,nVar);

  // Follower triad
  fprintf(myFile,"  followerTriad = %d\n", activeJoint->getSlaveTriad()->getBaseID());

  // Reference to contact surface
  if (activeJoint->getMaster())
    fprintf(myFile,"  contactSurface = %d\n", activeJoint->getMaster()->getBaseID());

  fprintf(myFile,"/\n\n");
  return 0;
}


int FmSolverParser::writeFriction(FmJointBase* aJoint, const IntVec& iDof)
{
  FmFrictionBase* aFriction = aJoint->getFriction();
  if (!aFriction) return false;

  int fId[7] = { 0, 0, 0, 0, 0, 0, 0 };

  size_t i, nDof = iDof.size();
  for (i = 0; i < nDof; i++)
    if (iDof[i] == aJoint->getFrictionDof())
      fId[i] = aFriction->getBaseID();
  fprintf(myFile,"  frictionSetId =");
  for (i = 0; i < nDof; i++)
    fprintf(myFile," %d", fId[i]);
  fprintf(myFile,"\n");

  if (aFriction->getStickStiffness() > 0.0) {
    for (i = 0; i < nDof; i++)
      if (fId[i]) fId[i] = aJoint->getBaseID();
    fprintf(myFile,"  frictionSpringId =");
    for (i = 0; i < nDof; i++)
      fprintf(myFile," %d", fId[i]);
  }

  // Beta feature: Hydro-, Skin- and Radial friction for pipes (DrillSim)
  FFaString fDesc = aFriction->getUserDescription();
  if (fDesc.hasSubString("#PipeRadius")) {
    FFaString fDesc = aFriction->getUserDescription();
    double pipeRadius = fDesc.getDoubleAfter("#PipeRadius");
    double outerPipeRadius = fDesc.getDoubleAfter("#OuterPipeRadius");
    double hydroFric = fDesc.getDoubleAfter("#HydroFric");
    double skinFric = fDesc.getDoubleAfter("#SkinFric");
    double radFric = fDesc.getDoubleAfter("#RadFric");
    fprintf(myFile, " pipeRadius = %f\n", pipeRadius);
    fprintf(myFile, " outerPipeRadius = %f\n", outerPipeRadius);
    fprintf(myFile, " hydroFricCoeff = %f\n", hydroFric);
    fprintf(myFile, " skinFricCoeff = %f\n", skinFric);
    fprintf(myFile, " radFricCoeff = %f\n", radFric);
  }

  // Beta feature: User-defined normal force via an engine
  FFaString jDesc = aJoint->getUserDescription();
  int fricFEngine = jDesc.getIntAfter("#FrictionForceEngine");
  if (fricFEngine > 0) {
    for (i = 0; i < nDof; i++)
      if (fId[i]) fId[i] = fricFEngine;
    fprintf(myFile,"  frictionEngineId =");
    for (i = 0; i < nDof; i++)
      fprintf(myFile," %d", fId[i]);
    FmEngine::betaFeatureEngines.insert(fricFEngine);
  }

  return true;
}


int FmSolverParser::writeSprings()
{
  int err = 0;

  std::vector<FmAxialSpring*> allAxialSprings;
  FmDB::getAllAxialSprings(allAxialSprings);

  for (FmAxialSpring* activeSpring : allAxialSprings)
  {
    fprintf(myFile,"! Axial spring\n");
    err += activeSpring->FmSpringBase::printSolverEntry(myFile);
  }

  // Lambda function checking if a Cam joint DOF is fixed or not,
  // through a description-field command (beta feature)
  auto&& isFixedCamDof = [](FmJointBase* joint, FmJointSpring* spr)
  {
    if (!joint->isOfType(FmCamJoint::getClassTypeID()))
      return false;

    char FixDof[6] = "#FixX";
    FixDof[4] += spr->getDOF();
    return FFaString(joint->getUserDescription()).hasSubString(FixDof);
  };

  std::vector<FmJointSpring*> jointSprings;
  FmDB::getAllJointSprings(jointSprings);

  for (FmJointSpring* activeSpring : jointSprings)
  {
    if (!activeSpring->getActiveOwner())
      continue; // joint dof is not SPRING_CONSTRAINED

    FmJointBase* activeJoint = activeSpring->getOwnerJoint();
    if (activeJoint->isSuppressed())
      continue; // dependent triad is suppressed

    if (activeJoint->isContactElement())
      fprintf(myFile,"! Contact element spring\n");
    else if (isFixedCamDof(activeJoint,activeSpring))
      continue; // the Cam joint does not have this dof, ignore spring
    else if (activeJoint->isGlobalSpringElement())
      fprintf(myFile,"! Global spring\n");
    else
      fprintf(myFile,"! Joint spring\n");

    err += activeSpring->printSolverEntry(myFile);
  }

  return err;
}


int FmSolverParser::writeSensors()
{
  int err = 0;

  std::set<int> writtenSensors;
  std::vector<FmEngine*> allEngines;
  FmDB::getAllEngines(allEngines);

  for (FmEngine* engine : allEngines)
    if (engine->isActive())
    {
      int lerr = err;
      size_t nArgs = engine->getNoArgs();
      for (size_t j = 0; j < nArgs; j++)
      {
        // Avoid writing any sensors more than once.
        // set<int>::insert() returns a pair<iterator,bool> where the bool
        // tells whether the element was actually inserted or already present.
        int sensorId = engine->getSensorId(j);
        if (sensorId > 0 && writtenSensors.insert(sensorId).second)
        {
          FmSensorBase* sensor = engine->getSensor(j);
          fprintf(myFile,"&SENSOR\n");
          fprintf(myFile,"  id = %d\n",sensorId);
          sensor->printID(myFile,false);
          err += sensor->printSolverData(myFile,engine,j);
          fprintf(myFile,"/\n\n");
        }
      }
      if (err > lerr)
        ListUI <<"---> ERROR: "<< engine->getIdString(true) <<" is inconsistent\n";
    }

  if (err > 0)
    ListUI <<"     A total of "<< err <<" Sensor error(s) were detected.\n";

  return err;
}


int FmSolverParser::writeRosettes(const std::vector<FmPart*>& gageParts)
{
  int nros = 0;

  std::vector<FmModelMemberBase*> rosettes;
  FmDB::getAllOfType(rosettes,FmStrainRosette::getClassTypeID());

  for (FmPart* part : gageParts)
    for (FmModelMemberBase* ros : rosettes)
      if (static_cast<FmStrainRosette*>(ros)->rosetteLink.getPointer() == part)
        nros += ros->printSolverEntry(myFile);

  return nros;
}


int FmSolverParser::writeAllOfType(int classTypeID)
{
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,classTypeID);

  int err = 0;
  for (FmModelMemberBase* obj : objs)
    err += obj->printSolverEntry(myFile);

  if (err > 0)
  {
    std::set<std::string> objTypes;
    for (FmModelMemberBase* obj : objs) objTypes.insert(obj->getUITypeName());
    ListUI <<" ==> Detected "<< err
           <<" error(s) while writing solver input for "<< *objTypes.begin();
    objTypes.erase(objTypes.begin());
    for (const std::string& objType : objTypes) ListUI <<"s, "<< objType;
    ListUI <<"s\n";
  }
  return err;
}
