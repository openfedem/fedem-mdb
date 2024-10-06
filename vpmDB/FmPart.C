// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaAlgebra/FFaBody.H"
#ifdef FT_USE_CONNECTORS
#include "FFaLib/FFaGeometry/FFaLineGeometry.H"
#include "FFaLib/FFaGeometry/FFaPlaneGeometry.H"
#include "FFaLib/FFaGeometry/FFaCylinderGeometry.H"
#include "FFaLib/FFaGeometry/FFaPointSetGeometry.H"
#endif
#include "FFaLib/FFaCmdLineArg/FFaCmdLineArg.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"
#include "FFaLib/FFaOS/FFaFilePath.H"

#include "FFlLib/FFlInit.H"
#include "FFlLib/FFlIOAdaptors/FFlReaders.H"
#include "FFlLib/FFlIOAdaptors/FFlFedemWriter.H"
#include "FFlLib/FFlIOAdaptors/FFlVTFWriter.H"
#include "FFlLib/FFlFEParts/FFlNode.H"
#include "FFlLib/FFlFEParts/FFlRGD.H"
#ifdef FT_USE_VISUALS
#include "FFlLib/FFlFEParts/FFlVDetail.H"
#endif
#include "FFlLib/FFlGroup.H"
#include "FFlLib/FFlAttributeBase.H"
#include "FFlLib/FFlConnectorItems.H"
#include "FFlLib/FFlLinkHandler.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdPart.H"
#include "vpmDisplay/FdDB.H"
#endif

#include "vpmDB/FmPart.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmJointMotion.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmFileSys.H"

#include <algorithm>
#include <functional>
#include <fstream>
#include <cstring>

#if _MSC_VER > 1310
#define popen  _popen
#define pclose _pclose
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

bool FmPart::enableNonlinearFE = false;
long int FmPart::nFENodesTotal = 0;

Fmd_DB_SOURCE_INIT(FcPART, FmPart, FmLink);


FmPart::FmPart(const FaVec3& globalPos) : FmLink(globalPos)
{
  Fmd_CONSTRUCTOR_INIT(FmPart);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdPart(this);
#endif

  // Initialize fields

  FFA_FIELD_DEFAULT_INIT(BMatFile,       "B_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(EMatFile,       "E_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(GMatFile,       "G_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(MMatFile,       "M_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(SMatFile,       "S_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(LMatFile,       "L_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(DMatFile,       "D_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(FMatFile,       "F_MATRIX_FILE");
  FFA_FIELD_DEFAULT_INIT(SAMdataFile,    "SAM_DATA_FILE");
  FFA_FIELD_DEFAULT_INIT(baseFTLFile,    "BASE_FTL_FILE");
  FFA_FIELD_DEFAULT_INIT(reducedFTLFile, "REDUCED_FTL_FILE");
  FFA_FIELD_DEFAULT_INIT(originalFEFile, "ORIGINAL_FE_FILE");
  FFA_FIELD_DEFAULT_INIT(myRepository,   "PART_REPOSITORY");
  FFA_FIELD_INIT(externalSource, false,  "IMPORTED_REDUCED_MATRICES");
  FFA_FIELD_INIT(ramUsageLevel, FULL_FE, "RAM_USAGE_LEVEL");

  FFA_FIELD_DEFAULT_INIT(importConverter, "ORIGINAL_FE_FILE_CONVERSION");
  FFA_FIELD_DEFAULT_INIT(myRSD,           "PART_RSD");
  FFA_FIELD_DEFAULT_INIT(cachedChecksum,  "CACHED_CHECK_SUM");
  FFA_FIELD_INIT(savedCS, 0,              "SAVED_CS");
  FFA_FIELD_INIT(needsCSupdate, true,     "NEEDS_CS_UPDATE");

  FFA_FIELD_INIT(lockLevel, FM_ALLOW_MODIFICATIONS, "LOCK_LEVEL");
  FFA_FIELD_INIT(suppressInSolver, false, "SUPPRESS_IN_SOLVER");

  FFA_FIELD_INIT(minSize, 0,   "MINIMUM_MESH_SIZE");
  FFA_FIELD_INIT(quality, 2.0, "MESH_QUALITY");

  FFA_FIELD_INIT(useGenericProperties,           false, "USE_GENERIC_DATA");
  FFA_FIELD_INIT(condenseOutCoG,                 false, "CONDENSE_OUT_COG");
  FFA_FIELD_INIT(myGenericPartStiffType, DEFAULT_RIGID, "GEN_PART_STIFF_TYPE");

  FFA_REFERENCE_FIELD_INIT(materialField, material, "MATERIAL");

  // Define the default stiffness and mass to be used for generic parts
  double defaultTraStiff = 1.0e9; // that is, 1000 KN/mm
  double defaultRotStiff = 1.0e9; // that is, 1000 MNm/rad
  double defaultMass = 0.1; // that is, 100 grams
  // Avoid creating a mechanism object here
  FmMechanism* mech = FmDB::getMechanismObject(false);
  if (mech && mech->modelDatabaseUnits.getValue().isValid())
  {
    // Apply scaling factor from SI to current modelling units
    mech->modelDatabaseUnits.getValue().inverse(defaultTraStiff,"FORCE/LENGTH");
    mech->modelDatabaseUnits.getValue().inverse(defaultRotStiff,"FORCE*LENGTH");
    mech->modelDatabaseUnits.getValue().inverse(defaultMass,"MASS");
  }

  FFA_FIELD_INIT(kt,       defaultTraStiff, "GEN_TRANS_STIFF");
  FFA_FIELD_INIT(kr,       defaultRotStiff, "GEN_ROT_STIFF");
  FFA_FIELD_INIT(myCG, FFa3DLocation(true), "CENTER_OF_GRAVITY");

  FFA_REFERENCE_FIELD_INIT(myCGPosRefField, myCGPosRef, "CENTER_OF_GRAVITY_POS_REF");
  FFA_REFERENCE_FIELD_INIT(myCGRotRefField, myCGRotRef, "CENTER_OF_GRAVITY_ROT_REF");

  FFA_FIELD_INIT(mass,           defaultMass, "MASS");
  FFA_FIELD_INIT(inertia,    FFaTensor3(0.0), "MASS_INERTIA");
  FFA_FIELD_INIT(myCalculateMass,   EXPLICIT, "MASS_CALCULATION");
  FFA_FIELD_INIT(myInertiaRef, POS_CG_ROT_CG, "INERTIA_REFERENCE");

  FFA_FIELD_INIT(overrideChecksum, false, "OVERRIDE_CHECKSUM");

  FFA_FIELD_INIT(nGenModes,   12, "NUM_GEN_MODES");
  FFA_FIELD_INIT(nEigvalsCalc, 0, "NUM_EIGVALS_CALC");

  FFA_FIELD_INIT(useConsistentMassMatrix, false, "USE_CONSISTENT_MASS_MATRIX");
  FFA_FIELD_INIT(factorizeMassMxEigSol,   true,  "FACTORIZE_MASS_MX_EIGENSOLV");
  FFA_FIELD_INIT(expandModeShapes,        true,  "EXPAND_MODE_SHAPES");

  FFA_FIELD_INIT(tolFactorize, 1.0e-12, "TOL_FACTORIZE");
  FFA_FIELD_INIT(tolEigenval,  1.0e-8,  "TOL_EIGENVAL");

  FFA_FIELD_INIT(recoveryMatrixSavePrecision, DOUBLE_PRECISION, "RECOVERY_MATRIX_SAVE_PRECISION");

  FFA_FIELD_INIT(myCentripOption, MODEL_DEFAULT_CENTRIP_CORRECTION, "CENTRIPETAL_CORRECTION");

  FFA_FIELD_DEFAULT_INIT(myLoadCases,   "LOAD_CASES");
  FFA_FIELD_DEFAULT_INIT(myLoadFactors, "LOAD_FACTORS");
  FFA_FIELD_DEFAULT_INIT(myLoadDelays,  "LOAD_DELAYS");
  FFA_REFERENCELIST_FIELD_INIT(myLoadEnginesField, myLoadEngines, "LOAD_ENGINES");

  FFA_FIELD_INIT(recoveryDuringSolve, 0,      "RECOVERY_DURING_SOLVE");
  FFA_FIELD_INIT(useExternalResFile, false,   "USE_EXTERNAL_RESULT_FILE");
  FFA_FIELD_DEFAULT_INIT(externalResFileName, "EXTERNAL_RESULT_FILE");

  FFA_FIELD_INIT(hasBuoyancy, false, "BUOYANCY");

  if (enableNonlinearFE) {
    FFA_FIELD_INIT(useNonlinearReduction, false,  "USE_NONLINEAR_REDUCTION");
    FFA_FIELD_DEFAULT_INIT(nonlinearDataFileName, "NONLINEAR_DATA_FILE");
    FFA_FIELD_INIT(numberOfNonlinearSolutions, 0, "NUMBER_OF_NONLINEAR_SOLUTIONS");
    FFA_FIELD_INIT(nonLinStates,               0, "NUMBER_OF_NONLINEAR_STATES");
  }
  else {
    useNonlinearReduction.setValue(false);
    numberOfNonlinearSolutions.setValue(0);
    nonLinStates.setValue(0);
  }

  myFEData = NULL;

  this->setCGPosRef(this);
  this->setCGRotRef(this);
  isCGedited = false;
  fileVersion = 0;
}


/*!
  Constructor for the earth link (no field initialization or visualization)
*/

FmPart::FmPart(const char* EarthName)
{
  Fmd_CONSTRUCTOR_INIT(FmPart);

  FFA_REFERENCE_INIT(myCGPosRef);
  FFA_REFERENCE_INIT(myCGRotRef);
  FFA_REFERENCELIST_INIT(myLoadEngines);

  myFEData = NULL;

  this->setID(-1);
  this->setUserDescription(EarthName);
  this->setCGPosRef(this);
  this->setCGRotRef(this);
  isCGedited = false;
  fileVersion = 0;
}


FmPart::~FmPart()
{
  this->disconnect();

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->removeDisplayData();
#endif

  this->clearElemGroupProxies(true);

  if (myFEData)
  {
    nFENodesTotal -= myFEData->getNodeCount();
    delete myFEData;
  }
  myFEData = NULL;

  // Cannot use the getTriads method here, because the detach call on one triad
  // may detach other triads too if they are glider triads in a point-to-path joint
  FmTriad* triad = NULL;
  while (this->hasReferringObjs(triad,"myAttachedLinks"))
    triad->detach(this);
}


/*!
  Reimplemented to update the location data on all triads on this part
  when changing this parts position.

  Also move any joints whose dependent triad is connected to this part,
  if the corresponding independent triad is un-attached or attached to ground.
*/

void FmPart::setLocalCS(const FaMat34& localCS)
{
  FaMat34 oldCS = this->getGlobalCS();
  this->FmLink::setLocalCS(localCS);
  FaMat34 TrMat = this->getGlobalCS() * oldCS.inverse();

  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  for (FmTriad* triad : triads)
  {
    triad->updateLocation();

    // Update location in joints using this triad
    // as its independent triad
    std::vector<FmJointBase*> jnts;
    triad->getReferringObjs(jnts,"itsMasterTriad");
    for (FmJointBase* joint : jnts)
      joint->updateLocation();

    // Move all joints whose dependent triad is on this part, but not if
    // the independent triads are connected to other parts except for ground
    std::vector<FmSMJointBase*> sjnts;
    triad->getReferringObjs(sjnts,"itsSlaveTriad");
    for (FmSMJointBase* joint : sjnts)
      if (joint->isSlaveMovedAlong())
      {
        if (!joint->isMasterMovedAlong())
        {
          // This joint is set to move with its dependent triad only,
          // so update the joint coordinate system only
          // (relative to its independent triad)
          joint->setGlobalCS(TrMat*joint->getGlobalCS());
          joint->updateDisplayTopology();
        }
        else if (!joint->getMasterPart(true))
        {
          // This joint is set to move with both joint triads,
          // and is either un-attached or attached to ground.
          // Update the independent triad location accordingly.
          // The joint will then automatically follow with.
          FmTriad* other = joint->getItsMasterTriad();
          other->setGlobalCS(TrMat*other->getGlobalCS());
          other->updateDisplayTopology();
        }
      }

    std::vector<FmMMJointBase*> mjnts;
    triad->getReferringObjs(mjnts,"itsSlaveTriad");
    for (FmMMJointBase* joint : mjnts)
      if (!joint->isContactElement() && !joint->getMasterPart(true))
      {
        std::vector<FmTriad*> others;
        joint->getMasterTriads(others);
        for (FmTriad* other : others)
        {
          other->setGlobalCS(TrMat*other->getGlobalCS());
          other->updateDisplayTopology();
        }
      }
  }
}


/*!
  Checks if the part data files are saved in an "external" repository.
*/

FmPart::ReposType FmPart::usesRepository() const
{
  if (!myRepository.getValue().empty())
    return LINK_SPECIFIC;

  if (!FmDB::getMechanismObject()->modelLinkRepository.getValue().empty())
    return EXTERNAL_REP;

  return INTERNAL_REP;
}


/*!
  Returns the path to the directory in which the part will be saved.
  Tries to create the directory if it does not exist, if \a createDir is true.
*/

std::string FmPart::getAbsFilePath(bool createDir) const
{
  FmMechanism* mech = FmDB::getMechanismObject();
  const std::string& modPath = mech->getAbsModelFilePath();

  // First, check local part repository
  std::string filePath = myRepository.getValue();
  if (!filePath.empty())
  {
    FFaFilePath::makeItAbsolute(FFaFilePath::checkName(filePath),modPath);
    if (FmFileSys::verifyDirectory(filePath,createDir))
      return filePath;

    ListUI <<"Warning: Could not open part-specific repository for "
           << this->getIdString(true) <<"\n     --> "<< filePath
           <<"\n         Switching to ";
    if (mech->modelLinkRepository.getValue().empty())
      ListUI <<"internal part repository instead.\n";
    else
      ListUI <<"part repository \"" << mech->modelLinkRepository.getValue()
             <<"\" instead.\n";
    const_cast<FmPart*>(this)->myRepository.setValue("");
  }

  // Then, the mechanism repository
  filePath = mech->modelLinkRepository.getValue();
  if (!filePath.empty())
  {
    FFaFilePath::makeItAbsolute(FFaFilePath::checkName(filePath),modPath);
    if (FmFileSys::verifyDirectory(filePath,createDir))
      return filePath;

    ListUI <<"Warning: Could not open part repository for "
           << this->getIdString(true) <<"\n     --> "<< filePath
           <<"\n         Switching to internal part repository instead.\n";
    mech->modelLinkRepository.setValue("");
  }

  // ...and if nothing is found, use the default
  return mech->getAbsModelLRDBPath(createDir);
}


/*!
  Returns the full path of the saved part file of this part.
*/

std::string FmPart::getBaseFTLFile(bool createDir) const
{
  if (baseFTLFile.getValue().empty())
    return "";

  return FFaFilePath::appendFileNameToPath(this->getAbsFilePath(createDir),
                                           baseFTLFile.getValue());
}


/*!
  Returns the base ftl-file name (optionally without extension) of this part.
*/

std::string FmPart::getBaseFTLName(bool includExt) const
{
  const std::string& ftlFile = baseFTLFile.getValue();
  if (includExt || ftlFile.empty())
    return ftlFile;

  size_t idot = ftlFile.find_last_of('.');
  return idot < ftlFile.size() ? ftlFile.substr(0,idot) : ftlFile;
}


/*!
  Returns true if the part is saved in its associated repository.
  Always return true for generic parts if \a checkFEpartsOnly is true.
*/

bool FmPart::isSaved(bool checkFEpartsOnly) const
{
  if (checkFEpartsOnly && useGenericProperties.getValue())
    return true;

  std::string ftlFile = this->getBaseFTLFile();
  if (ftlFile.empty()) return false;

  return FmFileSys::isFile(ftlFile);
}


/*!
  Returns true if this is a used (and loaded) FE part.
*/

bool FmPart::isFEPart(bool loadedOnly) const
{
  if (useGenericProperties.getValue())
    return false;

  if (suppressInSolver.getValue())
    return false;

  return loadedOnly ? myFEData != NULL : true;
}


/*!
  Returns true if the part's FE model currently is loaded.
*/

bool FmPart::isFELoaded(bool fullOnly) const
{
  if (!myFEData)
    return false;

  if (ramUsageLevel.getValue() == FULL_FE)
    return true;

  return !fullOnly && ramUsageLevel.getValue() == SURFACE_FE;
}


/*!
  Returns true if the part's FE model is used for visualization.
*/

bool FmPart::useFEModelAsVisualization() const
{
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    if (static_cast<FdPart*>(itsDisplayPt)->isUsingGenPartVis())
      return false;
#endif
  return myFEData != NULL;
}


/*!
  Returns true if external node status changes are allowed.
*/

bool FmPart::isAttachable() const
{
  return (lockLevel.getValue() == FM_ALLOW_MODIFICATIONS ||
          lockLevel.getValue() == FM_ALLOW_LINK_EXT_NODE_MOD);
}


void FmPart::setLinkHandler(FFlLinkHandler* part, bool updateNnodes)
{
  this->clearElemGroupProxies();

  if (myFEData)
  {
    if (updateNnodes) nFENodesTotal -= myFEData->getNodeCount();
    delete myFEData;
  }

  myFEData = part;
}


void FmPart::updateCachedCheckSum()
{
  // The check-sum is needed only for FE parts that are used in the solver
  if (!needsCSupdate.getValue() || !this->isFEPart(true))
    return;

  if (cachedChecksum.getValue().getCurrent())
    ListUI <<"  -> Recalculating";
  else
    ListUI <<"  -> Calculating";
  ListUI <<" check-sum for "<< this->getIdString(true) <<": ";
  myFEData->calculateChecksum(&(cachedChecksum.getValue()),
                              fileVersion == 1 || fileVersion >= 7);
  ListUI << (size_t)cachedChecksum.getValue().getCurrent() <<"\n";
  needsCSupdate.setValue(false);
}


void FmPart::getCheckSum(FFaCheckSum& cs)
{
  this->updateCachedCheckSum();
  cs = cachedChecksum.getValue();
}


bool FmPart::hasChangedFEdata() const
{
  return myFEData && savedCS.getValue() != myFEData->calculateChecksum();
}


bool FmPart::hasStrainRosettes() const
{
  std::vector<FmModelMemberBase*> rosettes;
  FmDB::getAllOfType(rosettes,FmStrainRosette::getClassTypeID());

  for (FmModelMemberBase* obj : rosettes)
    if (static_cast<FmStrainRosette*>(obj)->rosetteLink.getPointer() == this)
      return true;

  return false;
}


bool FmPart::hasStrainCoat() const
{
  if (myFEData)
    return myFEData->getElementCount(FFlLinkHandler::FFL_STRC) > 0;

  // When the part is unloaded, we need to check the file on disk instead
  std::string fileName = this->getBaseFTLFile();
  if (!FmFileSys::isFile(fileName))
    return false;

  char cline[BUFSIZ];
  std::ifstream ftl(fileName.c_str(),std::ios::in);
  while (ftl.getline(cline,BUFSIZ,'\n'))
    if (std::string(cline,4) == "STRC")
      return true;

  return false;
}


bool FmPart::hasResults() const
{
  return this->isFEPart(false) ? !myRSD.getValue().isEmpty() : false;
}


bool FmPart::setVisDetail(const std::vector<FmElementGroupProxy*>& groups,
                          int dType)
{
#ifdef FT_USE_VISUALS
  if (!myFEData)
    // Geometry visualization
    return this->setModelType(dType == FFlVDetail::ON ?
                              FmLink::SURFACE : FmLink::OFF);

  FFlVDetail* detail;
  if (dType == FFlVDetail::OFF)
    detail = myFEData->getOffDetail();
  else
    detail = myFEData->getOnDetail();

  if (groups.empty()) // only the part itself is selected, no groups
    myFEData->setVisDetail(detail);
  else
  {
    // Groups are selected also
    std::vector<FFlPartBase*> parts;
    parts.reserve(groups.size());
    for (FmElementGroupProxy* group : groups)
      parts.push_back(group->getRealObject());
    myFEData->setVisDetail(parts,detail);
  }

  // Update the visuals of the part
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    static_cast<FdPart*>(itsDisplayPt)->updateElementVisibility();
#endif
  myFEData->updateGroupVisibilityStatus();
  return true;
#else
  return false;
#endif
}


std::string FmPart::getTaskName(const char* fmt) const
{
  std::string taskName;
  if (!this->getParentAssembly())
    taskName = FFaNumStr(fmt,this->getID());
  else if (strstr(fmt,"[")) // Assume format "[%d,%d,...] "
    taskName = this->getIdPath(true) + ' ';
  else // Assume format "%d_%d_..._"
    taskName = this->getIdPath(false) + '_';

  if (baseFTLFile.getValue().empty())
    taskName += "noname";
  else
    taskName += this->getBaseFTLName();

  return taskName;
}


void FmPart::getStickers(std::vector<FmSticker*>& stickers) const
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  this->getFreeJointEnds(triads);

  this->getLocalStickers(stickers);
  for (FmTriad* triad : triads)
    triad->getStickers(stickers);
}


int FmPart::getNumberOfTriads() const
{
  std::vector<FmTriad*> tr;
  this->getReferringObjs(tr,"myAttachedLinks");
  return tr.size();
}


FmTriad* FmPart::findTriad(int baseID) const
{
  std::vector<FmTriad*> triads;
  this->getReferringObjs(triads,"myAttachedLinks");
  for (FmTriad* triad : triads)
    if (triad->getBaseID() == baseID)
      return triad;

  return NULL;
}


void FmPart::getTriads(std::vector<FmTriad*>& triads, bool sortOnId) const
{
  triads.clear(); // optionally sorted w.r.t to user ID
  this->getReferringObjs(triads,"myAttachedLinks",sortOnId);

#ifdef FM_DEBUG
  std::cout << (sortOnId ? "S":"Uns") <<"orted Triads for "
            << this->getIdString(true);
  for (FmTriad* triad : triads) std::cout <<" "<< triad->getID();
  std::cout << std::endl;
#endif
}


void FmPart::getElementGroups(std::vector<FmElementGroupProxy*>& groups) const
{
  groups.clear(); // sorted w.r.t to user ID
  this->getReferringObjs(groups,"myOwner",true);
}


bool FmPart::isDisabled() const
{
  if (this->isEarthLink())
    return false;

  if (ramUsageLevel.getValue() == NOTHING)
    return true;

  if (useGenericProperties.getValue())
    return false;

  return myFEData == NULL;
}


bool FmPart::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmPart::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmPart::getClassTypeID()))
    return false;
  else if (depth < FmBase::SHALLOW)
    return true;

  FmPart* copyObj = static_cast<FmPart*>(obj);
  if (copyObj->myFEData)
    myFEData = new FFlLinkHandler(*(copyObj->myFEData));

  return true;
}


/*!
  \brief Attaching a triad when having an old triad in the same place.

  This is what has to be done in this method:
  * If the triad to attach is a dependent joint triad
    and the old triad has boundary conditions,
    inquire if the boundary conditions should be moved to the joint instead.
  * Clone properties from the old triad to the attached one
  * Erase the old triad

  HOW THE COORDINATE SYSTEMS FOR THE TRIADS ARE CHECKED AND MATCHED
  -----------------------------------------------------------------
  Master = Independent joint triad
  Slave  = Dependent joint triad

  Old triad                    New triad
  ---------                    ---------
  Master    --- cs check >>--  Master >> if the CSs are the same: OK
  Master    ---- XX ----       Slave  (except FreeJoint: same as None->Slave)
  Master    -- cs -->          None w/o dirs

  Slave     ---- XX ----       Master (except FreeJoint: same as None->Master)
  Slave     ---- XX ----       Slave  (except FreeJoint: same as None->Slave)
  Slave     -- cs -->          None

  None      <-- cs --          Master
  None      <-- cs --          Slave
  None      -- cs -->          None
*/

bool FmPart::attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool isSilent)
{
#ifdef FM_DEBUG
  isSilent = false;
#endif

  // In case the attached Triad is plotted
  std::vector<FmModelMemberBase*> curves;
  attachTr->getReferringObjs(curves,"myResultObject[XAXIS]");
  attachTr->getReferringObjs(curves,"myResultObject[YAXIS]");
  attachTr->releaseReferencesToMe("myResultObject[XAXIS]",oldTr);
  attachTr->releaseReferencesToMe("myResultObject[YAXIS]",oldTr);
  for (FmModelMemberBase* obj : curves) obj->onChanged();

  // If the attachTr is a dependent joint triad
  // and the oldTr has boundary conditions,
  // inquire whether they should be assigned to the joint DOFs instead
  int answer = -1;
  FmJointBase* joint = NULL;
  if (attachTr->hasReferringObjs(joint,"itsSlaveTriad"))
    for (int d = 0; d < 6; d++)
    {
      FmHasDOFsBase::DOFStatus stat = oldTr->getStatusOfDOF(d);
      if (stat > FmHasDOFsBase::FREE)
      {
	if (answer < 0)
	{
	  answer = FFaMsg::dialog("The existing Triad on the Part you are attaching to "
				  "has boundary conditions.\nDo you want to move "
				  "these properties to the joint DOFs instead?",
				  FFaMsg::YES_NO_CANCEL);
	  if (answer == 2) return false;
	}
	if (answer == 1 && joint->isLegalDOF(d))
	{
	  FmHasDOFsBase::DOFStatus jstat = joint->getStatusOfDOF(d);
	  if (jstat == FmHasDOFsBase::FREE ||
	      jstat == FmHasDOFsBase::FREE_DYNAMICS)
	  {
	    // Transfer the triad BC over to the joint DOF
	    joint->setStatusForDOF(d,stat);
	    if (stat == FmHasDOFsBase::PRESCRIBED)
	      joint->getMotionAtDOF(d,true)->clone(oldTr->getMotionAtDOF(d),
						   FmBase::DEEP_REPLACE);
	  }
	}
	// The dependent triad can only have FREE dofs
	oldTr->setStatusForDOF(d,FmHasDOFsBase::FREE);
	oldTr->setMotionAtDOF(d,NULL,true);
      }
    }

  // Store the FE Node number on the attached triad, so that we can
  // apply it after disconnect/connect when we have no part data
  int FENodeNr = oldTr->FENodeNo.getValue();

#ifdef FT_USE_CONNECTORS
  // Store the connector elements from the old triad
  FFlConnectorItems conItems = oldTr->itsConnectorItems.getValue();
  // Remove them, to avoid that they are erased from the part on disconnect
  oldTr->itsConnectorItems = FFlConnectorItems();
#endif

  // Disconnect both triads so that the coordinate system is correct
  oldTr->disconnect();
  attachTr->disconnect();

  // If the attachTr is a joint triad, and the oldTr is not:
  // Use the CS from the new triad (by applying the CS from the new triad
  // to the old one...)

  if (!oldTr->importantDirections() && attachTr->importantDirections())
  {
    oldTr->setGlobalCS(attachTr->getGlobalCS());
    if (!isSilent)
      ListUI <<"Warning: The coordinate system of "<< oldTr->getIdString()
	     <<" is changed to match "<< attachTr->getIdString() <<".\n";
  }

  // Clone the old triad with values from the new
  attachTr->clone(oldTr, FmBase::DEEP_REPLACE);
  // Set the attachTr ID to the ID of the oldTr
  attachTr->setID(oldTr->getID());
  // Connect the attachTr once again
  attachTr->connect(this);

  // If connect is not able to find the FE node, set back the one we had
  if (!myFEData)
    attachTr->FENodeNo.setValue(FENodeNr);

#ifdef FT_USE_CONNECTORS
  // Set the connector elements into the new triad
  attachTr->itsConnectorItems = conItems;
#endif

  // Remove the cloned triad
  return oldTr->erase();
}


/*!
  Returns any existing triad that is positioned at the same spot as \a triad
  on this part, and whether \a triad is attachable in case no triad is found.
  Used internally by FmLink::isTriadAttachable.
*/

std::pair<FmTriad*,bool> FmPart::getExistingTriad(const FmTriad* triad)
{
  // Search for an existing triad at this location
  FaVec3 point = this->getGlobalCS().inverse() * triad->getGlobalTranslation();
  double positionTol = FmDB::getPositionTolerance();

  if (useGenericProperties.getValue())
    return std::make_pair(this->getTriadAtPoint(point,positionTol),true);

  else if (myFEData)
  {
    // Find the FE node, if any, at this triads location
    FFlNode* tmpNode = this->getNodeAtPoint(point,positionTol);
    if (!tmpNode)
    {
      bool allow3DOF = false;
      FFaCmdLineArg::instance()->getValue("allow3DofAttach",allow3DOF);
      ListUI <<"Error: "<< triad->getIdString() <<" is not coincident";
      if (allow3DOF)
        ListUI <<" with any 3- or 6-DOF FE node in "<< this->getIdString(true);
      else
        ListUI <<" with any 6-DOF FE node in "<< this->getIdString(true);
      ListUI <<".\n       Verify that the closest FE node in this Part do has";
      if (allow3DOF)
        ListUI <<" at least 3 DOFs,\n";
      else
        ListUI <<" 6 DOFs,\n";
      ListUI <<"       and that the Triad is properly positioned.\n";
    }
    else if (tmpNode->isSlaveNode() &&
	     lockLevel.getValue() != FM_ALLOW_MODIFICATIONS)
      ListUI <<"Error: "<< triad->getIdString()
	     <<" is coincident with a dependent node of a\n       rigid- or"
	     <<" interpolation constraint element in the FE model,\n"
	     <<"       and can therefore not be attached to "
	     << this->getIdString(true) <<".\n";
    else
      // Get the existing triad associated with this node, if any
      return std::make_pair(this->getTriadAtNode(tmpNode->getID()),true);
  }
  else
  {
    // FE data is not loaded, we must have an existing triad here
    FmTriad* existingTriad = this->getTriadAtPoint(point,positionTol);
    if (existingTriad || !this->isAttachable())
      return std::make_pair(existingTriad,true);

    ListUI <<"Error: "<< triad->getIdString()
	   <<" is not coincident with any of the Triads already\n"
	   <<"       attached to "<< this->getIdString(true)
	   <<". If you want to attach a Triad to a new\n       FE node,"
	   <<" you will have to load the FE data of this Part first.\n";
  }

  ListUI <<"       Also verify that the modeling tolerance ("<< positionTol
	 <<") used by\n       the point coincidence check is appropriate.\n";

  return { static_cast<FmTriad*>(NULL), false };
}


/*!
  Returns the triad associated with the given FE node.
  Returns NULL if no triad is found.
*/

FmTriad* FmPart::getTriadAtNode(int nodeNo) const
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  for (FmTriad* triad : triads)
    if (triad->FENodeNo.getValue() == nodeNo)
      return triad;

  return NULL;
}


/*!
  Returns the closest FE node to point using tolerance, or NULL if none found.
  If the found node is a dependent FE node, a new node is created by adding a
  stiff CBUSH element between the dependent node and the new node.
  The new node is then returned.
*/

FFlNode* FmPart::getNodeAtPoint(const FaVec3& point, double tolerance,
				FFlConnectorItems* addItems)
{
  if (!myFEData) return NULL;

  bool allow3DOF = false; // Look for nodes having (at least) three nodal DOFs
  FFaCmdLineArg::instance()->getValue("allow3DofAttach",allow3DOF);
  int dofFilter = allow3DOF ? FFlNode::FFL_THREE_DOFS : FFlNode::FFL_SIX_DOFS;

  // If more than one node matches the point, prefer the non-dependent nodes.
  // If 3-DOF nodes are allowed, prefer 6-DOF nodes if more than one
  // non-dependent nodes matches.
  FFlNode* attachNode = myFEData->findFreeNodeAtPoint(point,tolerance,dofFilter);
  if (attachNode && attachNode->isSlaveNode() && addItems)
    if (addItems->empty() && lockLevel.getValue() == FM_ALLOW_MODIFICATIONS)
    {
      // Must create a new node at this location.
      // Use the position of the existing dependent node instead of given point
      // to ensure the new node is created at exactly the same location.
      FaVec3 pos = attachNode->getPos();
      attachNode = myFEData->createAttachableNode(attachNode,pos,addItems);
      needsCSupdate.setValue(true);
    }

  return attachNode;
}


int FmPart::getNodeIDAtPoint(const FaVec3& point, double tolerance)
{
  FFlNode* node = this->getNodeAtPoint(point,tolerance);
  return node ? node->getID() : -1;
}


FFlNode* FmPart::getNode(int nodeNo) const
{
  return myFEData ? myFEData->getNode(nodeNo) : NULL;
}


int FmPart::getNodePos(int nodeNo, double* x, double* y, double* z) const
{
  if (!myFEData) return -1;

  FFlNode* node = myFEData->getNode(nodeNo);
  if (!node) return -1;

  if (x && y && z)
  {
    FaVec3 pos = node->getPos();
    *x = pos.x();
    *y = pos.y();
    *z = pos.z();
  }

  return node->getStatus();
}


FFlNode* FmPart::getClosestNode(const FaVec3& point) const
{
  return myFEData ? myFEData->findClosestNode(point) : NULL;
}


void FmPart::getFreeJointEnds(std::vector<FmTriad*>& triads) const
{
  // Note that the triads vector is not cleared on entry
  std::vector<FmJointBase*> joints;
  this->getJoints(joints);

  for (FmJointBase* joint : joints)
  {
    std::vector<FmTriad*> jointTriads;
    joint->getMasterTriads(jointTriads);
    jointTriads.insert(jointTriads.begin(),joint->getSlaveTriad());
    for (FmTriad* triad : jointTriads)
      if (!triad->isAttached())
        triads.push_back(triad);
  }
}


bool FmPart::getBBox(FaVec3& max, FaVec3& min) const
{
  if (myFEData && !useGenericProperties.getValue())
    return myFEData->getExtents(max,min);

  return this->FmLink::getBBox(max,min);
}


void FmPart::updateMassProperties()
{
  // Not needed for suppressed parts
  if (suppressInSolver.getValue())
    return;

  FaVec3 CoG;
  if (myFEData && myCalculateMass.getValue() == FROM_FEM)
  {
    // Calculate mass properties from the FE data
    ListUI <<"  -> Calculating mass properties for "
           << this->getIdString(true) <<"\n";
    myFEData->getMassProperties(mass.getValue(),CoG,inertia.getValue());
    myInertiaRef.setValue(FmPart::POS_CG_ROT_CS);
  }
  else if (!useGenericProperties.getValue())
    return; // FE part with unavailable FE data ==> no mass calculation

  else
  {
    // We have a generic part without a FE data file.
    // Calculate mass properties from the associated CAD data file, if any.
    std::string cadFile = this->getGeometryFile();
    if (!cadFile.empty() && myCalculateMass.getValue() == FROM_GEOMETRY)
    {
      ListUI <<"  -> Calculating mass properties for "<< this->getIdString(true)
             <<"\n     based on CAD geometry in "
             << FmDB::getMechanismObject()->getRelativePath(cadFile) <<"\n";
      double Volume = 0.0;
      FFaTensor3 Inertia;
      FFaBody::prefix = FFaFilePath::getPath(cadFile);
      std::ifstream is(cadFile.c_str(),std::ios::in);
      FFaBody* body = is ? FFaBody::readFromCAD(is) : NULL;
      bool ok = body ? body->computeTotalVolume(Volume,CoG,&Inertia) : false;
      if (ok)
      {
        double rho = material.isNull() ? 7850.0 : material->Rho.getValue();
        double lScale = 1.0;
        visDataFileUnitConverter.getValue().convert(lScale,"LENGTH");
        CoG *= lScale;
        mass.setValue(Volume*rho*pow(lScale,3.0));
        inertia.setValue(Inertia*rho*pow(lScale,5.0));
        myInertiaRef.setValue(FmPart::POS_CG_ROT_CS);
      }
      else
        ListUI <<"     Failed.\n";
      delete body;
      if (!ok) return;
    }
    else if (!isCGedited && myCG.isDefault())
    {
      // Get triads
      std::vector<FmTriad*> triads;
      this->getTriads(triads);
      if (triads.empty()) return;

      // FE data is unavailable and CoG is still zero,
      // estimate the CoG from the triad positions
      ListUI <<"  -> Estimating CoG for "<< this->getIdString(true) <<"\n";
      for (FmTriad* triad : triads)
        CoG += triad->getLocalTranslation(this);
      CoG /= (double)triads.size();
    }
    else
      return;
  }

  this->setCGPosRef(this);
  this->setCGRotRef(this);
  this->setPositionCG(CoG);
}


int FmPart::getCompModesFlags(IntVec& BC) const
{
  int nDOFs = nGenModes.getValue();
  if (nDOFs < 1) return 0;

  // Beta feature: Suppression of individual component modes
  FFaString descr = this->getUserDescription();

  if (descr.hasSubString("#ExclModes"))
  {
    BC = IntVec(nDOFs,1);
    int* exclm = new int[nDOFs];
    int nModes = descr.getIntsAfter("#ExclModes",nDOFs,exclm);
    for (int i = 0; i < nModes; i++)
      if (exclm[i] > 0 && exclm[i] <= nDOFs)
        BC[--exclm[i]] = 0;
    delete[] exclm;
  }
  else if (descr.hasSubString("#InclModes"))
  {
    BC = IntVec(nDOFs,0);
    int* inclm = new int[nDOFs];
    int nModes = descr.getIntsAfter("#InclModes",nDOFs,inclm);
    for (int i = 0; i < nModes; i++)
      if (inclm[i] > 0 && inclm[i] <= nDOFs)
        BC[--inclm[i]] = 1;
    delete[] inclm;
  }
  else
    return 0;

  // Check if we have any fixed component modes
  int nModes = 0;
  for (int flag : BC)
    if (flag) nModes++;

  return nModes > 0 && nModes < nDOFs ? nModes : 0;
}


bool FmPart::getCompModesAlpha(DoubleVec& alpha, int type) const
{
  int nDOFs = nGenModes.getValue();
  if (nDOFs < 1) return false;

  // Beta feature: Individual component modes damping coefficients
  const char* Tag = (type == 1 ? "#Alpha1" : "#Alpha2");
  FFaString descr = this->getUserDescription();
  if (!descr.hasSubString(Tag))
    return false;

  double* a1 = new double[nDOFs];
  int nAlpha = descr.getDoublesAfter(Tag,nDOFs,a1);
  if (nAlpha < 1)
  {
    delete[] a1;
    return false;
  }

  IntVec BC(nDOFs,1);
  this->getCompModesFlags(BC);
  alpha.resize(nDOFs,0.0);

  int i, j = 0;
  for (i = 0; i < nAlpha; i++)
  {
    while (BC[j] == 0 && j < nDOFs) j++;
    if (j >= nDOFs) break;
    alpha[j++] = a1[i];
  }
  while (j < nDOFs)
    alpha[j++] = a1[nAlpha-1];

  delete[] a1;
  return true;
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmPart::writeFMF(std::ostream& os)
{
  this->updateCachedCheckSum();

  os <<"PART\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmPart::readAndConnect(std::istream& is, std::ostream&)
{
  FmPart* obj = new FmPart();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  obj->isCGedited = true;
  obj->connect();
  return true;
}


void FmPart::initAfterResolve()
{
  this->FmLink::initAfterResolve();

  FFaFilePath::checkName(myRepository.getValue());

  // Ensure the the load fields are of equal size
  // (in case someone has tampered with the model file)
  size_t nlc = myLoadCases.getValue().size();
  if (nlc > 0) {
    myLoadFactors.getValue().resize(nlc,0.0);
    myLoadDelays.getValue().resize(nlc,0.0);
  }
}


bool FmPart::setVisualizationFile(const std::string& fileName, bool updateViz)
{
#ifdef USE_INVENTOR
  if (FdDB::getCadFileType(fileName) == FdDB::FD_UNKNOWN_FILE)
    return false;
#endif
  if (!visDataFile.setValue(fileName))
    return false;

  if (!fileName.empty())
  {
    FFaMsg::pushStatus("Deleting FE Data");
#ifdef USE_INVENTOR
    if (itsDisplayPt && updateViz)
      static_cast<FdPart*>(itsDisplayPt)->removeVisualizationData(true);
#endif
    this->setLinkHandler(NULL);
    FFaMsg::popStatus();
  }
  else if (!myFEData)
  {
    FFaMsg::pushStatus("Reading FE Data");
    this->openFEData();
    FFaMsg::popStatus();
  }
  if (updateViz)
  {
    FFaMsg::pushStatus("Creating Visualization");
    this->draw();
    FFaMsg::popStatus();
  }
  return true;
}


/*!
  Static (private) method to search for the original FE file of a part,
  as given by \a fileName on input, in the following possible locations:
  - The original FE part location, as given in the model file
  - The original FE part, in a sub-folder of current model file path
  - The original FE part, in a parallel folder to current model file path
  - The original FE part, next to the model file
  - (new) ftl file, next to model file, unless this is the same as the original
    part file (for this case, ignore the specified unit calculator).
    The latter case is considered only when \a useUnitCalc is true on input.
*/

bool FmPart::locateOriginalFEfile(std::string& fileName, bool& useUnitCalc)
{
  if (fileName.empty()) return false;

  const std::string& modelFilePath = FmDB::getMechanismObject()->getAbsModelFilePath();

  std::string origFEfile(fileName);
  FFaFilePath::checkName(fileName);
  FFaFilePath::makeItAbsolute(fileName,modelFilePath);

  // Checking original FE file, original location
  if (FmFileSys::isFile(fileName))
    return true;

  // Stripping path from original FE file
  std::string origName = FFaFilePath::getFileName(origFEfile);

  std::string failedPaths;
  if (FFaFilePath::isRelativePath(origFEfile))
    failedPaths = "\t" + fileName + "\n";
  else
  {
    failedPaths = "\t" + origFEfile + "\n";

    // Try the original FE file path converted to a relative path, assuming it
    // was stored in a sub-folder, or in a parallel folder to the model file
    // (that are the most likely scenarios)
    std::string origPath = FFaFilePath::getPath(origFEfile,false);
    std::string origSdir = FFaFilePath::getFileName(origPath);
    std::string orelPath = FFaFilePath::appendFileNameToPath(origSdir,origName);
    fileName = FFaFilePath::appendFileNameToPath(modelFilePath,orelPath);

    // Checking original FE file, in a sub-folder
    if (FmFileSys::isFile(fileName))
      return true;

    if (failedPaths.find(fileName) > failedPaths.size())
      failedPaths += "\t" + fileName + "\n";

    std::string parentPath = FFaFilePath::getPath(modelFilePath);
    fileName = FFaFilePath::appendFileNameToPath(parentPath,orelPath);

    // Checking original FE file, in a parallel folder
    if (FmFileSys::isFile(fileName))
      return true;

    if (failedPaths.find(fileName) > failedPaths.size())
      failedPaths += "\t" + fileName + "\n";
  }

  fileName = FFaFilePath::appendFileNameToPath(modelFilePath,origName);

  // Checking original FE file, next to model file
  if (FmFileSys::isFile(fileName))
    return true;

  if (failedPaths.find(fileName) > failedPaths.size())
    failedPaths += "\t" + fileName + "\n";

  if (useUnitCalc)
  {
    fileName = FFaFilePath::getBaseName(fileName) + ".ftl";

    // Last resort, checking if there are any ftl file next to model file
    useUnitCalc = false; // assuming found ftl-file is already properly scaled
    if (FmFileSys::isFile(fileName))
      return true;

    if (failedPaths.find(fileName) > failedPaths.size())
      failedPaths += "\t" + fileName + "\n";
  }

  ListUI <<"  -> Search for the FE data file at these locations also failed:\n";
  FFaMsg::list(failedPaths,true);
  fileName = "";

  return false;
}


std::string FmPart::locateFEfile(const std::string& originalName)
{
  std::string fileName(originalName);
  bool usePartUnitCalc = false; // Skip the last-resort ftl-check
  if (locateOriginalFEfile(fileName,usePartUnitCalc))
    return fileName;
  else
    return originalName;
}


/*!
  Imports part from an external source (other file format, other location etc).
  \a fileName is here supposed to be a full-path name of an existing file.
*/

bool FmPart::importPart(const std::string& fileName,
			const FFaUnitCalculator* conv,
			bool storeRelativePath,
			bool autoRefNodeTriads)
{
  FFaMsg::list("  -> Importing FE data file " + fileName);

  // Set the name of the imported FE data file
  std::string newFEFile;
  if (storeRelativePath)
    newFEFile = FmDB::getMechanismObject()->getRelativePath(fileName);
  else
    newFEFile = fileName;

  if (ramUsageLevel.getValue() == NOTHING)
  {
    // Skip import of FE data for this part
    FFaMsg::list(" (Disabeled)\n");
    originalFEFile.setValue(newFEFile);
    return true;
  }

  if (!this->renewFEmodel())
    return false;

  // Check if we shall allow triad attachments to dependent RGD nodes
  FFaCmdLineArg::instance()->getValue("allowDepAttach",
                                      FFlRGDTopSpec::allowSlvAttach);

  // Check if we shall convert all parabolic elements to linear elements
  FFaCmdLineArg::instance()->getValue("convertToLinear",fileVersion);
  FFlReaders::convertToLinear = fileVersion;

  // Read and interpret the part data file
  if (FFlReaders::instance()->read(fileName,myFEData) > 0)
    FFaMsg::list(" ...OK\n");
  else
  {
    // Read failure, set lock level to avoid usage of this part
    if (myFEData->isTooLarge())
      lockLevel.setValue(FM_DENY_LINK_USAGE);
    delete myFEData;
    myFEData = NULL;
    fileVersion = 0;
    return false;
  }

  fileVersion = 1; // Don't care about file version on import
  nFENodesTotal += myFEData->getNodeCount();

  needsCSupdate.setValue(true);
  originalFEFile.setValue(newFEFile);
  importConverter.setValue(conv ? *conv : FFaUnitCalculator());

  // Convert units, unless the unit converter is only one-to-one (default)
  if (!importConverter.isDefault())
    myFEData->convertUnits(&(importConverter.getValue()));

  std::vector<FFlNode*> rNodes;
  if (autoRefNodeTriads) // Automatically create triads at reference nodes
    myFEData->getRefNodes(rNodes);

  // Create triads at the external nodes from the imported file
  double positionTol = FmDB::getPositionTolerance();
  for (NodesCIter it = myFEData->nodesBegin(); it != myFEData->nodesEnd(); ++it)
    if ((*it)->isExternal() || std::find(rNodes.begin(),rNodes.end(),*it) != rNodes.end())
    {
      // Check if a triad already exists at this nodes position
      FaVec3 point = (*it)->getPos();
      FmTriad* candidate = this->getTriadAtPoint(point,positionTol);
      if (candidate)
        // Check that the closest node to this position
        // is the same node that we want to attach
        if (this->getNodeAtPoint(candidate->getLocalTranslation(),
                                 positionTol) == *it)
        {
          // The node matches, no new triad needed
          ListUI <<"   > Existing "<< candidate->getIdString()
                 <<" matches external FE node "<< (*it)->getID() <<"\n";
          continue;
        }

      // Create a new triad at the (global) position of this node
      FmTriad* tmpTr = new FmTriad(this->getGlobalCS()*point);
      tmpTr->setParentAssembly(this->getParentAssembly());
      tmpTr->connect(this);

      ListUI <<"   > New "<< tmpTr->getIdString()
             <<" created at external FE node "<< (*it)->getID() <<"\n";
    }

  this->updateElemGroupProxies();
  this->updateMassProperties();
  this->updateLoadCases();

  bool doMemPoll = false;
  FFaCmdLineArg::instance()->getValue("memPoll",doMemPoll);
  if (doMemPoll)
  {
    std::cout << this->getLinkIDString() << std::endl;
    myFEData->dump();
  }

  // Check if the number of component modes was specified on the FE data file
  if (myFEData->getNumberOfGenDofs())
    nGenModes.setValue(myFEData->getNumberOfGenDofs());

  // Check if externally reduced matrix files were specified on the FE data file
  if (!myFEData->getOP2files().empty())
  {
    externalSource.setValue(true);
    lockLevel.setValue(FM_DENY_ALL_LINK_MOD);
    savedCS.setValue(0);
  }
  else if (FFaFilePath::isExtension(fileName,"ftl"))
  {
    // Lambda function checking for externally reduced matrix file
    auto&& checkExt = [fileName](FFaField<std::string>& field, char mType)
    {
      std::string matrixFile = FFaFilePath::getBaseName(fileName);
      matrixFile.append(1,'_');
      if (mType == 's')
        matrixFile.append("SAM.fsm");
      else
      {
        matrixFile.append(1,mType);
        matrixFile.append(".fmx");
      }
      if (!FmFileSys::isFile(matrixFile))
        return false;

      field.setValue(FmDB::getMechanismObject()->getRelativePath(matrixFile));
      ListUI <<"  -> Using externally reduced "<< field.getValue() <<"\n";
      return true;
    };

    // Check for fmx-files with same base name as the imported ftl-file
    if (checkExt(SMatFile,'S'))
    {
      checkExt(MMatFile,'M');
      checkExt(GMatFile,'G');
      checkExt(LMatFile,'L');
      checkExt(DMatFile,'D');
      checkExt(BMatFile,'B');
      checkExt(EMatFile,'E');
      checkExt(SAMdataFile,'s');
      this->setValidBaseFTLFile();
      externalSource.setValue(true);
      lockLevel.setValue(FM_DENY_ALL_LINK_MOD);
      savedCS.setValue(0);
    }
  }

  return true;
}


/*!
  Reads the FE data from a file already in the part DB.
*/

bool FmPart::openFEData()
{
  // Lambda function that re-imports the part from the given FE data file
  auto&& reImport = [this](std::string fileName)
  {
    this->readyForUpdate(true);

    bool useRelativePath = FFaFilePath::isRelativePath(fileName);
    bool usePartUnitCalc = true;
    FFaUnitCalculator* calc = NULL;
    if (!locateOriginalFEfile(fileName,usePartUnitCalc))
      return false;
    else if (usePartUnitCalc)
      calc = &importConverter.getValue();

    return this->importPart(fileName,calc,useRelativePath);
  };

  std::string readerFileName = this->getBaseFTLFile();
  if (readerFileName.empty())
  {
    // a generic part does not need an FTL-file
    if (useGenericProperties.getValue())
      return true;

    // no FTL-file name, we probably have an old model file format
    // try the file of the original import, if any...
    ListUI <<"  -> No saved FE data file for "<< this->getIdString() <<"\n";
    return reImport(originalFEFile.getValue());
  }
  else if (!FmFileSys::isFile(readerFileName))
  {
    // no part file in the PartDB (has been deleted, or..), must re-import
    FFaMsg::list("  -> Could not find " + readerFileName + "\n");
    return reImport(originalFEFile.getValue());
  }

  FFaMsg::list("  -> Reading " + readerFileName);

  if (ramUsageLevel.getValue() == NOTHING)
  {
    // Skip import of FE data for this part
    FFaMsg::list(" (Disabeled)\n");
    return true;
  }

  if (!this->renewFEmodel())
    return false;

  // Check if we shall allow triad attachments to dependent RGD nodes
  FFaCmdLineArg::instance()->getValue("allowDepAttach",FFlRGDTopSpec::allowSlvAttach);

  // Read and interpret the part data file
  fileVersion = FFlReaders::instance()->read(readerFileName,myFEData);
  if (fileVersion > 0)
  {
    FFaMsg::list(" ...OK\n");
    nFENodesTotal += myFEData->getNodeCount();

    this->updateElemGroupProxies();
    this->updateMassProperties();
    this->updateLoadCases();

    savedCS.setValue(myFEData->calculateChecksum());

    bool doMemPoll = false;
    FFaCmdLineArg::instance()->getValue("memPoll",doMemPoll);
    if (doMemPoll)
    {
      std::cout << this->getLinkIDString() << std::endl;
      myFEData->dump();
    }
  }
  else if (myFEData->isTooLarge())
  {
    // Part is larger than allowed, set lock level to avoid usage of this part
    lockLevel.setValue(FM_DENY_LINK_USAGE);
    delete myFEData;
    myFEData = NULL;
    fileVersion = 0;
    return false;
  }
  else
  {
    // Read failure, clear everything and try to re-import from original file
    delete myFEData;
    myFEData = NULL;
    fileVersion = 0;
    ListUI <<"     FE data file \""<< readerFileName <<"\" is corrupt.\n"
	   <<"     Trying original file: \""<< originalFEFile.getValue()
	   <<"\"\n";
    if (!reImport(originalFEFile.getValue()))
      return false;
  }

  needsCSupdate.setValue(true);
  return true;
}


static int execute(const std::string& program,
                   const std::string& options, bool toListUI)
{
  // Open a temporary console window, unless we already have one.
  // This is needed because all solvers are console applications,
  // and a separate console window will be opened for each one of them
  // unless the parent process has a console, TT #2889.
  bool IHaveConsole = FFaAppInfo::haveConsole();
  if (!IHaveConsole) FFaAppInfo::openConsole(true);

  ListUI <<"  -> Executing: "<< program <<" "<< options <<"\n";

  std::string command(program);
#if defined(win32) || defined(win64)
  // On windows the solver modules are not necessarily found in default $PATH
  // Assume their executables are located in the same directory as this program
  FFaFilePath::makeItAbsolute(command,FFaAppInfo::getProgramPath());
  command = "\"" + command + "\""; // Since the pathname may contain spaces
#endif
  command += " " + options;

  FILE* fd = popen(command.c_str(),"r");
  if (!fd) return -1;

  char cline[128];
  while (fgets(cline,128,fd))
    if (toListUI)
      ListUI << cline;
    else
      std::cout << cline;

  if (!IHaveConsole) FFaAppInfo::closeConsole(false);
  return pclose(fd);
}


bool FmPart::isMeshable() const
{
  if (baseCadFileName.getValue().empty() && visDataFile.getValue().empty())
    return false;

  return !FFaAppInfo::checkProgramPath("fedem_mesher").empty();
}


/*!
  Generates a tetrahedron mesh from CAD data, if any.
*/

bool FmPart::createFEData(bool parabolic)
{
  std::string cadFile = this->getGeometryFile();
  if (cadFile.empty()) return false; // no geometry file, silently ignore

  if (!FmFileSys::isFile(cadFile))
  {
    // The CAD-file is missing (possibly deleted), cannot continue
    FFaMsg::list("  -> Could not open CAD-file " + cadFile + "\n",true);
    return false;
  }

  std::string cwd = this->getAbsFilePath(true);
  std::string ftlFile = this->getBaseFTLFile();
  if (ftlFile.empty())
  {
    // Create a new ftl baseName that doesn't conflict with other parts
    // possibly using the same CAD/geometry file
    std::string baseName = FFaFilePath::appendFileNameToPath(cwd,FFaFilePath::getBaseName(cadFile,true));
    ftlFile = baseName + ".ftl";
    for (int incr = 1; FmFileSys::isFile(ftlFile); incr++)
      ftlFile = baseName + FFaNumStr("_%d.ftl",incr);
  }

  // Set up the meshing command
  std::string command = "-cwd " + cwd;
  command += " -cadFile " + FFaFilePath::getRelativeFilename(cwd,cadFile);
  command += " -partFile " + FFaFilePath::getRelativeFilename(cwd,ftlFile);

  // Specify material properties
  if (!material.isNull())
  {
    command += FFaNumStr(" -E=%g",material->E.getValue());
    command += FFaNumStr(" -Nu=%g",material->nu.getValue());
    command += FFaNumStr(" -Rho=%g",material->Rho.getValue());
  }

  // Specify minimum mesh size
  if (minSize.getValue() > 0)
    command += FFaNumStr(" -minElm=%d",minSize.getValue());

  // TetGen-specific options
  std::string tgOptions = FFaString(this->getUserDescription()).getTextAfter("#TetGen","#");
  if (quality.getValue() > 1.0)
    tgOptions += FFaNumStr("q%g",quality.getValue());

  if (parabolic)
    tgOptions += "o2";

  if (!tgOptions.empty())
    command += " -tetgen=" + tgOptions;

  // Launch the mesher sub-process
  if (execute("fedem_mesher",command,true))
  {
    FFaMsg::list("     Failed.\n",true);
    return false;
  }

  // Meshing succeeded
  this->readyForUpdate();

  FFaMsg::list("  -> Reading " + ftlFile);
  FFaMsg::pushStatus("Loading FE model");
  myFEData = new FFlLinkHandler();

  // Read and interpret the newly created part data file
  fileVersion = FFlReaders::instance()->read(ftlFile,myFEData);
  if (fileVersion > 0)
  {
    FFaMsg::list(" ...OK\n");
    nFENodesTotal += myFEData->getNodeCount();

    this->updateElemGroupProxies();
    this->updateLoadCases();

    savedCS.setValue(myFEData->calculateChecksum());
    baseFTLFile.setValue(FFaFilePath::getFileName(ftlFile));
    myMeshType.setValue(FULL);
    ramUsageLevel.setValue(FULL_FE);
    needsCSupdate.setValue(true);
    FFaMsg::popStatus();
    return true;
  }
  else if (myFEData->isTooLarge())
    lockLevel.setValue(FM_DENY_LINK_USAGE);
  else
    FFaMsg::list("  -> FE data file " + ftlFile + " is corrupt.\n",true);

  delete myFEData;
  myFEData = NULL;
  FFaMsg::popStatus();
  return false;
}


bool FmPart::getMeshParams(int* nnod, int* nel, bool* parabolic, int* ndof,
                           std::string* elmTypeCount) const
{
  if (!myFEData) return false;

  if (ndof) *ndof = myFEData->getDofCount();
  if (nnod) *nnod = myFEData->getNodeCount();
  if (nel)  *nel  = myFEData->getElementCount();

  if (elmTypeCount)
    for (const ElmTypeCount::value_type& elt : myFEData->getElmTypeCount())
      *elmTypeCount += "\t" + elt.first + FFaNumStr(":\t%7zu\n",elt.second);

  if (!parabolic) return true;

  int nt4  = myFEData->getElementCount("TET4");
  int nt10 = myFEData->getElementCount("TET10");
  if (nt4 > 0 && nt10 == 0)
    *parabolic = false;
  else if (nt4 == 0 && nt10 > 0)
    *parabolic = true;

  return true;
}


/*!
  Allocates a new part handler object for storage of FE data.
  Sets the FE part size limit for the free version of Fedem.
*/

bool FmPart::renewFEmodel()
{
  if (myFEData) return false; // logic error

  // Initialize singleton objects associated with FE parts
  FFl::initAllReaders();
  FFl::initAllElements();

  myFEData = new FFlLinkHandler();

  if (myFEData) return true;

  std::cerr <<"Error allocating FFlLinkHandler object.\n";
  return false;
}


bool FmPart::hasOP2files() const
{
  return myFEData ? !myFEData->getOP2files().empty() : false;
}


bool FmPart::convertOP2files(const std::string& absPartPath)
{
  if (!this->hasOP2files()) return true;
  if (absPartPath.empty()) return false;

  // Assuming only 6-dof triads for now
  int ndim = this->getNumberOfTriads()*6 + (int)myFEData->getNumberOfGenDofs();
  // Assuming all OP2-files have a common basename
  std::string partName = myFEData->getOP2files()[0];
  size_t lastUS = partName.rfind('_');
  if (lastUS < partName.size()) partName.erase(lastUS);

  // Create the conversion command and execute using the op2fmx utility
  std::string command = "-cwd " + absPartPath;
  command += " -partName " + partName;
  command += FFaNumStr(" -ndim=%d",ndim);
  if (execute("fedem_op2fmx",command,false))
  {
    FFaMsg::list("     Failed.\n",true);
    return false;
  }
  else
    FFaMsg::list("     Done.\n");

  // The S, M and G fmx-files should now reside in the directory absPartPath
  myFEData->clearOP2files();
  std::string path = FmDB::getMechanismObject()->getRelativePath(absPartPath);
  FFaFilePath::appendToPath(path,FFaFilePath::getFileName(partName));
  SMatFile.setValue(path + "_S.fmx");
  MMatFile.setValue(path + "_M.fmx");
  GMatFile.setValue(path + "_G.fmx");
  return true;
}


bool FmPart::copyExternalFiles(const std::string& from, const std::string& to)
{
  Strings files;
  if (!SMatFile.getValue().empty()) files.push_back(SMatFile.getValue());
  if (!MMatFile.getValue().empty()) files.push_back(MMatFile.getValue());
  if (!GMatFile.getValue().empty()) files.push_back(GMatFile.getValue());
  if (!LMatFile.getValue().empty()) files.push_back(LMatFile.getValue());
  if (files.empty()) return false;

  // Only copy the files with relative pathnames
  for (const std::string& fName : files)
    if (FFaFilePath::isRelativePath(fName))
      if (FmFileSys::copyFile(FFaFilePath::appendFileNameToPath(from,fName),
			      FFaFilePath::appendFileNameToPath(to,fName)))
	ListUI <<" "<< FFaFilePath::getFileName(fName);

  return true;
}


bool FmPart::saveFEData(bool forceSave)
{
  if (!this->saveCadData() || !myFEData)
    return false;

  // Check if we have saved before and not changed CS
  unsigned int newCS = myFEData->calculateChecksum();

  if (!forceSave && savedCS.getValue() == newCS && this->isSaved())
    return false;

  // Find a valid base name that does not conflict with the other parts
  this->setValidBaseFTLFile(newCS);

  if (forceSave)
  {
    if (newCS == savedCS.getValue() && this->isSaved(false))
      return false; // This file has already been written by another part
    else
      ListUI <<"     ["<< this->getID() <<"] "<< baseFTLFile.getValue();
  }

  // Save part data to the FTL-file
  if (this->exportPart(this->getBaseFTLFile(true),false,true))
    savedCS.setValue(newCS);
  else
    return false;

  if (!forceSave)
    ListUI <<"  -> "<< this->getIdString() <<" saved in "
	   << baseFTLFile.getValue() <<"\n";

  // Convert the Nastran OP2-files from external reduction to fedem matrix files
  if (externalSource.getValue())
    this->convertOP2files(this->getAbsFilePath(true));

  return true;
}


/*!
  Writes the FE data of this part to the specified \a ftlFile, with some
  additional meta information on from where, who, and when the file was written.
  Optionally, the part check-sum is also written, which then is used to detect
  whether the file has been edited manually when reading it.
*/

bool FmPart::exportPart(const std::string& ftlFile, bool extNodeInfo,
                        bool withCheckSum, bool noMetaData) const
{
  if (!myFEData) return false;

  FFlFedemWriter fedemwriter(myFEData);
  if (noMetaData)
    return fedemwriter.write(ftlFile,extNodeInfo,withCheckSum);

  FFaAppInfo current;
  return fedemwriter.write(ftlFile,extNodeInfo,withCheckSum,{
      "Fedem version: " + current.version,
      "Original FE data: " + originalFEFile.getValue(),
      "This file: " + ftlFile,
      "Model file: " + FmDB::getMechanismObject()->getModelFileName(),
      "Written by: " + current.user + ", " + current.date});
}


/*!
  Writes the FE data of this part to the specified \a vtfFile.
*/

bool FmPart::writeToVTF(VTFAFile& vtfFile, IntVec* outputOrder, IntVec* fstOrdNodes)
{
  // If FE data currently is disabled - read the part data file (if any)
  if (!myFEData && !baseFTLFile.getValue().empty())
  {
    myFEData = new FFlLinkHandler();
    if (FFlReaders::instance()->read(this->getBaseFTLFile(),myFEData) <= 0)
      this->setLinkHandler(NULL,false);
  }

  bool createdSpider = false;
  if (!myFEData && useGenericProperties.getValue())
  {
    // Create a dummy FE part consisting of one RGD element only
    // The nodal coordinates must be local to the part CS
    FaVec3 X0 = this->getPositionCG(false).translation();
    double tol = FmDB::getPositionTolerance()*0.1;
    myFEData = new FFlLinkHandler();
    int ID = 1;
    FFlRGD* spider = new FFlRGD(ID);
    FFlNode* rNode = new FFlNode(ID,X0);
    spider->setMasterNode(rNode);
    myFEData->addNode(rNode);
    std::vector<FmTriad*> triads;
    this->getTriads(triads);
    for (FmTriad* triad : triads)
      if (!triad->getLocalTranslation(this).equals(X0,tol))
      {
	rNode = new FFlNode(++ID,triad->getLocalTranslation(this));
	rNode->pushDOFs(1);
	spider->addSlaveNode(rNode);
	myFEData->addNode(rNode);
      }

    myFEData->addElement(spider);
    createdSpider = true;
  }

  // If still no FE data we cannot write VTF file
  if (!myFEData) return false;

  bool success = true;
  if (myFEData->getFiniteElement(1)) // ignore parts with no finite elements
  {
    // Append FE data of this part to the VTF file
    FFaMsg::setSubTask(this->getBaseFTLName());
    FFlVTFWriter vtf(myFEData);
    success = vtf.write(vtfFile,this->getUserDescription(),-this->getBaseID(),
                        outputOrder,fstOrdNodes);
    FFaMsg::setSubTask("");
  }

  // Erase the FE data again if currently disabled or a dummy spider was used
  if (ramUsageLevel.getValue() == NOTHING || createdSpider)
    this->setLinkHandler(NULL,false);

  return success;
}


/*!
  Extracts the recorded file check-sum from the meta data section.
*/

static unsigned int extractCheckSum (const char* fName)
{
  char cline[BUFSIZ], *csend;
  std::ifstream ftl(fName,std::ios::in);
  for (int l = 0; l < 10 && ftl.getline(cline,BUFSIZ,'\n'); l++)
    if (l == 0 && std::string(cline,10) != "FTLVERSION")
      break; // Not an FTL-file
    else if (std::string(cline,16) == "# File checksum:")
      return strtoul(cline+16,&csend,10);

  return 0;
}


/*!
  Sets the \a baseFTLFile name of this part based on the \a originalFEFile, such
  that the name is unique for all parts (in case this FE model is used more than
  once). The name is set to <bName>.ftl where <bName> equals \a originalFEFile
  without the directory part (if any) and file extension (if any).
  Note that all non-alphanumeric characters also are replaced by "_"'s.
*/

const std::string& FmPart::setValidBaseFTLFile(unsigned int myCS)
{
  if (baseFTLFile.getValue().empty())
  {
    if (originalFEFile.getValue().empty())
      return baseFTLFile.getValue(); // probably a generic part without FE data

    std::string name = FFaFilePath::getBaseName(originalFEFile.getValue(),true);
    if (name.empty())
      return baseFTLFile.getValue(); // invalid file name

    baseFTLFile.setValue(FFaFilePath::distillName(name) + ".ftl");
  }

  if (myFEData)
  {
    std::vector<FmPart*> allParts;
    FmDB::getAllParts(allParts);

    // Recursive lambda function checking that this->baseFTLFile is a valid and
    // unique file name, i.e., not already in use by other parts in the model.
    // If the name conflicts with the baseFTLFile of some of the other parts,
    // <bName>_ftl<###>.ftl is tried instead, where <bName> is the base name of
    // the original FE data file without the extension, and <###> is an integer
    // that is incremented from 1 until no name conflict remains.
    // If two parts have the same check-sum, they are regarded as identical
    // and can then also share the same baseFTLFile.
    std::function<int(std::string&)> checkName = [this,allParts,&myCS,&checkName]
      (std::string& ftlName) -> int
    {
      for (FmPart* part : allParts)
        if (part != this && part->isFEPart(true) &&
            part->baseFTLFile.getValue() == ftlName)
        {
          // We have common names and two existing FE data handlers.
          // Now check if the parts match (except from external nodes).
          if (!myCS) // Avoid calculating check-sum more than once.
            myCS = myFEData->calculateChecksum();

          if (myCS == part->savedCS.getValue() && part->isSaved(false))
          {
            // This part matches a part that already has been saved.
            // Update the savedCS value to avoid saving more than once.
            savedCS.setValue(myCS);
            break;
          }

          if (myCS != part->myFEData->calculateChecksum())
          {
            // The check-sums are different - need to create a new file name.
            // Check if we have the string "_ftl###" at the end of the basename
            // and increment the counter if we do.

            int tagID = 1;
            size_t underscorePos = ftlName.rfind("_ftl");
            if (underscorePos != std::string::npos)
            {
              tagID = 1 + atoi(ftlName.substr(underscorePos+4).c_str());
              ftlName.erase(underscorePos);
            }
            else
              ftlName = FFaFilePath::getBaseName(ftlName);

            ftlName += FFaNumStr("_ftl%d.ftl",tagID);
            return 1 + checkName(ftlName); // Start over with the new file name
          }
        }

#ifdef FM_DEBUG
      std::cout <<"Valid base name: "<< ftlName << std::endl;
#endif
      return 0;
    }; // End of lambda function

    // Check that baseFTLFile is not already in use by other parts in the model
    int nTrial = checkName(baseFTLFile.getValue());
    if (nTrial > 0)
      ListUI <<"  -> Conflicting file name for "<< this->getIdString()
             <<". New FTL base name: "<< baseFTLFile.getValue()
             <<" ("<< nTrial <<")\n";
  }

  // If we are using a repository and the check-sum has changed,
  // we need to create a new file name if the file already exists.

  if (this->usesRepository() == INTERNAL_REP ||
      myCS == 0 || myCS == savedCS.getValue())
    return baseFTLFile.getValue();

  std::string fName = this->getBaseFTLFile();
  if (!FmFileSys::isFile(fName))
    return baseFTLFile.getValue();

  std::string bName = FFaFilePath::getBaseName(fName);
  size_t underscorePos = bName.rfind("_ftl");
  size_t minusPos = bName.find_first_of('-',underscorePos);
  if (minusPos != std::string::npos)
    bName.erase(minusPos); // Erase the existing file numbering
  else if (underscorePos == std::string::npos)
    bName.append("_ftl1"); // No numbering yet, add one

  // Increment until the file name no longer exists
  // or until we find a file with matching check-sum
  unsigned int fileCS = 0;
  for (int i = 1; fileCS != myCS; i++)
    if (FmFileSys::isFile(fName = bName + FFaNumStr("-%d.ftl",i)))
      fileCS = extractCheckSum(fName.c_str());
    else
      break;

  baseFTLFile.setValue(FFaFilePath::getFileName(fName));

  ListUI <<"  -> Conflicting file name for "<< this->getIdString();
  if (fileCS == myCS)
  {
    ListUI <<". Using matching file: ";
    savedCS.setValue(myCS);
  }
  else
    ListUI <<". New FTL base name: ";

  ListUI << baseFTLFile.getValue() <<"\n";
  return baseFTLFile.getValue();
}


/*!
  Clears all existing proxy element groups in the part,
  either they are erased completely or only the pointer to the
  real FE element group.
*/

void FmPart::clearElemGroupProxies(bool doErase)
{
  std::vector<FmElementGroupProxy*> groups;
  this->getReferringObjs(groups,"myOwner");

  for (FmElementGroupProxy* group : groups)
    if (doErase)
      group->erase();
    else
      group->setRealObject(NULL);
}


/*!
  Creates proxy element groups based on the groups in the part object.
*/

void FmPart::createElemGroupProxies()
{
  // Not needed for generic parts and suppressed parts
  if (!this->isFEPart(true))
    return;

  myFEData->updateGroupVisibilityStatus();

  // Lambda function creating a new proxy for an element group
  auto&& newGProxy = [this](FFlNamedPartBase* group)
  {
    FmElementGroupProxy* newGrp = new FmElementGroupProxy();
    newGrp->setParentAssembly(this->getParentAssembly());
    newGrp->setRealObject(group);
    newGrp->connect(this);
    return newGrp->getID();
  };

  // Traverse the element groups in reverse order,
  // to avoid clashing ids when giving new IDs to groups with IDs == 0

  std::vector<FFlGroup*> groups;
  for (GroupCIter git = myFEData->groupsBegin(); git != myFEData->groupsEnd(); ++git)
    groups.push_back(git->second);
  if (groups.size() > 1)
    std::reverse(groups.begin(),groups.end());

  // Create group proxies

  int newGid = 0;
  for (FFlGroup* group : groups)
    if ((newGid = newGProxy(group)) > 0 && group->getID() == 0)
    {
      // Connecting will result in a new ID for those groups with ID == 0
      // Print warning and update FFlGroup::ID
      ListUI <<"  -> Warning for "<< this->getIdString()
             <<", file "<< baseFTLFile.getValue()
             <<".\n     Found group with ID = 0. This group is assigned the ID "
             << newGid << ".\n";
      group->setID(newGid);
    }

  // Also create from some attributes:
  AttributeTypeCIter ait;
  for (ait = myFEData->attributeTypesBegin(); ait != myFEData->attributeTypesEnd(); ++ait)
    if (ait->first == "PMAT" || ait->first == "PTHICK")
      // Loop over all attributes of this type:
      for (const AttributeMap::value_type& attr : ait->second)
        if ((attr.second->getVisibilityStatus() & FFlNamedPartBase::FFL_USED_MASK) == FFlNamedPartBase::FFL_USED)
          newGProxy(attr.second);
}


/*!
  Updates the proxy element groups for this part to be in sync
  with the FE part data object.
*/

void FmPart::updateElemGroupProxies()
{
  // Not needed for generic parts and suppressed parts
  if (!this->isFEPart(true))
    return;

  std::vector<FmElementGroupProxy*> oldGroups;
  this->getReferringObjs(oldGroups,"myOwner");
  if (oldGroups.empty())
  {
    this->createElemGroupProxies();
    return;
  }

  myFEData->updateGroupVisibilityStatus();

  // Lambda function searching for a specific element proxy group object,
  // creating a new proxy group if not found
  auto&& getGProxy = [this,&oldGroups](int gId, const std::string& name)
  {
    size_t idx = 0;
    for (FmElementGroupProxy* grp : oldGroups)
      if (grp->getID() == gId && grp->getTypeName() == name)
      {
        // Found an existing proxy group with this ID
        oldGroups.erase(oldGroups.begin()+idx);
        return grp;
      }
      else
        ++idx;

    // Create a new proxy element group
    FmElementGroupProxy* newGrp = new FmElementGroupProxy();
    newGrp->setParentAssembly(this->getParentAssembly());
    newGrp->setID(gId);
    newGrp->connect(this);
    return newGrp;
  };

  for (GroupCIter git = myFEData->groupsBegin(); git != myFEData->groupsEnd(); ++git)
    if (git->second->getID() == 0)
      // We cannot use this group, because we don't know which ID
      // it actually got if this file was imported earlier
      ListUI <<"  -> Warning for "<< this->getIdString()
             <<", file "<< baseFTLFile.getValue()
             <<".\n     Found group with ID = 0 (ignored).\n";
    else
      getGProxy(git->second->getID(),"Group")->setRealObject(git->second);

  // Now check the implicit groups:

  for (AttributeTypeCIter ait = myFEData->attributeTypesBegin(); ait != myFEData->attributeTypesEnd(); ++ait)
    if (ait->first == "PMAT" || ait->first == "PTHICK")
      // Loop over all attributes of this type:
      for (const AttributeMap::value_type& attr : ait->second)
        if ((attr.second->getVisibilityStatus() & FFlNamedPartBase::FFL_USED_MASK) == FFlNamedPartBase::FFL_USED)
          getGProxy(attr.second->getID(),ait->first)->setRealObject(attr.second);

  // Erase all old groups which haven't been updated
  for (FmElementGroupProxy* group : oldGroups)
    group->erase();
}


/*!
  Updates the load case data from the FE part data object.
*/

void FmPart::updateLoadCases()
{
  // Not needed for generic parts and suppressed parts
  if (!this->isFEPart(true))
    return;

  std::set<int> lcset;
  myFEData->getLoadCases(lcset);

  IntVec    newCases;
  DoubleVec newFactors;
  DoubleVec newLDelays;
  std::vector<FmEngine*> newEngines;

  const IntVec& oldLC = myLoadCases.getValue();
  for (int lc : lcset)
  {
    newCases.push_back(lc);

    // If any of the new load cases also existed before,
    // keep the associated load factors, delays and engines
    IntVec::const_iterator it = std::find(oldLC.begin(),oldLC.end(),lc);
    if (it != oldLC.end())
    {
      size_t idx = it - oldLC.begin();
      newFactors.push_back(myLoadFactors.getValue()[idx]);
      newLDelays.push_back(myLoadDelays.getValue()[idx]);
      newEngines.push_back(myLoadEngines.getPtr(idx));
    }
    else
    {
      newFactors.push_back(0.0);
      newLDelays.push_back(0.0);
      newEngines.push_back(NULL);
    }
  }

  myLoadCases.setValue(newCases);
  myLoadFactors.setValue(newFactors);
  myLoadDelays.setValue(newLDelays);
  myLoadEngines.setPtrs(newEngines);
}


/*!
  Check if this part has any distributed loads.
*/

bool FmPart::hasLoads() const
{
  if (myLoadCases.getValue().empty())
    return false;

  size_t iLoad = 0;
  for (double loadFactor : myLoadFactors.getValue())
    if (loadFactor != 0.0 || myLoadEngines.getPtr(iLoad++))
      return true;

  return false;
}


/*!
  Writes distributed load data to the solver input file.
*/

void FmPart::printSolverLoads(FILE* fd) const
{
  FmEngine* loadEngine = NULL;
  const IntVec& loadCases  = myLoadCases.getValue();
  const DoubleVec& factors = myLoadFactors.getValue();
  const DoubleVec& delays  = myLoadDelays.getValue();
  for (size_t i = 0; i < loadCases.size(); i++)
    if ((loadEngine = myLoadEngines.getPtr(i)) || factors[i] != 0.0)
    {
      double f0 = loadEngine ? 0.0 : factors[i]; // Constant load part
      double f1 = loadEngine ? 1.0 : 0.0;        // Scalable load part
      int loadEngineId = loadEngine ? loadEngine->getBaseID() : 0;
      fprintf(fd,"&SUPEL_LOAD\n");
      this->printID(fd);
      fprintf(fd,"  loadCase = %u\n", (unsigned int)i+1);
      fprintf(fd,"  supElId = %d\n", this->getBaseID());
      fprintf(fd,"  f0 = %17.9e, f1 = %17.9e", f0,f1);
      fprintf(fd,", loadEngineId = %d\n", loadEngineId);
      fprintf(fd,"  delay = %17.9e", delays[i]);
      fprintf(fd,"/\n\n");
    }
}


/*!
  This method is supposed to find all triads on this part that are not valid
  for solving, because they are wrongly positioned or can not be associated with
  a valid FE node. It is used to keep the model as consistent as possible.

  If it is a generic or suppressed part:
    No checking, all triads are valid.
  If it is a FE part:
    If no FE data is loaded:
      Checks that all triads have a FE node assigned.
      Report those triads that don't.
    If FE data is loaded:
      Checks which triads are validly connected to the underlying FE mesh.
      Updates the FE node number on each triad. Set to -1 if no FE node found.
*/

bool FmPart::updateTriadTopologyRefs(bool checkUnloaded, char useOutput)
{
  if (!this->isFEPart(false))
    return true;

  std::vector<FmTriad*> triads, badTriads;
  this->getTriads(triads);

  if (myFEData)
  {
    // Initialize node status
    std::set<int> oldExt, newExt;
    NodesCIter it;
    for (it = myFEData->nodesBegin(); it != myFEData->nodesEnd(); ++it)
      if ((*it)->isExternal())
      {
        (*it)->setExternal(false);
        oldExt.insert((*it)->getID());
      }

    // Syncronize the FE node references
    int nodeNo = 0;
    for (FmTriad* triad : triads)
      if ((nodeNo = triad->syncOnFEmodel()) < 0)
        badTriads.push_back(triad);
      else
        newExt.insert(nodeNo);

    // Check if any nodes have changed their status
    if (newExt.size() != oldExt.size())
      needsCSupdate.setValue(true);
    else if (newExt != oldExt)
      needsCSupdate.setValue(true);
  }
  else if (checkUnloaded)
    // We have an unloaded FE model, use the cached node values as a hint
    for (FmTriad* triad : triads)
      if (triad->FENodeNo.getValue() == -1)
      {
        triad->onChanged();
        badTriads.push_back(triad);
      }

  if (badTriads.empty())
    return true;

  std::string msg1("The following Triads are not positioned on a valid FE node:\n");
  for (FmTriad* triad : badTriads)
    msg1 += FFaNumStr("     [%d] ",triad->getID())
      + triad->getUserDescription() + "\n";

  std::string msg2("This must be corrected before the Dynamics Solver can be run.");
  if (useOutput % 2) FFaMsg::list("  -> Warning : "+ msg1 +"     "+ msg2 +"\n");
  if (useOutput / 2) FFaMsg::dialog(msg1 + msg2);

  return false;
}


/*!
  Checking:
  1. At least one triad on each part.
  2. All triads must have a proper FE-node.
*/

int FmPart::checkParts()
{
  std::vector<FmPart*> allParts;
  FmDB::getAllParts(allParts);
  FmSeaState* sea = FmDB::getSeaStateObject(false);
  int errCount = 0, nIgnoreCS = 0;

  FmPart* icsPart = NULL;
  for (FmPart* activePart : allParts)
  {
    // Skip all checks for suppressed parts
    if (activePart->suppressInSolver.getValue())
      continue;

    // Check for number of triads on this part (must be at least one)
    if (activePart->getNumberOfTriads() < 1)
    {
      errCount++;
      ListUI <<"  -> Error: "<< activePart->getIdString()
	     <<" has no Triads attached to it.\n";
      continue;
    }

    // Ensure the internal CAD data file is up-to-date
    // if geometry-based buoyancy calculation is to be performed.
    // Also create a default sea environment if no sea state object yet
    if (activePart->hasBuoyancy.getValue())
    {
      if (!activePart->saveCadData())
	errCount++;
      else if (!sea)
	sea = FmDB::getSeaStateObject(true);
    }

    // The remaining checks are relevant for FE parts only
    if (activePart->useGenericProperties.getValue())
      continue;

    // Check that all triads have an FE-node associated with it
    std::vector<FmTriad*> triads;
    activePart->getTriads(triads);
    for (FmTriad* triad : triads)
      if (triad->FENodeNo.getValue() < 1)
      {
	errCount++;
	ListUI <<"  -> Error: "<< triad->getIdString()
	       <<" on "<< activePart->getIdString()
	       <<" is not on a valid FE node.\n";
      }

    if (activePart->myFEData)
    {
      // Check that the number of component modes is valid
      int nIntDofs = activePart->myFEData->getDofCount(false);
      if (activePart->nGenModes.getValue() > nIntDofs)
      {
	ListUI <<"  -> Warning: The specified number of component modes for "
	       << activePart->getIdString(true) <<" is too high.\n"
	       <<"              It is therefore reset to "<< nIntDofs <<".\n";
	activePart->nGenModes.setValue(nIntDofs);
	activePart->onChanged();
      }
    }

    if (activePart->overrideChecksum.getValue())
    {
      if (++nIgnoreCS == 1) icsPart = activePart;
      ListUI <<"  -> Warning: "<< activePart->getIdString(true)
	     <<" has the 'Ignore check-sum test' toggled ON\n";
    }
  }

  if (nIgnoreCS)
  {
    std::string msg;
    if (nIgnoreCS > 1)
      msg = FFaNumStr("This model has %d Parts with",nIgnoreCS);
    else
      msg = icsPart->getIdString(true) + " in this model has";
    msg += " the 'Ignore check-sum test' toggled ON.\n"
      "This may yield incorrect results or other problems "
      "in the Dynamics Solver, unless\nyou are 100% sure ";
    if (nIgnoreCS > 1)
      msg += "the reduced FE Parts in question are ";
    else
      msg += "this reduced FE Part is ";
    msg += "up to date with the current model.";
    FFaMsg::dialog(msg,FFaMsg::WARNING);
  }

  return errCount;
}


void FmPart::clearSupelFiles(bool includeFTL)
{
  // Clear all file paths derived from the imported FE data file name
  // and that contains reduced superelement data
  BMatFile.setValue("");
  EMatFile.setValue("");
  GMatFile.setValue("");
  MMatFile.setValue("");
  SMatFile.setValue("");
  LMatFile.setValue("");
  DMatFile.setValue("");
  FMatFile.setValue("");
  SAMdataFile.setValue("");
  reducedFTLFile.setValue("");
  if (includeFTL)
    baseFTLFile.setValue("");
}


void FmPart::readyForUpdate(bool useExistingFmx)
{
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->removeDisplayData();
#endif
  this->setLinkHandler(NULL);

  // Check if reduced matrix files exist, try to use those who does
  std::string partPath = this->getAbsFilePath();
  if (useExistingFmx && FmFileSys::isDirectory(partPath))
  {
    // Lambda function checking for existing fmx-file names
    auto&& checkFile = [&partPath](std::string& fName)
    {
      if (!fName.empty())
        if (!FmFileSys::isFile(FFaFilePath::appendFileNameToPath(partPath,fName)))
          fName.clear();
    };

    checkFile(baseFTLFile.getValue());
    FFaFilePath::appendToPath(partPath,myRSD.getValue().getCurrentTaskDirName());
    checkFile(reducedFTLFile.getValue());
    checkFile(SAMdataFile.getValue());
    checkFile(BMatFile.getValue());
    checkFile(EMatFile.getValue());
    checkFile(GMatFile.getValue());
    checkFile(MMatFile.getValue());
    checkFile(SMatFile.getValue());
    checkFile(LMatFile.getValue());
    checkFile(DMatFile.getValue());
    checkFile(FMatFile.getValue());
  }
  else
  {
    // Clear all file paths that are derived from the imported FE data file
    this->clearSupelFiles();
    myRSD.getValue().clear();
  }

  externalSource.setValue(false);
  lockLevel.setValue(FM_ALLOW_MODIFICATIONS);

#ifdef FT_USE_CONNECTORS
  // Clear connector elements and nodes for all triads attached to the part,
  // since they have to be regenerated anyway when loading a new FE model
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  for (FmTriad* triad : triads)
    triad->itsConnectorItems.getValue().clear();
#endif
}


bool FmPart::syncRSD(bool askForMissingFiles)
{
  // Set absolute path to the current FE part repository in the RSD-object
  FmResultStatusData& rsd = myRSD.getValue();
  rsd.setPath(this->getAbsFilePath());

  if (baseFTLFile.getValue().empty())
  {
#ifdef FM_DEBUG
    std::cout <<"FmPart::syncRSD(): baseFTLFile is not set for "
              << this->getIdString(true) <<" - RSD syncronization skipped."
              << std::endl;
#endif
    return false;
  }

  if (!askForMissingFiles && rsd.isEmpty())
    rsd.setTaskName(this->getBaseFTLName());

  if (askForMissingFiles)
  {
    FmResultStatusData diskRSD;
    if (rsd.isNamed())
    {
      diskRSD.setPath(rsd.getPath());
      diskRSD.setTaskName(rsd.getTaskName());
      diskRSD.setTaskVer(rsd.getTaskVer());
    }
    else
    {
      diskRSD.setPath(this->getAbsFilePath());
      diskRSD.setTaskName(this->getBaseFTLName());
    }

    // Find set of files currently listed in the RSD-object
    std::set<std::string> oldFiles, newFiles;
    rsd.getAllFileNames(oldFiles);

    // Find absolute path to all files in the FE part repository on disk
    std::string absPath = diskRSD.getCurrentTaskDirName(true);
    diskRSD.syncFromRDB(absPath,diskRSD.getTaskName(),diskRSD.getTaskVer());
    diskRSD.getAllFileNames(newFiles);

    // Find list of files not already listed in the RSD-object
    Strings missingInRSD;
    std::set_difference(newFiles.begin(), newFiles.end(),
                        oldFiles.begin(), oldFiles.end(),
                        std::back_inserter(missingInRSD));
    if (missingInRSD.empty())
      return false; // The RSD-object is up-to-date

    ListUI <<"\nWARNING: The following reduced files found on disk for "
           << this->getIdString(true) <<" are not listed in the model file:";
    std::string msg = "These additional files were found"
      " in the FE part repository:\n\n";
    for (const std::string& file : missingInRSD)
    {
      ListUI <<"\n\t"<< FFaFilePath::getRelativeFilename(absPath,file);
      msg += FFaFilePath::getRelativeFilename(rsd.getPath(),file) + "\n";
    }
    ListUI <<"\n";
    msg += "\nDo you want to include these files in your model?";
    if (!FFaMsg::dialog(msg,FFaMsg::YES_NO))
      return false; // Ignore the extra files, don't touch the model

    // Include the missing files
    rsd.addFiles(missingInRSD);
  }
  else // Update the RSD-object with content from disk
    rsd.syncFromRDB(rsd.getCurrentTaskDirName(true),
                    rsd.getTaskName(),rsd.getTaskVer());

  // Get list of reduced matrix files
  std::set<std::string> fmxFiles;
  rsd.getAllFileNames(fmxFiles,"fmx",false);
  rsd.getAllFileNames(fmxFiles,"fsm",false);

  // Lambda function syncronizing an explicit file name field
  auto&& syncField = [&fmxFiles](FFaField<std::string>& field,
                                 const std::string& ext) -> bool
  {
    for (const std::string& fname : fmxFiles)
      if (fname.find(ext) == fname.size()-ext.size())
        return field.setValue(fname);

    return false;
  };

  // Update the matrix file fields based on current disk content
  syncField(BMatFile,"_B.fmx");
  syncField(EMatFile,"_E.fmx");
  syncField(DMatFile,"_D.fmx");
  syncField(FMatFile,"_F.fmx");
  syncField(GMatFile,"_G.fmx");
  syncField(LMatFile,"_L.fmx");
  syncField(MMatFile,"_M.fmx");
  syncField(SMatFile,"_S.fmx");
  syncField(SAMdataFile,"_SAM.fsm");

  return true;
}


bool FmPart::isTranslatable() const
{
  std::vector<FmJointBase*> joints;
  this->getJoints(joints);

  std::vector<FmTriad*> triads;
  triads.reserve(2);

  // Check if the (ball, revolute and rigid) joints attached
  // to this part also are attached to (at least) one other part.
  // In that case, this part is not translatable.
  for (FmJointBase* joint : joints)
    if (joint->isOfType(FmBallJoint::getClassTypeID()) ||
	joint->isOfType(FmRevJoint::getClassTypeID()) ||
	joint->isOfType(FmRigidJoint::getClassTypeID()))
    {
      joint->getMasterTriads(triads);
      triads.push_back(joint->getSlaveTriad());
      for (FmTriad* triad : triads)
        if (triad && triad->isAttached(this,true))
          return false;
    }

  return true;
}


void FmPart::setCGPosRef(FmIsPositionedBase* refObj)
{
  if (myCGPosRef == refObj) return;

  FaMat34 newRefCS;
  FaMat34 oldRefCS;

  if (refObj)     newRefCS = refObj->getGlobalCS();
  if (myCGPosRef) oldRefCS = myCGPosRef->getGlobalCS();

  myCG.getValue().changePosRefCS(newRefCS,oldRefCS);

  myCGPosRef = refObj;
}


void FmPart::setCGRotRef(FmIsPositionedBase* refObj)
{
  if (myCGRotRef == refObj) return;

  FaMat34 newRefCS;
  FaMat34 oldRefCS;

  if (refObj)     newRefCS = refObj->getGlobalCS();
  if (myCGRotRef) oldRefCS = myCGRotRef->getGlobalCS();

  myCG.getValue().changeRotRefCS(newRefCS,oldRefCS);

  myCGRotRef = refObj;
}


/*!
  Returns the Center of Gravity position in either global or part coordinates.
*/

FaMat34 FmPart::getPositionCG(bool globalCS) const
{
  FaMat34 cgPosRefCS; // CS for position reference of CoG
  FaMat34 cgRotRefCS; // CS rot rotation reference of CoG

  const FmIsPositionedBase* cgPosRef = this->getCGPosRef();
  const FmIsPositionedBase* cgRotRef = this->getCGRotRef();
  if (cgPosRef) cgPosRefCS = cgPosRef->getGlobalCS();
  if (cgRotRef) cgRotRefCS = cgRotRef->getGlobalCS();

  FFa3DLocation cgLoc = myCG.getValue();
  if (cgPosRef != this) cgLoc.changePosRefCS(this->getGlobalCS(),cgPosRefCS);
  if (cgRotRef != this) cgLoc.changeRotRefCS(this->getGlobalCS(),cgRotRefCS);

  if (globalCS)
  {
    FaMat34 identity;
    cgLoc.changePosRefCS(identity,this->getGlobalCS());
    cgLoc.changeRotRefCS(identity,this->getGlobalCS());
  }

  return cgLoc.getMatrix();
}


void FmPart::setPositionCG(const FaVec3& CoG, bool edited)
{
  if (edited) isCGedited = true;
  myCG.getValue().setPos(FFa3DLocation::CART_X_Y_Z,CoG);
}


void FmPart::setLocationCG(const FaVec3& CoG, const FaVec3& Iaxes)
{
  isCGedited = true;
  myCG.setValue(FFa3DLocation(FFa3DLocation::CART_X_Y_Z,CoG,
			      FFa3DLocation::EUL_Z_Y_X,Iaxes));
}


void FmPart::setLocationCG(const FFa3DLocation& cg)
{
  isCGedited = true;
  myCG.setValue(cg);
}


void FmPart::setOrientationCG(const FaVec3& Xaxis, const FaVec3& XYplane)
{
  myCG.getValue().setRot(FFa3DLocation::DIR_EX_EXY,
                         FaMat33(Xaxis,XYplane,FaVec3()));
}


FmBase* FmPart::duplicate() const
{
  if (this->isEarthLink())
    return NULL;

  FFaMsg::list("Copying Part.\n");

  FmPart* part = new FmPart();
  part->clone(const_cast<FmPart*>(this),FmBase::SHALLOW);
  part->connect();
  part->createElemGroupProxies();
  part->setLocalCS(this->getLocalCS());
  part->setTranslation(this->getTranslation() + 0.2*this->getExtents());
  part->draw();

  return part;
}


bool FmPart::mergeGenericParts(FmPart* that)
{
  if (!this->useGenericProperties.getValue()) return false;
  if (!that->useGenericProperties.getValue()) return false;

  FmModelMemberBase::inInteractiveErase = true;

  std::vector<FmTriad*> triads;
  that->getTriads(triads);

  // Transfer all triads from that part into this part
  for (FmTriad* triad : triads)
  {
    std::vector<FmSMJointBase*> joints;
    triad->getReferringObjs(joints,"itsMasterTriad");
    triad->getReferringObjs(joints,"itsSlaveTriad");
    for (FmSMJointBase* joint : joints)
      if (joint->isAttachedToLink(this))
      {
        // This joint connects the two parts that are to be merged.
        // It will no longer have any effect and can be erased.
        joint->removeItsMasterTriad();
        joint->removeItsSlaveTriad();
        joint->eraseInternal();
      }

    triad->disconnect();
    triad->connect(this);
  }

  std::string mergedPart = that->getIdString(true);
  that->erase();
  ListUI <<" ==> "<< mergedPart <<" merged into "
         << this->getIdString(true) <<"\n";

  FmModelMemberBase::inInteractiveErase = false;
  return true;
}


double FmPart::getConnectorTolerance() const
{
  if (myFEData)
    return 0.05*myFEData->getMeanElementSize();

  double dimension = this->getExtents().length();
  return dimension > 0.0 ? dimension*0.0001 : 1.0e-6;
}


/*!
  Check if the given triad can be attached and used as a connector triad.
*/

char FmPart::isTriadConnectable(FmTriad* triad) const
{
  if (!triad) return 2; // We need a new triad

  if (triad->isAttached(this))
  {
    if (triad->FENodeNo.getValue() == -1)
      return 1; // Attached to this part but has no FE node assigned ==> can use
#ifdef FT_USE_CONNECTORS
    else if (!triad->itsConnectorItems.getValue().empty())
      return 1; // Attached to this part and has connector info ==> can redefine it
#endif

    FFaMsg::dialog("You can not use the selected Triad as attach point of the connector\n"
                   "because the Triad is already attached directly to the FE model.\n"
                   "If you really need to attach this Triad with a connector, detach it first.");
    return 0;
  }
  else if (!triad->isAttached())
    return 1; // Not attached at all, OK to use as connector triad

  return FFaMsg::dialog("The selected Triad is already attached to another Part.\n"
                        "A new Triad will be created at this position.",
                        FFaMsg::OK_CANCEL) ? 2 : 0;
}


#ifdef FT_USE_CONNECTORS
void FmPart::updateConnectorVisualization()
{
#ifdef USE_INVENTOR
  if (itsDisplayPt && myFEData)
    static_cast<FdPart*>(itsDisplayPt)->updateSpecialLines();
#endif
}


bool FmPart::createConnector(const IntVec& nodes, const FaVec3& refNodePos,
                             FmTriad* triad, int spiderType)
{
  char isUsable = this->isTriadConnectable(triad);
  if (!isUsable) return false;

  // Put the nodal points into a FFaCompoundGeometry
  FFaPointSetGeometry pointsGeom;
  FFaCompoundGeometry geometry(this->getConnectorTolerance());
  geometry.addGeometry(pointsGeom);
  int nErr = 0;
  for (int ID : nodes)
  {
    FFlNode* node = this->getNode(ID);
    if (node)
      pointsGeom.addPoint(node->getPos());
    else
    {
      ListUI <<" --> Error: Node "<< ID <<" does not exist.\n";
      ++nErr;
    }
  }
  if (nErr > 0) return false;

  // Position of the spider reference node
  FaVec3 nodePos = this->getGlobalCS().inverse()*refNodePos;

  return this->createConnector(geometry, nodePos, this->getOrientation(),
			       isUsable == 2 ? NULL : triad, spiderType);
}


/*!
  Creates a (rigid or flexible) spider element at the specified location,
  connecting the given triad, or a newly created triad if no triad specified,
  to the set of FE nodes in this part that lie on the specified geometry.
*/

bool FmPart::createConnector(const FFaCompoundGeometry& geometry,
                             const FaVec3& refNodePos, const FaMat33& refNodeCS,
                             FmTriad* triad, int spiderType)
{
  ListUI <<"===> Creating FE connection to "<< this->getIdString(true) <<"\n";

  FFlConnectorItems cItems;
  if (myFEData)
  {
    int spiderSize = myFEData->createConnector(geometry,refNodePos,spiderType,cItems);
    if (spiderSize == 0)
    {
      ListUI <<"  -> Warning : Could not find any nodes within the geometry.\n"
             <<"               The FE connection could not be made.\n";
      FFaMsg::dialog("Could not find any nodes within the specified geometry.\n"
                     "The FE connection could not be created.\n");
    }
    else if (spiderSize < 0)
      return false;
    else
      needsCSupdate.setValue(true);
  }

  bool newTriad = false;
  if (triad)
    triad->disconnect();
  else
  {
    newTriad = true;
    triad = new FmTriad(this->getGlobalCS()*refNodePos);
    triad->setOrientation(this->getGlobalCS().direction()*refNodeCS);
  }
  triad->connect(this);

  if (newTriad)
    ListUI <<"  -> Created Triad ["<< triad->getID() <<"]\n";

  triad->itsConnectorGeometry = geometry;
  triad->itsConnectorType = static_cast<FmTriad::ConnectorType>(spiderType);
  triad->itsConnectorItems = cItems;
  this->updateConnectorVisualization();
  triad->draw();

  return true;
}


/*!
  Creates a (rigid or flexible) circular/cylindric connector at the specified location.
*/

bool FmPart::createCylinderConnector(const FaVec3Vec& cylPoints, bool useArcOnly,
                                     const FaVec3* refNodePos, bool projectRefNodeToAxis,
                                     FmTriad* triad, int spiderType)
{
  if (cylPoints.size() < 3)
    return false;

  double geoTol = this->getConnectorTolerance();
  FFaCompoundGeometry geometry(geoTol);
  FFaCylinderGeometry cylinder(cylPoints,useArcOnly);
  geometry.addGeometry(cylinder);

  // If the cylinder is bigger than the tolerance, use only the surface
  if (cylinder.getRadius() > geoTol)
  {
    cylinder.setAddExclude(false);
    geometry.addGeometry(cylinder);
  }

  // Position of the spider reference node
  const FaMat34& cylCS = cylinder.getTransMatrix();
  FaVec3 localRefNodePos(cylCS.translation());
  if (refNodePos && projectRefNodeToAxis)
    localRefNodePos += cylCS[2]*(cylCS.inverse()*(*refNodePos)).z();
  else if (refNodePos)
    localRefNodePos = *refNodePos;
  else if (cylPoints.size() > 4)
    localRefNodePos += cylCS[2]*0.5*(cylinder.getZData().first + cylinder.getZData().second);

  // Create the spider element
  return this->createConnector(geometry,localRefNodePos,cylCS.direction(),triad,spiderType);
}


/*!
  Creates a (rigid or flexible) plane connector at the specified location.
*/

bool FmPart::createPlaneConnector(const FaVec3Vec& planePoints,
                                  const FaVec3* refNodePos, bool projectRefNodeToPlane,
                                  FmTriad* triad, int spiderType)
{
  if (planePoints.size() < 3)
    return false;

  FFaCompoundGeometry geometry(this->getConnectorTolerance());
  FFaPlaneGeometry plane(planePoints[0],planePoints[1],planePoints[2]);
  geometry.addGeometry(plane);

  // Add another plane with opposite normal
  FaMat34 newCS = plane.getTransMatrix();
  newCS[VY] *= -1.0;
  newCS[VZ] *= -1.0;
  plane.setTransMatrix(newCS);
  geometry.addGeometry(plane);

  // Position of the spider reference node
  const FaMat34& planeCS = plane.getTransMatrix();
  FaVec3 localRefNodePos(planeCS.translation());
  if (refNodePos && projectRefNodeToPlane)
    localRefNodePos = planeCS.projectOnXY(*refNodePos);
  else if (refNodePos)
    localRefNodePos = *refNodePos;

  // Create the spider element
  return this->createConnector(geometry,localRefNodePos,planeCS.direction(),triad,spiderType);
}


/*!
  Creates a (rigid or flexible) plane connector at the specified location.
*/

bool FmPart::createLineConnector(const FaVec3Vec& linePoints,
                                 const FaVec3* refNodePos, bool projectRefNodeToAxis,
                                 FmTriad* triad, int spiderType)
{
  if (linePoints.size() < 2)
    return false;

  FFaCompoundGeometry geometry(this->getConnectorTolerance());
  FFaLineGeometry line(linePoints[0],linePoints[1]);
  geometry.addGeometry(line);

  // Position of the spider reference node
  const FaMat34& lineCS = line.getTransMatrix();
  FaVec3 localRefNodePos(lineCS.translation());
  if (refNodePos && projectRefNodeToAxis)
    localRefNodePos += lineCS[2]*(lineCS.inverse()*(*refNodePos)).z();
  else if (refNodePos)
    localRefNodePos = *refNodePos;

  // Create the spider element
  return this->createConnector(geometry,localRefNodePos,lineCS.direction(),triad,spiderType);
}
#endif


bool FmPart::interactiveErase()
{
  std::vector<FmStrainRosette*> rosettes;
  this->getReferringObjs(rosettes,"rosetteLink");

  if (!this->FmLink::interactiveErase())
    return false;

  for (FmStrainRosette* rosette : rosettes)
    rosette->erase();

  return true;
}


bool FmPart::enforceStrainRosetteRecovery()
{
  int recover = recoveryDuringSolve.getValue();
  if (recover > 1) return false;

  return recoveryDuringSolve.setValue(recover+2);
}
