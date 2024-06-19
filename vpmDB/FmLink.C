// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
#pragma warning(disable:4068)
#endif

#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaDefinitions/FFaAppInfo.H"
#include "FFaLib/FFaOS/FFaFilePath.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdLink.H"
#endif

#include "vpmDB/FmLink.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmSMJointBase.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmGlobalViewSettings.H"

#include <algorithm>
#include <functional>
#include <fstream>
#include <cfloat>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcLINK, FmLink, FmIsPositionedBase);


FmLink::FmLink(const FaVec3& globalPos)
{
  Fmd_CONSTRUCTOR_INIT(FmLink);

  // Initialize fields

  FFA_FIELD_INIT(myMeshType, SURFACE, "MESH_TYPE");
  FFA_FIELD_INIT(myModelType, SURFACE, "MODEL_TYPE");

  FFA_FIELD_INIT(myShininess,    0.8, "SHININESS");
  FFA_FIELD_INIT(myTransparency, 0.0, "TRANSPARENCY");
  FFA_FIELD_INIT(myLineRGBColor, FmColor(1.0f,1.0f,1.0f), "LINE_COLOR");
  FFA_FIELD_INIT(myRGBColor,     FmGlobalViewSettings::getLinkDefaultColor(), "COLOR");
  FFA_FIELD_INIT(objFileGroupIndex, -1, "OBJ_FILE_GROUP_INDEX");

  FFA_FIELD_DEFAULT_INIT(visDataFile,              "VISUALIZATION_FILE");
  FFA_FIELD_DEFAULT_INIT(visDataFileUnitConverter, "ORIGINAL_VISDATA_FILE_CONVERSION");
  FFA_FIELD_DEFAULT_INIT(cadMainComponentId,       "CAD_MAIN_COMPONENT_ID");
  FFA_FIELD_DEFAULT_INIT(baseCadFileName,          "BASE_CAD_FILE");

  FFA_FIELD_INIT(alpha1, 0.0, "MASS_PROP_DAMP");
  FFA_FIELD_INIT(alpha2, 0.0, "STIF_PROP_DAMP");

  FFA_FIELD_INIT(massScale,      1.0, "MASS_SCALE");
  FFA_FIELD_INIT(stiffnessScale, 1.0, "STIFFNESS_SCALE");

  FFA_FIELD_INIT(myCSOption, MODEL_DEFAULT, "CS_POS_ALGORITHM");

  myCS.getValue()[3] = globalPos;
}


FmLink::~FmLink()
{
  this->disconnect();

#ifdef USE_INVENTOR
  if (itsDisplayPt) {
    itsDisplayPt->removeDisplayData();
    itsDisplayPt->fdErase();
  }
  itsDisplayPt = NULL;
#endif
}


/*!
  Returns the path to the directory in which the link will be saved.
  Tries to create the directory if it does not exist, if \a createDir is true.
*/

std::string FmLink::getAbsFilePath(bool createDir) const
{
  return FmDB::getMechanismObject()->getAbsModelLRDBPath(createDir);
}


/*!
  Returns the full path of the saved CAD file of this link.
*/

std::string FmLink::getBaseCadFile(bool createDir) const
{
  if (baseCadFileName.getValue().empty()) return "";

  return FFaFilePath::appendFileNameToPath(this->getAbsFilePath(createDir),
                                           baseCadFileName.getValue());
}


/*!
  Returns the full path of the geometry file of this link.
  If a CAD file is defined, return that file name.
  Otherwise, return the visualization file name, if any.
*/

std::string FmLink::getGeometryFile() const
{
  const std::string& geoFile = baseCadFileName.getValue();
  if (!geoFile.empty())
    return FFaFilePath::appendFileNameToPath(this->getAbsFilePath(),geoFile);

  std::string visFile = visDataFile.getValue();
  if (!visFile.empty())
    FFaFilePath::makeItAbsolute(visFile,FmDB::getMechanismObject()->getAbsModelFilePath());

  return visFile;
}


/*!
  Returns true if the link's CAD model currently is loaded.
*/

bool FmLink::isCADLoaded() const
{
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    if (static_cast<FdLink*>(itsDisplayPt)->getCadComponent())
      return true;
#endif
  return false;
}


/*!
  Returns true if the link uses generic part visualization.
*/

bool FmLink::isUsingGenPartVis() const
{
#ifdef USE_INVENTOR
  if (itsDisplayPt)
    return static_cast<FdLink*>(itsDisplayPt)->isUsingGenPartVis();
#endif
  return false;
}


/*!
  Updates the simplified generic part visualization.
*/

void FmLink::updateGPVisualization()
{
#ifdef USE_INVENTOR
  if (itsDisplayPt && this->isGenericPart())
    static_cast<FdLink*>(itsDisplayPt)->updateSimplifiedViz();
#endif
}


bool FmLink::interactiveErase()
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  size_t trCount = triads.size();
  for (std::vector<FmTriad*>::iterator it = triads.begin(); it != triads.end();)
    if ((*it)->isMasterTriad() || (*it)->isSlaveTriad() ||
	(*it)->getOwnerLink(1))
      it = triads.erase(it);
    else
      ++it;

  if (!triads.empty())
  {
    std::string msg = "Erase " + this->getIdString(true);
    msg += FFaNumStr(":\nDo you also want to erase the %i triad(s)"
		     " attached to this part ?",(int)triads.size());
    if (triads.size() < trCount)
      msg += "\nThe triads that are attached via joints will be retained.";

    switch (FFaMsg::dialog(msg,FFaMsg::YES_ALL_NO_ALL_CANCEL))
      {
      case 1: // yes
	for (FmTriad* triad : triads) triad->erase();
	triads.clear();
	break;

      case 2: // cancel
	return false;
      }
  }
  else if (FFaMsg::dialog("Erase " + this->getIdString(true) + " ?",
			  FFaMsg::OK_ALL_CANCEL) == 0)
    return false; // cancel

  bool status = this->erase();
  for (FmTriad* triad : triads) triad->updateDisplayDetails();
  return status;
}


bool FmLink::highlight(bool trueOrFalse)
{
  bool status = true;
  if (this->isEarthLink())
  {
    std::vector<FmRefPlane*> refPlanes;
    FmDB::getAllRefPlanes(refPlanes);
    for (FmRefPlane* plane : refPlanes)
      status &= plane->highlight(trueOrFalse);
  }
  else
    status = this->FmIsPositionedBase::highlight(trueOrFalse);

  return status;
}


bool FmLink::setLineRGBColor(const FmColor& col)
{
  if (!myLineRGBColor.setValue(col))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdApperance();
#endif
  return true;
}


bool FmLink::setMeshType(Detail enc)
{
  if (!myMeshType.setValue(enc))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdDetails();
#endif
  return true;
}


bool FmLink::setModelType(Detail enc)
{
  if (!myModelType.setValue(enc))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdDetails();
#endif
  return true;
}


std::string FmLink::getLinkIDString(bool objPrefix) const
{
  if (this->isEarthLink()) return std::string("Gnd");

  std::string LinkId;
  if (objPrefix) LinkId = std::string(this->getUITypeName()) + " ";
  LinkId += FFaNumStr("[%d] ",this->getID()) + this->getUserDescription();
  return LinkId;
}


bool FmLink::setRGBColor(float r, float g, float b)
{
  return this->setRGBColor(FmColor(r,g,b));
}


bool FmLink::setRGBColor(const FmColor& col)
{
  if (!myRGBColor.setValue(col))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdApperance();
#endif
  return true;
}


bool FmLink::setShininess(double var)
{
  if (!myShininess.setValue(var))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdApperance();
#endif
  return true;
}


bool FmLink::setTransparency(double var)
{
  if (!myTransparency.setValue(var))
    return false;

#ifdef USE_INVENTOR
  if (itsDisplayPt)
    itsDisplayPt->updateFdApperance();
#endif
  return true;
}


void FmLink::getJoints(std::vector<FmJointBase*>& joints) const
{
  joints.clear();
  std::vector<FmTriad*> triads;
  std::vector<FmJointBase*> tmpJoints;
  this->getTriads(triads);

  for (FmTriad* triad : triads)
  {
    triad->getJointBinding(tmpJoints);
    for (FmJointBase* joint : tmpJoints)
      if (std::find(joints.begin(),joints.end(),joint) == joints.end())
        joints.push_back(joint);
  }
}


bool FmLink::isEarthLink() const
{
  return this->getID() < 0 ? true : false;
}


bool FmLink::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmLink::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmLink* copyObj = static_cast<FmLink*>(obj);

  // append triads from the cloned
  std::vector<FmTriad*> triads;
  copyObj->getTriads(triads);
  for (FmTriad* triad : triads)
  {
    triad->disconnect();
    triad->connect(this);
  }

  return true;
}


bool FmLink::attach(FmBase* attachObject, bool isSilent)
{
  if (attachObject->isOfType(FmTriad::getClassTypeID()))
  {
    // Check if the triad is an independent line joint triad.
    // Attach the whole joint if that is the case.
    if (((FmTriad*)attachObject)->isMasterTriad())
    {
      bool isMM_Master  = false;
      bool attachStatus = false;
      std::vector<FmJointBase*> joints;
      ((FmTriad*)attachObject)->getJointBinding(joints);
      for (FmJointBase* joint : joints)
        if (joint->isOfType(FmMMJointBase::getClassTypeID()))
        {
          isMM_Master = true;
          if (this->attachMMJoint((FmMMJointBase*)joint))
            attachStatus = true;
        }
      if (isMM_Master) return attachStatus;
    }

    return this->attachTriad((FmTriad*)attachObject,isSilent,
                             !this->isOfType(FmBeam::getClassTypeID()));
  }

  if (attachObject->isOfType(FmLoad::getClassTypeID()))
    return this->attachTriad(((FmLoad*)attachObject)->getOwnerTriad(),isSilent,
                             !this->isOfType(FmBeam::getClassTypeID()));

  if (attachObject->isOfType(FmSMJointBase::getClassTypeID()))
    return this->attachSMJoint((FmSMJointBase*)attachObject,isSilent);

  if (attachObject->isOfType(FmMMJointBase::getClassTypeID()))
    return this->attachMMJoint((FmMMJointBase*)attachObject,isSilent);

  if (attachObject->isOfType(Fm1DMaster::getClassTypeID()))
  {
    Fm1DMaster* surf = (Fm1DMaster*)attachObject;
    std::vector<FmTriad*> triads;
    surf->getTriads(triads);

    // Check all triads in the joint
    FmTriad* existingTriad = NULL;
    for (FmTriad* triad : triads)
      if (!this->isTriadAttachable(existingTriad,triad,isSilent))
        return false;

    // All triads have been checked, attach them if all are attachable
    bool attachStatus = true;
    for (FmTriad* triad : triads)
      if (!this->attachTriad(triad,isSilent,false)) // attach, but no update
        attachStatus = false;

    // Update triad visualization etc.
    for (FmTriad* triad : triads)
      triad->updateTopologyInViewer();

    surf->updateDisplayDetails();
    return attachStatus;
  }

  return false; // an object of this type can not be attached
}


bool FmLink::attachSMJoint(FmSMJointBase* attachJt, bool isSilent)
{
  // Start the attach check with the dependent joint triad
  FmTriad* existingTriad = NULL;
  FmTriad* triadToAttach = attachJt->getSlaveTriad();
  if (triadToAttach && !triadToAttach->isAttached())
    if (this->isTriadAttachable(existingTriad,triadToAttach,isSilent))
      return this->attachTriad(triadToAttach,isSilent);

  // Then check the other joint triad if the first one
  // already is attached, or it is unattachable
  triadToAttach = attachJt->getItsMasterTriad();
  if (triadToAttach && !triadToAttach->isAttached())
  {
    if (!attachJt->isSlaveAttachedToLink(true))
      ListUI <<"       Trying the independent joint triad instead.\n";

    if (this->isTriadAttachable(existingTriad,triadToAttach,isSilent))
      return this->attachTriad(triadToAttach,isSilent);
  }

  return false;
}


bool FmLink::attachMMJoint(FmMMJointBase* attachJt, bool isSilent)
{
  bool attachStatus = true;
  FmTriad* existingTriad = NULL;

  std::vector<FmTriad*> triads;
  attachJt->getMasterTriads(triads);

  // Check all independent joint triads
  for (FmTriad* triad : triads)
    if (!this->isTriadAttachable(existingTriad,triad,isSilent))
      attachStatus = false;
    else if (existingTriad && existingTriad->getOwnerLink(1))
    {
      attachStatus = false;
      ListUI <<"Error: All independent joint triads have to be on one part only.\n";
    }

  if (!attachStatus) return false;

  // All triads have been checked, attach them if all are attachable
  for (FmTriad* triad : triads)
    if (!this->attachTriad(triad,isSilent,false)) // attach, but no update
      attachStatus = false;

  // Update triad visualization etc.
  attachJt->getSlaveTriad()->updateTopologyInViewer();
  for (FmTriad* triad : triads)
    triad->updateTopologyInViewer();

  return attachStatus;
}


/*!
  This is what has to be done in this method:
  1. check if the triad is owned.
  IF this link is a generic part or not loaded THEN
     2. check if the triad position matches an existing triad.
  ELSE
     3. check if the triad position is on a valid node.
     4. check if an existing triad is associated with this node.
  END IF
  5. if an existing triad is found and is attachable,
     clone properties from the new triad to the existing one.
*/

bool FmLink::attachTriad(FmTriad* attachTr, bool isSilent, bool doUpdate)
{
#ifdef FM_DEBUG
  std::cout <<"FmLink::attachTriad ["<< attachTr->getID()
	    <<"] "<< attachTr->getLocalTranslation() << std::endl;
#endif

  // Lambda function for checking if two triads are on a common joint
  std::function<bool(FmTriad*,FmTriad*)> onJoint = [](FmTriad* tr1, FmTriad* tr2)
  {
    FmJointBase* jnt = tr1->getJointWhereSlave();
    return jnt ? jnt->isMasterTriad(tr2) : false;
  };

  FmTriad* oldTr = NULL;
  if (!this->isTriadAttachable(oldTr,attachTr,isSilent))
    return false;

  else if (!oldTr)
  {
    // Attaching without any problems
    attachTr->disconnect();
    attachTr->connect(this);
  }

  // Check if the old triad and the triad to attach
  // are on the same joint. Then we cannot attach.
  else if (onJoint(attachTr,oldTr) || onJoint(oldTr,attachTr))
  {
    ListUI <<"Error: "<< attachTr->getIdString() <<" can not be attached to "
           << this->getIdString(true) <<"\n       because it matches "
           << oldTr->getIdString() <<" which already is on the same joint.\n";
    return false;
  }

  // Attaching when having an old triad in place
  else if (!this->attachTriad(attachTr,oldTr,isSilent))
    return false;
  else if (!this->isOfType(FmPart::getClassTypeID()))
    attachTr = oldTr; // The old existing triad is used instead
  else
    attachTr = NULL; // Don't need to update visualization for Triads on Parts

  this->onChanged();
  if (!doUpdate || !attachTr)
    return true;

  // Update the triad visualization
  std::vector<FmJointBase*> joints;
  attachTr->getJointBinding(joints);
  if (joints.empty())
    attachTr->updateTopologyInViewer();
  else for (FmJointBase* joint : joints)
  {
    std::vector<FmTriad*> triads;
    joint->getMasterTriads(triads);
    triads.push_back(joint->getSlaveTriad());
    for (FmTriad* triad : triads)
      triad->updateTopologyInViewer();
  }

#ifdef USE_INVENTOR
  if (itsDisplayPt && !FFaAppInfo::isConsole())
    itsDisplayPt->updateFdDetails();
#endif

  return true;
}


/*!
  \brief Attaching a triad when having an old triad in the same place.
*/

bool FmLink::attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool isSilent)
{
#ifdef FM_DEBUG
  isSilent = false;
#endif
  bool hadImportantDirections = oldTr->importantDirections();

  // In case the attached triad is plotted
  std::vector<FmModelMemberBase*> curves;
  attachTr->getReferringObjs(curves,"myResultObject[XAXIS]");
  attachTr->getReferringObjs(curves,"myResultObject[YAXIS]");
  attachTr->releaseReferencesToMe("myResultObject[XAXIS]",oldTr);
  attachTr->releaseReferencesToMe("myResultObject[YAXIS]",oldTr);
  for (FmModelMemberBase* curve : curves)
    curve->onChanged();

  // In case the attached triad is dependent (can be for one joint only)
  FmJointBase* joint = NULL;
  if (attachTr->hasReferringObjs(joint,"itsSlaveTriad"))
    joint->setAsSlaveTriad(oldTr);

  // In case the attached triad is an independent point joint triad
  std::vector<FmSMJointBase*> joints;
  attachTr->getReferringObjs(joints,"itsMasterTriad");
  for (FmSMJointBase* joint : joints)
    joint->setAsMasterTriad(oldTr);

  // In case the attached triad is an independent line joint triad
  std::vector<Fm1DMaster*> masters;
  attachTr->getReferringObjs(masters,"myTriads");
  for (Fm1DMaster* master : masters)
    master->releaseTriad(attachTr,oldTr);

  if (!hadImportantDirections)
  {
    oldTr->setGlobalCS(attachTr->getGlobalCS());
    if (!isSilent)
      ListUI <<"Warning: The coordinate system of "<< oldTr->getIdString()
             <<" is changed to match "<< attachTr->getIdString() <<".\n";
  }

  // In case the attached triad has axial spring/dampers or loads
  attachTr->releaseReferencesToMe("itsTriads",oldTr);
  attachTr->releaseReferencesToMe("itsOwnerTriad",oldTr);

  // Update triad visualization etc.
  oldTr->updateTopologyInViewer();
  oldTr->onChanged();

  return attachTr->erase();
}


/*!
  Checks whether \a attachTriad can be attached to this link.
  Internally used by attach, attachTriad, attachSMJoint and attachMMJoint.
  Will return true if it is, along with a possible \a existingTriad
  that is positioned at the same spot on this link.
*/

bool FmLink::isTriadAttachable(FmTriad*& existingTriad,
                               FmTriad* attachTr, bool isSilent)
{
#ifdef FM_DEBUG
  isSilent = false;
#endif
  existingTriad = NULL;

  // Check if triad is attached already
  if (attachTr->isAttached())
  {
    if (!isSilent)
      ListUI <<"Error: "<< attachTr->getIdString() <<" is already attached to "
	     << attachTr->getOwnerLink(0)->getIdString() <<".\n";
    return false;
  }

  // Search for an existing triad at the location of attachTr
  std::pair<FmTriad*,bool> existing = this->getExistingTriad(attachTr);
  if (!existing.second)
    return false;
  else if (!existing.first)
  {
    if (this->isAttachable())
      return true; // OK, triad is attachable at this point

    if (!isSilent)
      ListUI <<"Error: "<< attachTr->getIdString(true)
	     <<" is not coincident with any of the triads already\n"
	     <<"       attached to "<< this->getIdString() <<".\n";
    return false;
  }

  // Check coupling for the involved triads
  existingTriad = existing.first;
  if (existingTriad->isSlaveTriad(true) && attachTr->isSlaveTriad(true))
  {
    if (!isSilent)
      ListUI <<"Error: "<< attachTr->getIdString()
	     <<" is coincident with "<< existingTriad->getIdString()
	     <<" which also is a dependent joint triad.\n"
	     <<"       A dependent triad can not be connected to another dependent triad.\n";
    return false;
  }
  else if (existingTriad->importantDirections() && attachTr->importantDirections())
    if (!existingTriad->getGlobalCS().isCoincident(attachTr->getGlobalCS(),
						   FmDB::getPositionTolerance()))
    {
      if (!isSilent)
	ListUI <<"Error: "<< existingTriad->getIdString()
	       <<" and "<< attachTr->getIdString()
	       <<" are both orientated specifically.\n"
	       <<"       However, their coordinate systems do not match.\n";
      return false;
    }

  return true; // the existing triad may be used
}


/*!
  Returns any existing triad at the same location as \a triad on this link.
*/

std::pair<FmTriad*,bool> FmLink::getExistingTriad(const FmTriad* triad)
{
  FmTriad* existingTr = this->getTriadAtPoint(triad->getGlobalTranslation(),
					      FmDB::getPositionTolerance(),true);
  return std::make_pair(existingTr,true);
}


/*!
  Returns the closest triad to point using tolerance, or NULL if none found.
*/

FmTriad* FmLink::getTriadAtPoint(const FaVec3& point, double tolerance,
				 bool globalPoint) const
{
  FmTriad* closestTr = NULL;
  double dist, closestDist = tolerance*tolerance;

  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  for (FmTriad* triad : triads)
  {
    if (globalPoint)
      dist = (point - triad->getGlobalTranslation()).sqrLength();
    else
      dist = (point - triad->getLocalTranslation(this)).sqrLength();

    if (dist < closestDist)
    {
      closestTr = triad;
      closestDist = dist;
    }
  }

  return closestTr;
}


FaVec3 FmLink::getExtents() const
{
  FaVec3 max, min;
  if (this->getBBox(max,min))
    return max - min;
  else
    return FaVec3();
}


bool FmLink::getBBox(FaVec3& max, FaVec3& min) const
{
#ifdef USE_INVENTOR
  if (static_cast<FdLink*>(itsDisplayPt)->getGenPartBoundingBox(max,min))
    return true;
#endif

  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  if (triads.empty())
    return false;

  max = min = triads.front()->getLocalTranslation(this);
  for (FmTriad* triad : triads)
  {
    FaVec3 pos = triad->getLocalTranslation(this);
    for (int i = 0; i < 3; i++)
      if (pos[i] < min[i])
	min[i] = pos[i];
      else if (pos[i] > max[i])
	max[i] = pos[i];
  }

  return true;
}


/*!
  This method is only used when reading model files created in R7.0.4 and older.
  It converts the LINK record into either PART or BEAM.
*/

bool FmLink::readAndConnect(std::istream& is, std::ostream&)
{
  FmPart* obj = new FmPart();
  FmBeam* beam = NULL;

  // Old files without choice on Generic part stiffness type
  // should be initialized to NODE_STIFFNESS
  obj->myGenericPartStiffType = FmPart::NODE_STIFFNESS;

  // Obsolete fields
  FFaObsoleteField<bool>        useCalculatedMass;
  FFaObsoleteField<bool>        useDiagMassMx;
  FFaObsoleteField<double>      partMass;
  FFaObsoleteField<std::string> feDataFile;
  FFaObsoleteField<int>         nEigModes;

  FFA_OBSOLETE_FIELD_INIT(useCalculatedMass,false,"USE_MASS_CALCULATION"  ,obj);
  FFA_OBSOLETE_FIELD_INIT(useDiagMassMx    ,true ,"USE_LUMPED_MASS_MATRIX",obj);
  FFA_OBSOLETE_FIELD_INIT(partMass         ,0.0  ,"PART_MASS"             ,obj);
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(feDataFile     ,"FE_DATA_FILE"          ,obj);
  FFA_OBSOLETE_FIELD_INIT(nEigModes        ,0    ,"STORED_EIGENMODES"     ,obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"ELEMENT_PROPERTY") == 0)
      {
        // This is a beam element represented by a part object in older versions.
        // Convert to the new beam object, copying the properties already read.
        beam = new FmBeam();
        beam->clone(obj,FmBase::SHALLOW);
        beam->myBaseID.setValue(obj->getBaseID()); // The base ID is not copied
        beam->myCS.setValue(obj->getLocalCS()); // The coordinate system is not copied
        parentParse("PROPERTY", activeStatement, beam);
      }
      else if (beam)
        localParse(keyWord, activeStatement, beam);
      else
        localParse(keyWord, activeStatement, obj);
    }
  }

  if (beam)
  {
    obj->erase();
    beam->connect();
    return true;
  }

  // Clear invalid base name that might have been set by R3.2
  if (obj->baseFTLFile.getValue() == ".ftl") obj->baseFTLFile.setValue("");

  FFA_OBSOLETE_FIELD_REMOVE("USE_MASS_CALCULATION", obj);
  FFA_OBSOLETE_FIELD_REMOVE("USE_LUMPED_MASS_MATRIX", obj);
  FFA_OBSOLETE_FIELD_REMOVE("PART_MASS", obj);
  FFA_OBSOLETE_FIELD_REMOVE("FE_DATA_FILE", obj);
  FFA_OBSOLETE_FIELD_REMOVE("STORED_EIGENMODES", obj);

  // Update from old model file
  if (useCalculatedMass.wasOnFile())
    if (useCalculatedMass.getValue())
      if (!obj->baseFTLFile.getValue().empty())
        obj->myCalculateMass = FmPart::FROM_FEM;

  if (useDiagMassMx.wasOnFile())
  {
    if (useDiagMassMx.getValue())
    {
      ListUI <<"  -> WARNING: "<< obj->getIdString() <<" will need re-reduction due to change in the\n"
             <<"              implementation of mass matrix lumping. To use the old reduced matrix,\n"
             <<"              toggle on the \'Ignore check-sum test\' option in the part property panel.\n";
      obj->useConsistentMassMatrix = false;
    }
    else
      obj->useConsistentMassMatrix = true;
  }

  if (partMass.wasOnFile())
    obj->mass.setValue(partMass.getValue()); // update from old model file

  if (!feDataFile.getValue().empty())
    obj->originalFEFile.setValue(feDataFile.getValue());

  if (nEigModes.wasOnFile())
    obj->nGenModes.setValue(nEigModes.getValue());

  // Convert angles to degrees from old model file
  FFa3DLocation location = obj->getLocationCG();
  if (location.getRotType() == FFa3DLocation::EUL_Z_Y_X)
    if (FmDB::getModelFileVer() < FFaVersionNumber(4,1,0,3))
    {
      location[1] *= 180.0/M_PI;
      if (!location[1].isZero(0.001))
        ListUI <<"  -> WARNING: The orientation of the Principle Axes of Inertia for "<< obj->getIdString()
               <<" was stored in Radians in this model file.\n"
               <<"     The angles will be converted to Degrees when the model is saved.\n";
      obj->setLocationCG(location);
    }

  obj->connect();
  return true;
}


bool FmLink::localParse(const char* keyWord, std::istream& activeStatement, FmLink* obj)
{
  if (strcmp(keyWord,"OVERRIDE_LINK_CHECKSUM") == 0 ||
      strcmp(keyWord,"OVERRIDE_PART_CHECKSUM") == 0)
    return parentParse("OVERRIDE_CHECKSUM", activeStatement, obj);
  else if (strcmp(keyWord+4,"_CS_POS_ALGORITHM") == 0)
    return parentParse("CS_POS_ALGORITHM", activeStatement, obj);
  else if (strcmp(keyWord+4,"_CENTRIP_CORRECTION") == 0)
    return parentParse("CENTRIPETAL_CORRECTION", activeStatement, obj);
  else if (strcmp(keyWord,"ORIGINAL_FE_FILE_CONVESION") == 0)
    return parentParse("ORIGINAL_FE_FILE_CONVERSION", activeStatement, obj);
  else if (strcmp(keyWord,"GEN_PART_VISUALIZATION_FILE") == 0)
    return parentParse("VISUALIZATION_FILE", activeStatement, obj);
  else if (strcmp(keyWord,"CENTER_OF_GRAVITY") == 0)
    static_cast<FmPart*>(obj)->setLocationCG(static_cast<FmPart*>(obj)->getLocationCG());
  else if (strncmp(keyWord,"LINK_",5) == 0)
  {
#pragma GCC diagnostic push
#if __GNUC__ > 8
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
    strncpy(const_cast<char*>(keyWord),"PART",4);
#pragma GCC diagnostic pop
  }

  return parentParse(keyWord, activeStatement, obj);
}


bool FmLink::openCadData()
{
  if (baseCadFileName.getValue().empty())
    return false;

#ifdef USE_INVENTOR
  std::string filename = this->getBaseCadFile();
  std::ifstream in(filename.c_str(), std::ios::in);
  if (!in)
    ListUI <<"  -> Error: Could not open "<< filename <<" for reading.\n";
  else if (itsDisplayPt)
    return static_cast<FdLink*>(itsDisplayPt)->readCad(in);
#endif

  return false;
}


bool FmLink::saveCadData()
{
  if (baseCadFileName.getValue().empty() || !this->isCADLoaded())
    return true;

  std::string filename = this->getBaseCadFile(true);
  std::ofstream os(filename.c_str(), std::ios::out);
  if (!os)
    ListUI <<"  -> Error: Could not open "<< filename <<" for writing.\n";
#ifdef USE_INVENTOR
  else if (itsDisplayPt)
  {
    static_cast<FdLink*>(itsDisplayPt)->writeCad(os);
    return true;
  }
#endif

  return false;
}


void FmLink::updateChildrenDisplayTopology()
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  for (FmTriad* triad : triads)
    triad->updateTopologyInViewer();

  this->FmIsPositionedBase::updateChildrenDisplayTopology();
}


int FmLink::getStructDmpEngineId() const
{
  // Beta feature: Time-dependent structural damping.
  // Check description field for this object first, then its parent assemblies
  int baseId = 0;
  for (const FmBase* p = this; baseId <= 0 && p; p = p->getParentAssembly())
    baseId = FFaString(p->getUserDescription()).getIntAfter("#StructDmpEngine");

  if (baseId > 0)
    FmEngine::betaFeatureEngines.insert(baseId);

  return baseId;
}


bool FmLink::getRefPoints(FmTriad*& node1, FmTriad*& node2, FmTriad*& node3,
                          FaVec3& offset1, FaVec3& offset2, FaVec3& offset3) const
{
  // Superelement reference points
  node1 = node2 = node3 = NULL;

  // Default: Zero offset for all three points
  offset1 = offset2 = offset3 = FaVec3();

  // Beta feature: User-provided reference points
  FFaString descr = this->getUserDescription();
  if (descr.hasSubString("#RefTriads"))
  {
    int baseId[3];
    descr.getIntsAfter("#RefTriads",3,baseId);
    node1 = this->findTriad(baseId[0]);
    node2 = this->findTriad(baseId[1]);
    node3 = this->findTriad(baseId[2]);
    if (node1 && node2 && node3)
      return true;

    ListUI <<"Error: "<< this->getIdString()
           <<" does not have any triad(s) with baseID";
    if (!node1) ListUI <<" "<< baseId[0];
    if (!node2) ListUI <<" "<< baseId[1];
    if (!node3) ListUI <<" "<< baseId[2];
    ListUI <<"\n       Using automatically selected reference points instead.\n";
  }

  FmAnalysis* analysis = FmDB::getActiveAnalysis();
  bool useScaledOffset = false;
  int defAlg = 0;
  switch (myCSOption.getValue())
    {
    case MAX_TRI_LINK_SCALE_OFFSET:
      useScaledOffset = true;
      break;
    case MODEL_DEFAULT: // 1+ to match CoordSysOption enum
      defAlg = 1 + analysis->defaultShadowPosAlg.getValue();
      if (defAlg > 3) defAlg -= 2; // To remove beam-specific options
      if (defAlg == MAX_TRI_LINK_SCALE_OFFSET) useScaledOffset = true;
      break;
    default:
      break;
    }

  std::vector<FmTriad*> allTriads;
  this->getTriads(allTriads);
  if (allTriads.empty())
    return false; // no triads attached

  double minLen = useScaledOffset ? FmDB::getPositionTolerance() : 1.0e-3;

  // Get mean (centroid) of all triad positions
  FaVec3 centroid;
  for (FmTriad* triad : allTriads)
    centroid += triad->getLocalTranslation(this);
  centroid /= (double)allTriads.size();

  // Initialize all reference points to the first triad
  node1 = node2 = node3 = allTriads.front();
  allTriads.erase(allTriads.begin());

  // Find the triad furthest away from centroid, use as point 1
  FaVec3 relPos  = node1->getLocalTranslation(this) - centroid;
  double oldDist = relPos.length();
  for (FmTriad* triad : allTriads)
  {
    relPos = triad->getLocalTranslation(this) - centroid;
    if (relPos.length() > oldDist)
    {
      node1   = triad;
      oldDist = relPos.length();
    }
  }

  FaVec3 point1(node1->getLocalTranslation(this));

  // Find the triad furthest away from the selected point 1, use as point 2
  relPos  = node2->getLocalTranslation(this) - point1;
  oldDist = relPos.length();
  for (FmTriad* triad : allTriads)
  {
    relPos = triad->getLocalTranslation(this) - point1;
    if (relPos.length() > oldDist)
    {
      node2   = triad;
      oldDist = relPos.length();
    }
  }

  FaVec3 vec12(node2->getLocalTranslation(this) - point1);
  if (oldDist < minLen)
  {
    // All triads are coincident.
    // Obtain point 2 by unit offset along the global X-axis.
    offset2[0] = 1.0;
    vec12[0]  += 1.0;
  }

  if (useScaledOffset) // use offset equal to distance between point 1 and 2
  {
    if (allTriads.size() < 2)
      minLen = DBL_MAX;
    else // Changed 19/07/21 (kmo): Tighten straight line tolerance (from 0.5)
      minLen = analysis->shadowPosTol.getValue()*vec12.sqrLength();
  }
  else // use normalized offset
    vec12.normalize();

  // Find the triad furthest away from line 1-2, use as point 3
  relPos  = vec12 ^ (node3->getLocalTranslation(this) - point1);
  oldDist = relPos.length();
  for (FmTriad* triad : allTriads)
  {
    relPos = vec12 ^ (triad->getLocalTranslation(this) - point1);
    if (relPos.length() > oldDist)
    {
      node3   = triad;
      oldDist = relPos.length();
    }
  }

  if (oldDist < minLen)
  {
    // All triads are on (or close to) a straight line.
    // Obtain point 3 by rotating vec12 90 degrees in the XY-, YZ- or ZX-plane.
    // Determine which plane by looking at the smallest vector component.
    int imin = 0;
    if (fabs(vec12.y()) < fabs(vec12[imin])) imin = 1;
    if (fabs(vec12.z()) < fabs(vec12[imin])) imin = 2;

    switch (imin) {
    case 0: // Rotate in the YZ-plane
      offset3.x( vec12.x());
      offset3.y(-vec12.z());
      offset3.z( vec12.y());
      break;
    case 1: // Rotate in the XZ-plane
      // Changed 10.07.21 (kmo): Swapped sign on the local X- and Z-axes
      offset3.y( vec12.y());
      offset3.z(-vec12.x());
      offset3.x( vec12.z());
      break;
    case 2: // Rotate in the XY-plane
      offset3.z( vec12.z());
      offset3.x(-vec12.y());
      offset3.y( vec12.x());
      break;
    }
  }

  return true;
}
