// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "FFlLib/FFlLinkHandler.H"
#include "FFlLib/FFlFEParts/FFlBeams.H"
#include "FFlLib/FFlFEParts/FFlNode.H"
#include "FFlLib/FFlIOAdaptors/FFlVTFWriter.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdBeam.H"
#endif

#include "vpmDB/FmBeam.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmSMJointBase.H"
#include "vpmDB/FmGlobalViewSettings.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmRiser.H"
#include "vpmDB/FmJacket.H"
#include "vpmDB/FmSoilPile.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmAnalysis.H"

#include <algorithm>

#ifdef FF_NAMESPACE
using namespace FF_NAMESPACE;
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcBEAM, FmBeam, FmLink);


FmBeam::FmBeam()
{
  Fmd_CONSTRUCTOR_INIT(FmBeam);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdBeam(this);
#endif

  // Remove irrelevant fields inherited from FmIsPositionedBase.
  // These fields will then be ignored on read, write and copy.

  this->removeField("COORDINATE_SYSTEM");
  this->removeField("LOCATION3D_DATA");
  this->removeField("LOCATION3D_POS_VIEW_REF");
  this->removeField("LOCATION3D_ROT_VIEW_REF");

  // Initialize fields

  FFA_FIELD_DEFAULT_INIT(myLocalZaxis, "LOCAL_ZAXIS");

  FFA_REFERENCELIST_FIELD_INIT(myTriadsField, myTriads, "TRIADS");
  myTriads.setAutoSizing(false);

  FFA_REFERENCE_FIELD_INIT(myPropField, myProp, "PROPERTY");
  myProp.setPrintIfZero(false);

  FFA_FIELD_INIT(myVisualize3D, false, "VISUALIZE3D");
  FFA_FIELD_INIT(myVisualize3DAngles, Ints(0,360), "VISUALIZE3D_ANGLES");
}


bool FmBeam::connect(FmTriad* tr1, FmTriad* tr2)
{
  bool status = this->mainConnect();
  this->setTriads(tr1,tr2);
  return status;
}


void FmBeam::setTriads(FmTriad* tr1, FmTriad* tr2)
{
  std::vector<FmTriad*> v(2);
  v[0] = tr1; v[1] = tr2;
  myTriads.setPtrs(v);
}


bool FmBeam::replaceTriad(FmTriad* oldTr, FmTriad* newTr)
{
  for (size_t i = 0; i < myTriads.size(); i++)
    if (myTriads[i] == oldTr)
    {
      myTriads.setPtr(newTr,i);
      return true;
    }

  return false;
}


void FmBeam::removeTriads()
{
  myTriads.setPtrs(std::vector<FmTriad*>(2,static_cast<FmTriad*>(NULL)));
}


FmTriad* FmBeam::getFirstTriad() const
{
  return myTriads.size() < 1 ? static_cast<FmTriad*>(NULL) : myTriads[0];
}


FmTriad* FmBeam::getSecondTriad() const
{
  return myTriads.size() < 2 ? static_cast<FmTriad*>(NULL) : myTriads[1];
}


FmTriad* FmBeam::getOtherTriad(FmTriad* tr) const
{
  if (myTriads.size() != 2)
    return NULL;
  else if (myTriads[0] == tr)
    return myTriads[1];
  else if (myTriads[1] == tr)
    return myTriads[0];

  return NULL;
}


FmTriad* FmBeam::findTriad(int baseID) const
{
  for (size_t i = 0; i < myTriads.size(); i++)
    if (!myTriads[i].isNull())
      if (myTriads[i]->getBaseID() == baseID)
        return myTriads.getPtr(i);

  return NULL;
}


void FmBeam::getTriads(std::vector<FmTriad*>& tr, bool sortOnId) const
{
  myTriads.getPtrs(tr);
  if (sortOnId && tr.size() > 1)
    if (tr.front()->getID() > tr.back()->getID())
      std::swap(tr.front(),tr.back());
}


void FmBeam::setLocalCS(const FaMat34&)
{
  std::cerr <<"FmBeam::setLocalCS: Cannot set beam coordinate system directly."
            << std::endl;
}


void FmBeam::setGlobalCS(const FaMat34&, bool)
{
  std::cerr <<"FmBeam::setGlobalCS: Cannot set beam coordinate system directly."
            << std::endl;
}


void FmBeam::setTranslation(const FaVec3&)
{
  std::cerr <<"FmBeam::setTranslation: Cannot set position of beam directly."
            << std::endl;
}


void FmBeam::setOrientation(const FaMat33&)
{
  std::cerr <<"FmBeam::setOrientation: Cannot set orientation of beam directly."
            << std::endl;
}


const FaMat34& FmBeam::getLocalCS() const
{
  FaMat34& itsCS = const_cast<FaMat34&>(myCS.getValue());
  FmAssemblyBase* parent = this->getPositionedAssembly();
  if (parent)
    itsCS = parent->toLocal(this->getGlobalCS());
  else
    itsCS = this->getGlobalCS();

  return myCS.getValue();
}


FaMat34 FmBeam::getGlobalCS() const
{
  FmTriad* tr1 = this->getFirstTriad();
  if (tr1)
    return FaMat34(this->getGlobalOrientation(),tr1->getGlobalTranslation());
  else
    return FaMat34(this->getGlobalOrientation(),FaVec3());
}


FaVec3 FmBeam::getTranslation() const
{
  FaVec3 pos;
  FmTriad* tr1 = this->getFirstTriad();
  if (tr1) pos = tr1->getGlobalTranslation();

  FmAssemblyBase* parent = this->getPositionedAssembly();
  return parent ? parent->toLocal(pos) : pos;
}


FaMat33 FmBeam::getOrientation() const
{
  FmAssemblyBase* parent = this->getPositionedAssembly();
  if (parent)
    return parent->toLocal(this->getGlobalOrientation());
  else
    return this->getGlobalOrientation();
}


FaMat33 FmBeam::getGlobalOrientation() const
{
  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getSecondTriad();
  FaVec3 beamAxis;
  if (tr1 && tr2)
    beamAxis = tr2->getGlobalTranslation() - tr1->getGlobalTranslation();

  FaMat33 cs;
  if (myLocalZaxis.getValue().isZero())
    cs.makeGlobalizedCS(beamAxis);
  else
  {
    FmAssemblyBase* parent = this->getPositionedAssembly();
    if (parent)
      cs[2] = parent->toGlobal(myLocalZaxis.getValue(),true);
    else
      cs[2] = myLocalZaxis.getValue();

    if (beamAxis.isParallell(cs[2]))
      cs.makeGlobalizedCS(beamAxis);
    else
    {
      cs[0] = beamAxis;
      cs[1] = cs[2] ^ beamAxis;
      cs[0].normalize();
      cs[1].normalize();
      cs[2] = cs[0] ^ cs[1];
    }
  }

  return cs;
}


FaMat34 FmBeam::getPositionCG(bool globalCS) const
{
  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getSecondTriad();
  if (!tr1 || !tr2) return FaMat34();

  FaVec3 Xcg = 0.5*(tr1->getGlobalTranslation() + tr2->getGlobalTranslation());
  if (globalCS)
  {
    FaMat34 CoG(this->getGlobalCS());
    CoG[VW] = Xcg;
    return CoG;
  }

  return FaMat34(this->getGlobalCS().inverse()*Xcg);
}


/*!
  Defines the rigid body CS, using scaled offset.
*/

bool FmBeam::getRefPoints(FmTriad*& node1, FmTriad*& node2, FmTriad*& node3,
                          FaVec3& offset1, FaVec3& offset2, FaVec3& offset3) const
{
  node1 = this->getFirstTriad();
  node2 = this->getSecondTriad();
  node3 = node1;

  offset1 = offset2 = offset3 = FaVec3();

  if (!node1 || !node2) return false; // Beam is not connected

  FaVec3 vec12(node2->getGlobalTranslation() - node1->getGlobalTranslation());

  offset3.y(vec12.length());

  return true;
}


FmBeamProperty* FmBeam::getBeamProperty() const
{
  return dynamic_cast<FmBeamProperty*>(myProp.getPointer());
}


FmBladeProperty* FmBeam::getBladeProperty() const
{
  return dynamic_cast<FmBladeProperty*>(myProp.getPointer());
}


void FmBeam::setProperty(FmModelMemberBase* prop)
{
  myProp.setRef(prop);
  if (prop)
    this->setRGBColor(FmDB::getActiveViewSettings()->getLinkDefaultColor(prop->getID()));
}


/*!
  Returns the density of the internal fluid for beam elements, if any.
*/

double FmBeam::getInternalFluidDensity(bool checkLevel) const
{
  double rho = 0.0;
  FmBeamProperty* bprop = this->getBeamProperty();
  if (!bprop) return rho;
  if (!bprop->hydroToggle.getValue()) return rho;
  if (bprop->Di_hydro.getValue() == 0.0) return rho;

  FmBase* parent = this->getParentAssembly();
  if (!parent) return rho;
  FmSeaState* sea = FmDB::getSeaStateObject(false);
  if (!sea) return rho; // No environment in this model

  // The internal fluid diameter is set.
  // Now check the fluid density (and level) from the parent assembly object.

  double level = 0.0;
  FmRiser* riser = NULL;
  FmJacket* jacket = NULL;
  FmSoilPile* pile = NULL;
  if ((riser = dynamic_cast<FmRiser*>(parent)))
  {
    rho = riser->internalMud.getValue() ? riser->mudDensity.getValue() : 0.0;
    level = riser->mudLevel.getValue();
  }
  else if ((jacket = dynamic_cast<FmJacket*>(parent)))
    rho = jacket->waterFilled.getValue() ? sea->waterDensity.getValue() : 0.0;
  else if ((pile = dynamic_cast<FmSoilPile*>(parent)))
    rho = pile->internalSoil.getValue() ? pile->soilDensity.getValue() : 0.0;

  if (pile || rho <= 0.0 || !checkLevel)
    return rho;

  // This is either a riser or jacket element.
  // Now check if it is below the defined internal fluid level.
  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  FaVec3 XC = 0.5*(triads.back()->getGlobalTranslation() +
                   triads.front()->getGlobalTranslation());
  FaVec3 pos = sea->getLocalCS().inverse()*XC;

  return pos[VZ] > level ? 0.0 : rho;
}


double FmBeam::getLength() const
{
  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getSecondTriad();
  if (!tr1 || !tr2) return 0.0;

  return (tr2->getGlobalTranslation() - tr1->getGlobalTranslation()).length();
}


double FmBeam::getMass() const
{
  FmBeamProperty* beamP = this->getBeamProperty();
  if (beamP)
  {
    // Calculate mass per unit length from beam cross section and material data
    double mass = 0.0;
    if (beamP->crossSectionType.getValue() == FmBeamProperty::GENERIC)
      mass = beamP->Mass.getValue();
    else if (!beamP->material.isNull())
      mass = beamP->A.getValue() * beamP->material->Rho.getValue();
    else
      return mass;

    return mass * this->getLength();
  }

  FmBladeDesign* design = NULL;
  FmBladeProperty* bladeP = this->getBladeProperty();
  if (!bladeP || !bladeP->hasReferringObjs(design,"segment"))
    design = dynamic_cast<FmBladeDesign*>(myProp.getPointer());

  // Calculate element mass from blade properties
  return design ? design->getElementMass(this) : 0.0;
}


bool FmBeam::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmBeam::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmBeam::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmBeam* copyObj = static_cast<FmBeam*>(obj);
  this->setTriads(copyObj->getFirstTriad(),copyObj->getSecondTriad());
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->removeTriads();

  return true;
}


bool FmBeam::attachSMJoint(FmSMJointBase* attachJt, bool isSilent)
{
  bool hadImportantDirections = false;

  // Start the attach check with the dependent triad
  FmTriad* attachedTriad = NULL;
  FmTriad* existingTriad = NULL;
  FmTriad* triadToAttach = attachJt->getSlaveTriad();
  if (triadToAttach)
  {
    if (triadToAttach->isAttached())
      attachedTriad = triadToAttach;
    else if (!this->isTriadAttachable(existingTriad,triadToAttach,isSilent))
      triadToAttach = NULL;
    else if (!existingTriad)
      triadToAttach = NULL;
    else
    {
      hadImportantDirections = existingTriad->importantDirections();
      attachJt->setAsSlaveTriad(existingTriad);
    }
  }

  if (!triadToAttach || attachedTriad)
  {
    // Then check if the other triad already is attached, or unattachable
    triadToAttach = attachJt->getItsMasterTriad();
    if (!triadToAttach)
      return false;
    else if (triadToAttach->isAttached())
      return false;
    else if (!this->isTriadAttachable(existingTriad,triadToAttach,isSilent))
      return false;
    else if (!existingTriad)
      return false;
    else if (existingTriad == attachedTriad)
    {
      // We only found the already attached dependent triad of this joint.
      // Connect the independent triad of this joint to this beam instead.
      if (!this->replaceTriad(existingTriad,triadToAttach))
        return false;

      // Update triad visualization etc.
      existingTriad->updateTopologyInViewer();
      existingTriad->onChanged();
      triadToAttach->updateTopologyInViewer();
      triadToAttach->onChanged();
      return true;
    }

    hadImportantDirections = existingTriad->importantDirections();
    attachJt->setAsMasterTriad(existingTriad);
  }

  if (!hadImportantDirections)
  {
    existingTriad->setGlobalCS(triadToAttach->getGlobalCS());
#ifndef FM_DEBUG
    if (!isSilent)
#endif
      ListUI <<"Warning: The coordinate system of "<< existingTriad->getIdString()
             <<" is changed to match "<< triadToAttach->getIdString() <<".\n";
  }

  // In case the attached Triad is plotted
  std::vector<FmModelMemberBase*> curves;
  triadToAttach->getReferringObjs(curves,"myResultObject[XAXIS]");
  triadToAttach->getReferringObjs(curves,"myResultObject[YAXIS]");
  triadToAttach->releaseReferencesToMe("myResultObject[XAXIS]",existingTriad);
  triadToAttach->releaseReferencesToMe("myResultObject[YAXIS]",existingTriad);
  for (FmModelMemberBase* curve : curves) curve->onChanged();

  // In case the attached Triad has axial spring/dampers or loads
  triadToAttach->releaseReferencesToMe("itsTriads",existingTriad);
  triadToAttach->releaseReferencesToMe("itsOwnerTriad",existingTriad);

  // Update triad visualization etc.
  existingTriad->updateTopologyInViewer();
  existingTriad->onChanged();

  return triadToAttach->erase();
}


bool FmBeam::attachMMJoint(FmMMJointBase* attachJt, bool isSilent)
{
  size_t i, attachStatus = 0;

  std::vector<FmTriad*> triads;
  attachJt->getMasterTriads(triads);
  std::vector<FmTriad*> existing(triads.size(),NULL);

  // Check all independent joint triads
  FmTriad* startTriad = NULL;
  for (i = 0; i < triads.size(); i++)
    if (this->isTriadAttachable(existing[i],triads[i],true) && existing[i])
    {
      if (dynamic_cast<FmPart*>(existing[i]->getOwnerLink(0)))
      {
        ListUI <<"ERROR: Independent joint triads must be attached to beams only.\n";
        return false;
      }
      else
      {
        startTriad = existing[i];
        attachStatus++;
      }
    }

  if (attachStatus == triads.size())
  {
    // All triads have been checked and found attachable, so attach them
    for (i = 0; i < triads.size(); i++)
      if (!this->attachTriad(triads[i],existing[i],isSilent))
        attachStatus = 0;
  }
  else if (startTriad && triads.size() == 2)
  {
    // One of the two triads was found attachable.
    // We need to traverse the beam topology to see if there is a beamstring.
    std::vector<FmIsPlottedBase*> beamTriads;
    this->traverse(startTriad,beamTriads);

    // Check if any of the triads along the beamstring matches the other triad
    double dtol2 = FmDB::getPositionTolerance()*FmDB::getPositionTolerance();
    double tdist = 1.0+dtol2;
    FmTriad* end = existing.front() ? triads.back() : triads.front();
    FaVec3 endPt = end->getGlobalTranslation();
    for (i = 1; i < beamTriads.size() && tdist >= dtol2; i++)
    {
      FmTriad* tr = static_cast<FmTriad*>(beamTriads[i]);
      tdist = (endPt - tr->getGlobalTranslation()).sqrLength();
    }
    if (tdist >= dtol2)
      return false; // The other triad is not on the beamstring

    // Find the other existing triad which is on another beam
    size_t nt = i;
    i = existing.front() ? 1 : 0;
    existing[i] = static_cast<FmTriad*>(beamTriads[nt-1]);

    // Now attach the two end triads
    for (i = 0; i < triads.size(); i++)
      if (!this->attachTriad(triads[i],existing[i],isSilent))
        attachStatus = 0;

    // Then add the intermediate triads
    for (i = 1; i < nt-1; i++)
      if (attachJt->insertAsMaster(static_cast<FmTriad*>(beamTriads[i]),i))
      {
        beamTriads[i]->updateTopologyInViewer();
        beamTriads[i]->onChanged();
      }
      else
        attachStatus = 0;
  }

  return attachStatus > 0;
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmBeam::writeFMF(std::ostream& os)
{
  os <<"BEAM\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmBeam::readAndConnect(std::istream& is, std::ostream&)
{
  FmBeam* obj = new FmBeam();

  // Obsolete fields
  FFaObsoleteField<int> startAngle, stopAngle;
  FFA_OBSOLETE_FIELD_INIT(startAngle,0,"VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_INIT(stopAngle,360,"VISUALIZE3D_STOP_ANGLE",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_STOP_ANGLE",obj);

  // Update from old model file
  if (startAngle.wasOnFile())
    obj->myVisualize3DAngles.getValue().first = startAngle.getValue();
  if (stopAngle.wasOnFile())
    obj->myVisualize3DAngles.getValue().second = stopAngle.getValue();

  obj->connect();
  return true;
}


void FmBeam::initAfterResolve()
{
  this->FmLink::initAfterResolve();

  // Set local Z-direction vector for beams when reading older model files
  if (FmDB::getModelFileVer() < FFaVersionNumber(7,1,0,1))
  {
    if (this->getBladeProperty())
      this->setOrientation(myCS.getValue()[VX]);
    else
      this->setOrientation(myCS.getValue()[VZ]);
  }
}


int FmBeam::printSolverEntry(FILE* fp, int propId, FmBeamProperty* bProp,
                             const std::string* rdbPath)
{
  int err = 0;
  fprintf(fp,"&SUP_EL\n");
  this->printID(fp);

  FFaString bDesc = this->getUserDescription();

  if (bProp && bProp->hydroToggle.getValue())
  {
    // Geometry file for buoyancy calculation
    std::string bodyFile(this->getGeometryFile());
    if (this->getInternalFluidDensity(true) > 0.0)
      fprintf(fp,"  bodyFile = 'FILLED'\n");
    else if (bodyFile.empty())
      fprintf(fp,"  bodyFile = 'NONE'\n");
    else if (rdbPath)
      fprintf(fp,"  bodyFile = '%s'\n",
              FFaFilePath::getRelativeFilename(*rdbPath,bodyFile).c_str());
  }

  // Beam triads (should be 2)
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  fprintf(fp,"  numTriads = %u\n", (unsigned int)triads.size());
  fprintf(fp,"  triadIds =");
  for (FmTriad* triad : triads)
    fprintf(fp," %d",triad->getBaseID());
  fprintf(fp,"\n");

  // Beam properties
  if (bProp)
    propId = bProp->getBaseID();
  else if (propId <= 0 && !myProp.isNull())
    propId = myProp->getBaseID();
  if (propId > 0)
    fprintf(fp,"  elPropId = %d\n", propId);
  else
  {
    ListUI <<"\n---> ERROR: "<< this->getIdString(true)
           <<" has no cross section property.\n";
    err++;
  }

  // Corotated reference coordinate system positioning
  int shadowPosAlg = 0;
  switch (myCSOption.getValue())
    {
    case FmLink::MAX_TRI_UNIT_OFFSET:
    case FmLink::MAX_TRI_LINK_SCALE_OFFSET:
      shadowPosAlg = 1; // Triangle fit based on beam triads and offset
      break;
    case FmPart::MASS_BASED_AVERAGE:
      shadowPosAlg = 2; // Mass based average
      break;
    default:
      shadowPosAlg = FmDB::getActiveAnalysis()->defaultShadowPosAlg.getValue();
      if (shadowPosAlg == 0 || shadowPosAlg == 4)
        shadowPosAlg = 1;
      else if (shadowPosAlg == 3)
        shadowPosAlg = 2;
      break;
    }
  fprintf(fp,"  shadowPosAlg = %d\n", shadowPosAlg);

  if (shadowPosAlg == 1)
  {
    // Corotated coordinate system reference triads
    // Get triads and offsets based on chosen offset algorithm
    FmTriad* ref[3];
    FaVec3   off[3];
    if (!this->getRefPoints(ref[0],ref[1],ref[2],off[0],off[1],off[2]))
    {
      ListUI <<"\n---> ERROR: "<< this->getIdString(true)
             <<" is not connected.\n";
      err++;
    }
    else for (int i = 0; i < 3; i++)
      fprintf(fp,"  refTriad%dId = %d, offset%d =%17.9e %17.9e %17.9e\n",
              i+1, ref[i]->getBaseID(), i+1, off[i][0], off[i][1], off[i][2]);
  }

  fprintf(fp,"  massCorrFlag = 0\n");

  // Scaling of dynamic properties
  fprintf(fp,"  stiffScale =%17.9e\n", stiffnessScale.getValue());
  fprintf(fp,"  massScale  =%17.9e\n", massScale.getValue());

  // Beta feature: Time-dependent stiffness scaling
  int stifSclEngine = bDesc.getIntAfter("#StiffScaleEngine");
  if (stifSclEngine > 0) {
    fprintf(fp,"  stiffEngineId = %d\n", stifSclEngine);
    FmEngine::betaFeatureEngines.insert(stifSclEngine);
  }

  // Beta feature: Time-dependent mass scaling
  int massSclEngine = bDesc.getIntAfter("#MassScaleEngine");
  if (massSclEngine > 0) {
    fprintf(fp,"  massEngineId = %d\n", massSclEngine);
    FmEngine::betaFeatureEngines.insert(massSclEngine);
  }

  // Structural damping coefficients
  fprintf(fp,"  alpha1 =%17.9e," , alpha1.getValue());
  fprintf(fp,"  alpha2 =%17.9e\n", alpha2.getValue());

  // Possibly time-dependent structural damping
  int structDmpEngine = this->getStructDmpEngineId();
  if (structDmpEngine > 0)
    fprintf(fp,"  strDmpEngineId = %d\n", structDmpEngine);

  // Element position
  FaMat34 lCS = this->getGlobalCS();
  fprintf(fp,"  supPos =%17.9e %17.9e %17.9e %17.9e\n",
          lCS[0][0],lCS[1][0],lCS[2][0],lCS[3][0]);
  fprintf(fp,"          %17.9e %17.9e %17.9e %17.9e\n",
          lCS[0][1],lCS[1][1],lCS[2][1],lCS[3][1]);
  fprintf(fp,"          %17.9e %17.9e %17.9e %17.9e\n",
          lCS[0][2],lCS[1][2],lCS[2][2],lCS[3][2]);

  // Beta feature: Output of position matrices for specified beams
  if (bDesc.hasSubString("#savePos"))
    fprintf(fp,"  savePos = 1\n");

  // Variables to be saved
  // 1 - Center of gravity
  // 2 - Generalized DOF components (dis,vel,acc)
  // 3 - Energies
  this->writeSaveVar(fp,3);

  fprintf(fp,"/\n");
  return err;
}


/*!
  Writes the FE data of this beam to the specified \a vtfFile.
*/

bool FmBeam::writeToVTF(VTFAFile& vtfFile, IntVec*, IntVec*)
{
  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getSecondTriad();
  if (!tr1 || !tr2) return false;

  // Create an FE part consisting of a single beam element only
  // The nodal coordinates must be local to the beam CS
  FFlLinkHandler* feData = new FFlLinkHandler();
  FFlBEAM2* beam = new FFlBEAM2(1);
  FFlNode* node = new FFlNode(1,tr1->getLocalTranslation(this));
  beam->setNode(1,node);
  feData->addNode(node);
  node = new FFlNode(2,tr2->getLocalTranslation(this));
  beam->setNode(2,node);
  feData->addNode(node);
  feData->addElement(beam);
  feData->buildFiniteElementVec();

  // Append FE data of this beam to the VTF file
  FFaMsg::setSubTask(this->getUserDescription());
  FFlVTFWriter vtf(feData);
  bool success = vtf.write(vtfFile,this->getUserDescription(),-this->getBaseID());
  FFaMsg::setSubTask("");

  delete feData;
  return success;
}


/*!
  This method is used to get all beam elements along a generated beamstring.
  Its main purpose is for easy generation of force- and moment diagrams.
*/

int FmBeam::traverse(FmBase* start, std::vector<FmIsPlottedBase*>& objs)
{
  FmBeam* beam = dynamic_cast<FmBeam*>(start);
  if (!beam) return 0;

  std::vector<FmBeam*> beams;

  // Check which triad is the end triad of the beamstring, if any
  FmTriad* triad = beam->getFirstTriad();
  if (triad)
  {
    triad->getBeamBinding(beams);
    if (beams.size() != 1)
    {
      triad = beam->getSecondTriad();
      if (triad)
        triad->getBeamBinding(beams);
    }
  }
  if (beams.size() != 1 || beams.front() != beam)
    return 0; // this beam element is not at the end

  int nB = 0;
  while (beam)
  {
    // Check if this element is already in the list.
    // If so, we have a looping structure and have to stop the traveral.
    bool looping = std::find(objs.begin(),objs.end(),beam) != objs.end();

    // We have a valid beam element
    nB++;
    objs.push_back(beam);
    if (looping) break;

    // Beta feature: Terminate the traversal at a user-specified beam
    if (beam->getUserDescription().find("#Stop") != std::string::npos) break;

    // Get the next triad in the chain
    FmTriad* tr1 = beam->getFirstTriad();
    FmTriad* tr2 = beam->getSecondTriad();
    triad = triad == tr1 ? tr2 : tr1;
    if (!triad) return 0; // should not happen (logic error)

    // Get next beam in the chain
    triad->getBeamBinding(beams);
    if (beams.size() == 2)
      beam = beams.front() == beam ? beams.back() : beams.front();
    else
      beam = NULL; // chain ended

    // Check if the beamstring is interrupted by point joints.
    // If so, continue the traversal on "the other side" of it,
    // by invoking this method recursively.
    if (!beam) {
      FmSMJointBase* jt = NULL;
      if (triad->hasReferringObjs(jt,"itsMasterTriad"))
        return nB + traverse(jt->getSlaveTriad()->getOwnerLink(),objs);
      else if (triad->hasReferringObjs(jt,"itsSlaveTriad"))
        return nB + traverse(jt->getItsMasterTriad()->getOwnerLink(),objs);
    }
  }

  return nB;
}


int FmBeam::traverse(FmTriad* triad, std::vector<FmIsPlottedBase*>& objs) const
{
  std::vector<FmBeam*> beams;
  const FmBeam* beam = this;
  int nBeamElm = 0;
  while (true)
  {
    // Check if this triad is already in the list.
    // If so, we have a looping structure and have to stop the traveral.
    bool looping = std::find(objs.begin(),objs.end(),triad) != objs.end();

    // We have a valid beam element
    nBeamElm++;
    objs.push_back(triad);
    if (looping) return nBeamElm;

    // Get the next triad in the chain
    FmTriad* tr1 = beam->getFirstTriad();
    FmTriad* tr2 = beam->getSecondTriad();
    triad = triad == tr1 ? tr2 : tr1;
    if (!triad) return 0; // should not happen (logic error)

    // Beta feature: Terminate the traversal at a user-specified beam
    if (beam->getUserDescription().find("#Stop") != std::string::npos) break;

    // Get next beam in the chain
    triad->getBeamBinding(beams);
    if (beams.size() == 2)
      beam = beams.front() == beam ? beams.back() : beams.front();
    else
      break; // chain ended
  }

  // Make sure we also include the second node of the last valid element
  objs.push_back(triad);
  return nBeamElm;
}


bool FmBeam::split(const DoubleVec& rlen)
{
  if (dynamic_cast<FmTower*>(this->getParentAssembly()))
  {
    FFaMsg::dialog("Can not split a tower beam element.\n"
                   "Use the Turbine Tower Definition dialog instead.",
                   FFaMsg::ERROR);
    return false;
  }
  else if (rlen.size() < 2)
    return false;

  double totalLen = 0.0;
  for (double bLen : rlen)
    if (bLen > 0.0)
      totalLen += bLen;
    else
      return false;

  DoubleVec xi(rlen);
  for (double& xin : xi) xin /= totalLen;

  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getSecondTriad();
  if (!tr1 || !tr2) return false;

  FaVec3 start = tr1->getGlobalTranslation();
  FaVec3 axis = tr2->getGlobalTranslation() - start;
  FmAssemblyBase* p = dynamic_cast<FmAssemblyBase*>(this->getParentAssembly());
  if (p)
  {
    start = p->toLocal(start);
    axis = p->toLocal(axis,true);
  }

  totalLen = xi.front();
  FmTriad* triad = new FmTriad(start+axis*totalLen);
  triad->setParentAssembly(this->getParentAssembly());
  triad->connect();
  this->setTriads(tr1,triad);
  this->draw();
  triad->draw();

  tr1 = triad;
  FmBeam* beam;
  for (size_t i = 1; i+1 < xi.size(); i++)
  {
    totalLen += xi[i];
    triad = new FmTriad(start+axis*totalLen);
    triad->setParentAssembly(this->getParentAssembly());
    triad->connect();

    beam = new FmBeam();
    beam->setParentAssembly(this->getParentAssembly());
    beam->setProperty(this->getProperty());
    beam->setOrientation(myLocalZaxis.getValue());
    beam->connect(tr1,triad);
    beam->draw();
    triad->draw();
    tr1 = triad;
  }

  beam = new FmBeam();
  beam->setParentAssembly(this->getParentAssembly());
  beam->setProperty(this->getProperty());
  beam->setOrientation(myLocalZaxis.getValue());
  beam->connect(tr1,tr2);
  beam->draw();
  tr2->draw();

  return true;
}
