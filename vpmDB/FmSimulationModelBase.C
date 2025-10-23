// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <sstream>

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmSimulationEvent.H"
#include "vpmDB/FmAirState.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmArcSegmentMaster.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmBearingFriction.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmCamFriction.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmCtrlLine.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmDofLoad.H"
#include "vpmDB/FmDofMotion.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmEngine.H"
#ifdef FT_HAS_EXTCTRL
#include "vpmDB/FmExternalCtrlSys.H"
#endif
#include "vpmDB/FmFileReference.H"
#include "vpmDB/FmFppOptions.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmGageOptions.H"
#include "vpmDB/FmGear.H"
#include "vpmDB/FmGenericDBObject.H"
#include "vpmDB/FmJointDamper.H"
#include "vpmDB/FmJointMotion.H"
#include "vpmDB/FmJointSpring.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmModesOptions.H"
#include "vpmDB/FmPipeSurface.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmPrismaticFriction.H"
#include "vpmDB/FmRackPinion.H"
#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmRelativeSensor.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmRoad.H"
#include "vpmDB/FmRotFriction.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmSpringChar.H"
#include "vpmDB/FmSticker.H"
#include "vpmDB/FmStraightMaster.H"
#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmTire.H"
#include "vpmDB/FmTransFriction.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmFuncAdmin.H"
#include "vpmDB/FmControlAdmin.H"


Fmd_DB_SOURCE_INIT(FcSIMULATION_MODEL_BASE, FmSimulationModelBase, FmModelMemberBase);
Fmd_DB_SOURCE_INIT(FcSTRUCT_PROPERTY_BASE, FmStructPropertyBase, FmSimulationModelBase);

std::string FmSimulationModelBase::relPathCorrection;


FmSimulationModelBase::FmSimulationModelBase()
{
  Fmd_CONSTRUCTOR_INIT(FmSimulationModelBase);
}

FmStructPropertyBase::FmStructPropertyBase()
{
  Fmd_CONSTRUCTOR_INIT(FmStructPropertyBase);
}


bool FmSimulationModelBase::parseField(const std::string& keyWord,
				       const std::string& fieldValue)
{
  std::istringstream activeStatement(fieldValue.c_str());
  return this->readField(keyWord,activeStatement,true);
}


bool FmSimulationModelBase::localParse(const char* keyWord, std::istream& activeStatement,
				       FmSimulationModelBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmSimulationModelBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmSimulationModelBase::getClassTypeID());
}


void FmSimulationModelBase::initAfterResolve()
{
  // Initializations based on field values read from file
  this->initAfterParse();
}


/*!
  Removes all fields from this object, except for the specified ones.
*/

void FmSimulationModelBase::removeFieldsExceptFor(const std::vector<FDictIt>& keep)
{
  // Identify the fields that should be removed
  std::vector<FDictIt> toBeRemoved;
  for (const FieldContainerMap::value_type& field : myFields)
    if (std::find(keep.begin(),keep.end(),field.first) == keep.end())
      toBeRemoved.push_back(field.first);

  // Remove the found fields from the field container,
  // such that they are not referred when reading and writing model files
  for (FDictIt& it : toBeRemoved)
    myFields.erase(it);
}


/*!
  Updates the default value to the current value for the specified fields.
*/

void FmSimulationModelBase::setAsDefault(const std::vector<FDictIt>& fieldNames)
{
  for (const FDictIt& field : fieldNames)
  {
    FieldContainerMap::iterator it = myFields.find(field);
    if (it != myFields.end()) it->second->updateDefault();
  }
}


/*!
  Returns all field names present in this object.
*/

void FmSimulationModelBase::getFields(std::vector<FDictIt>& fieldNames) const
{
  for (const FieldContainerMap::value_type& field : myFields)
    fieldNames.push_back(field.first);
}


/*!
  Checks if \a *this is referred by simulation events.
*/

bool FmSimulationModelBase::isEventModified() const
{
  FmSimulationEvent* event;
  return this->hasReferringObjs(event);
}


/*!
  Returns a string with all information stored in this object.
*/

std::string FmSimulationModelBase::getObjectInfo() const
{
  std::string info(this->getTypeIDName());
  if (info.substr(0,3) == "Fcf")
    info.replace(0,3,"FUNC_");
  else if (info.substr(0,3) == "Fcc")
    info.replace(0,3,"CONTROL_");
  else
    info.erase(0,2);

  info.append("\n\n");
  for (const FieldContainerMap::value_type& field : myFields)
  {
    info.append(*field.first);
    if (field.second->isPrintable())
    {
      info.append("  ");
      std::stringstream is;
      is << *field.second;
      info.append(is.str());
    }
    else
      info.append("  (empty)");
    info.append("\n");
  }

  return info;
}


/*!
  Returns a new simulation model object of the same type as \a this.
  If \a cloneDepth > FmBase::NOTHING the field values are also copied.
*/

FmSimulationModelBase* FmSimulationModelBase::copy(int cloneDepth) const
{
  FmSimulationModelBase* newObj = NULL;
  if (this->isOfType(FmMathFuncBase::getClassTypeID()))
    newObj = FmFuncAdmin::createFunction(this->getTypeID());
  else if (this->isOfType(FmCtrlElementBase::getClassTypeID()))
    newObj = FmControlAdmin::createElement(this->getTypeID());
  else if (this->isOfType(FmAirState::getClassTypeID()))
    newObj = new FmAirState();
  else if (this->isOfType(FmAnalysis::getClassTypeID()))
    newObj = new FmAnalysis();
  else if (this->isOfType(FmArcSegmentMaster::getClassTypeID()))
    newObj = new FmArcSegmentMaster();
  else if (this->isOfType(FmAxialDamper::getClassTypeID()))
    newObj = new FmAxialDamper();
  else if (this->isOfType(FmAxialSpring::getClassTypeID()))
    newObj = new FmAxialSpring();
  else if (this->isOfType(FmBallJoint::getClassTypeID()))
    newObj = new FmBallJoint();
  else if (this->isOfType(FmMaterialProperty::getClassTypeID()))
    newObj = new FmMaterialProperty();
  else if (this->isOfType(FmBeamProperty::getClassTypeID()))
    newObj = new FmBeamProperty();
  else if (this->isOfType(FmBearingFriction::getClassTypeID()))
    newObj = new FmBearingFriction();
  else if (this->isOfType(FmBladeProperty::getClassTypeID()))
    newObj = new FmBladeProperty();
  else if (this->isOfType(FmCamFriction::getClassTypeID()))
    newObj = new FmCamFriction();
  else if (this->isOfType(FmCamJoint::getClassTypeID()))
    newObj = new FmCamJoint();
  else if (this->isOfType(FmCtrlLine::getClassTypeID()))
    newObj = new FmCtrlLine();
  else if (this->isOfType(FmCylJoint::getClassTypeID()))
    newObj = new FmCylJoint();
  else if (this->isOfType(FmDofLoad::getClassTypeID()))
    newObj = new FmDofLoad();
  else if (this->isOfType(FmDofMotion::getClassTypeID()))
    newObj = new FmDofMotion();
  else if (this->isOfType(FmElementGroupProxy::getClassTypeID()))
    newObj = new FmElementGroupProxy();
  else if (this->isOfType(FmEngine::getClassTypeID()))
    newObj = new FmEngine();
#ifdef FT_HAS_EXTCTRL
  else if (this->isOfType(FmExternalCtrlSys::getClassTypeID()))
    newObj = new FmExternalCtrlSys();
#endif
  else if (this->isOfType(FmFileReference::getClassTypeID()))
    newObj = new FmFileReference();
  else if (this->isOfType(FmFppOptions::getClassTypeID()))
    newObj = new FmFppOptions();
  else if (this->isOfType(FmFreeJoint::getClassTypeID()))
    newObj = new FmFreeJoint();
  else if (this->isOfType(FmGageOptions::getClassTypeID()))
    newObj = new FmGageOptions();
  else if (this->isOfType(FmGear::getClassTypeID()))
    newObj = new FmGear();
  else if (this->isOfType(FmGenericDBObject::getClassTypeID()))
    newObj = new FmGenericDBObject();
  else if (this->isOfType(FmJointDamper::getClassTypeID()))
    newObj = new FmJointDamper();
  else if (this->isOfType(FmJointMotion::getClassTypeID()))
    newObj = new FmJointMotion();
  else if (this->isOfType(FmJointSpring::getClassTypeID()))
    newObj = new FmJointSpring();
  else if (this->isOfType(FmPart::getClassTypeID()))
    newObj = new FmPart();
  else if (this->isOfType(FmBeam::getClassTypeID()))
    newObj = new FmBeam();
  else if (this->isOfType(FmLoad::getClassTypeID()))
    newObj = new FmLoad();
  else if (this->isOfType(FmMechanism::getClassTypeID()))
    newObj = new FmMechanism();
  else if (this->isOfType(FmModesOptions::getClassTypeID()))
    newObj = new FmModesOptions();
  else if (this->isOfType(FmPipeSurface::getClassTypeID()))
    newObj = new FmPipeSurface();
  else if (this->isOfType(FmPrismJoint::getClassTypeID()))
    newObj = new FmPrismJoint();
  else if (this->isOfType(FmPrismaticFriction::getClassTypeID()))
    newObj = new FmPrismaticFriction();
  else if (this->isOfType(FmRackPinion::getClassTypeID()))
    newObj = new FmRackPinion();
  else if (this->isOfType(FmRefPlane::getClassTypeID()))
    newObj = new FmRefPlane();
  else if (this->isOfType(FmRelativeSensor::getClassTypeID()))
    newObj = new FmRelativeSensor();
  else if (this->isOfType(FmRevJoint::getClassTypeID()))
    newObj = new FmRevJoint();
  else if (this->isOfType(FmRigidJoint::getClassTypeID()))
    newObj = new FmRigidJoint();
  else if (this->isOfType(FmRoad::getClassTypeID()))
    newObj = new FmRoad();
  else if (this->isOfType(FmRotFriction::getClassTypeID()))
    newObj = new FmRotFriction();
  else if (this->isOfType(FmSeaState::getClassTypeID()))
    newObj = new FmSeaState();
  else if (this->isOfType(FmSimpleSensor::getClassTypeID()))
    newObj = new FmSimpleSensor();
  else if (this->isOfType(FmSpringChar::getClassTypeID()))
    newObj = new FmSpringChar();
  else if (this->isOfType(FmSticker::getClassTypeID()))
    newObj = new FmSticker();
  else if (this->isOfType(FmStraightMaster::getClassTypeID()))
    newObj = new FmStraightMaster();
  else if (this->isOfType(FmStrainRosette::getClassTypeID()))
    newObj = new FmStrainRosette();
  else if (this->isOfType(FmTire::getClassTypeID()))
    newObj = new FmTire();
  else if (this->isOfType(FmTransFriction::getClassTypeID()))
    newObj = new FmTransFriction();
  else if (this->isOfType(FmTriad::getClassTypeID()))
    newObj = new FmTriad();
  else if (this->isOfType(FmVesselMotion::getClassTypeID()))
    newObj = new FmVesselMotion();

  if (newObj && cloneDepth > FmBase::NOTHING) {
    newObj->clone(const_cast<FmSimulationModelBase*>(this),cloneDepth);
    newObj->setUserDescription("Copy of " + this->getInfoString());
  }

  return newObj;
}


FmBase* FmStructPropertyBase::duplicate() const
{
  FmBase* prop = this->copy(FmBase::DEEP_APPEND);
  prop->connect();
  return prop;
}
