// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmCreate.C
  \brief Global functions for creating positioned mechanism objects.
*/

#include "vpmDB/FmCreate.H"
#include "vpmDB/FmStructAssembly.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmRefPlane.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmBallJoint.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmPrismJoint.H"
#include "vpmDB/FmCylJoint.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmStraightMaster.H"
#include "vpmDB/FmGear.H"
#include "vpmDB/FmRackPinion.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmTire.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmSensorBase.H"
#include "vpmDB/FmSticker.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmGlobalViewSettings.H"
#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmUserDefinedElement.H"
#include "FiUserElmPlugin/FiUserElmPlugin.H"
#include "chainShape/ChainShape.H"

#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include <functional>


static FmTriad* getTriadOn(FmBase* obj, const FaVec3& point)
{
  double posTol = FmDB::getPositionTolerance();
  FmTriad* retTriad = NULL;
  if (obj)
  {
    if (obj->isOfType(FmLink::getClassTypeID()))
      retTriad = static_cast<FmLink*>(obj)->getTriadAtPoint(point,posTol,true);
    else if (obj->isOfType(FmTriad::getClassTypeID()))
      retTriad = static_cast<FmTriad*>(obj);
    else if (obj->isOfType(FmFreeJoint::getClassTypeID()))
      retTriad = NULL;
    else if (obj->isOfType(FmSMJointBase::getClassTypeID()))
      retTriad = static_cast<FmSMJointBase*>(obj)->getSlaveTriad();
  }

  // Compare the specified position with the correct triad position
  if (!retTriad || !retTriad->getGlobalTranslation().equals(point,posTol))
  {
    retTriad = new FmTriad(point); // the positions differ, a new triad is made
    if (obj && obj->isOfType(FmPart::getClassTypeID()))
      retTriad->connect(obj);
    else
      retTriad->connect();
  }

  return retTriad;
}


FmTriad* Fedem::createTriad(const FaVec3& createPos, FmBase* onObject)
{
  FFaMsg::list("Creating Triad");

  FmRefPlane* rp = NULL;
  FmPart* onPart = NULL;
  FmTriad* triad = new FmTriad();
  if (onObject)
  {
    if (onObject->isOfType(FmSubAssembly::getClassTypeID()))
    {
      FFaMsg::list(" in "+ onObject->getIdString());
      triad->setParentAssembly(onObject);
    }
    else if (onObject->isOfType(FmPart::getClassTypeID()))
    {
      FFaMsg::list(" on "+ onObject->getIdString());
      triad->setParentAssembly(onObject->getParentAssembly());
      onPart = static_cast<FmPart*>(onObject);
    }
    else if (onObject->isOfType(FmRefPlane::getClassTypeID()))
    {
      FFaMsg::list(" on "+ onObject->getIdString());
      rp = static_cast<FmRefPlane*>(onObject);
    }
    else if (onObject->getParentAssembly())
    {
      FFaMsg::list(" in "+ onObject->getParentAssembly()->getIdString());
      triad->setParentAssembly(onObject->getParentAssembly());
    }
  }
  FFaMsg::list(".\n");

  FmAssemblyBase* parent = triad->getPositionedAssembly();
  if (parent)
    triad->setTranslation(parent->toLocal(createPos));
  else
    triad->setTranslation(createPos);

  triad->connect();
  triad->draw();

  if (onPart)
  {
    triad->setOrientation(onPart->getOrientation());
    if (onPart->attach(triad))
      return triad;
  }
  else if (rp)
  {
    if (rp->attach(triad))
      return triad;
  }
  else
    return triad;

  FFaMsg::list("ERROR: Attachment failure. Triad not created.\n");
  triad->erase();
  return NULL;
}


static FaMat33 getCreationMX(const FaVec3& zAxisDir,
                             const FaVec3* yAxisDir = NULL)
{
  FaVec3 xAxis, yAxis, zAxis;

  // The joint is oriented with the "up" vector closest to
  // the negative g-vector unless yAxisDir is specified,
  // and the z-vector in zAxisDir

  if (zAxisDir.length() < FmDB::getPositionTolerance())
    zAxis = FaVec3(0.0,0.0,1.0);
  else
    zAxis = zAxisDir;

  if (yAxisDir)
    yAxis = *yAxisDir;
  else if (FmDB::getGrav().length() < FmDB::getPositionTolerance())
    yAxis =  FaVec3(0.0,-1.0,0.0);
  else
    yAxis = -FmDB::getGrav();

  xAxis = yAxis^zAxis;
  if (xAxis.length() < FmDB::getPositionTolerance())
  {
    // The yAxis is either parallel to the zAxis or not given (zero).
    // Choose the closest global axis instead.
    double z1 = zAxis.x();
    double z2 = zAxis.y();
    double z3 = zAxis.z();
    if (fabs(z1) < fabs(z2) && fabs(z1) < fabs(z3))
      xAxis = FaVec3(0.0,-z3,z2); // = [1,0,0]x(Z-axis)
    else if (fabs(z2) < fabs(z1) && fabs(z2) < fabs(z3))
      xAxis = FaVec3(z3,0.0,-z1); // = [0,1,0]x(Z-axis)
    else
      xAxis = FaVec3(-z2,z1,0.0); // = [0,0,1]x(Z-axis)
  }

  zAxis.normalize();
  xAxis.normalize();
  yAxis = zAxis^xAxis;

  return FaMat33(xAxis,yAxis,zAxis);
}


void Fedem::createFreeJoint(const FaVec3& posJnt, const FaVec3& posDep,
                            const FaVec3* zAxisDir)
{
  FFaMsg::list("Creating Free joint.\n");

  FmTriad*    triad1 = new FmTriad();
  FmTriad*    triad2 = new FmTriad();
  FmFreeJoint* joint = new FmFreeJoint();

  FmSticker* sticker1 = new FmSticker(posJnt);
  FmSticker* sticker2 = new FmSticker(posDep);

  if (zAxisDir) {
    // Use same initial rotation for both triads to avoid singularities
    triad1->setLocalCS(FaMat34(getCreationMX(*zAxisDir),posJnt));
    triad2->setLocalCS(FaMat34(getCreationMX(*zAxisDir),posDep));
  }
  else {
    triad1->setLocalCS(FaMat34(posJnt));
    triad2->setLocalCS(FaMat34(posDep));
  }

  triad1->addSticker(sticker1);
  triad2->addSticker(sticker2);

  joint->setAsMasterTriad(triad1);
  joint->setAsSlaveTriad(triad2);
  joint->updateLocation();

  triad1->connect();
  triad2->connect();
  joint->connect();

  triad1->draw();
  triad2->draw();
  sticker1->draw();
  sticker2->draw();
  joint->draw();
}


FmAxialSpring* Fedem::createAxialSpring(FmTriad* first, FmTriad* second,
                                        FmBase* subAssembly)
{
  if (!first || !second)
  {
    FFaMsg::list("ERROR: Unspecified axial spring triad(s).\n");
    return NULL;
  }

  FFaMsg::list("Creating Axial spring.\n");

  FmAxialSpring* spring = new FmAxialSpring();
  if (subAssembly)
    spring->setParentAssembly(subAssembly);
  else
    spring->setParentAssembly(first->getCommonAncestor(second));
  spring->connect(first,second);

  spring->draw();
  first->draw();
  second->draw();

  return spring;
}


void Fedem::createAxialSpring(const FaVec3& firstPt, const FaVec3& secondPt,
                              FmBase* first, FmBase* second)
{
  createAxialSpring(getTriadOn(first,firstPt), getTriadOn(second,secondPt));
}


FmAxialDamper* Fedem::createAxialDamper(FmTriad* first, FmTriad* second,
                                        FmBase* subAssembly)
{
  if (!first || !second)
  {
    FFaMsg::list("ERROR: Unspecified axial damper triad(s).\n");
    return NULL;
  }

  FFaMsg::list("Creating Axial damper.\n");

  FmAxialDamper* damper = new FmAxialDamper();
  if (subAssembly)
    damper->setParentAssembly(subAssembly);
  else
    damper->setParentAssembly(first->getCommonAncestor(second));
  damper->connect(first,second);

  first->draw();
  second->draw();
  damper->draw();

  return damper;
}


void Fedem::createAxialDamper(const FaVec3& firstPt, const FaVec3& secondPt,
                              FmBase* first, FmBase* second)
{
  createAxialDamper(getTriadOn(first,firstPt), getTriadOn(second,secondPt));
}


FmLoad* Fedem::createLoad(int lType, const FaVec3& createPos, FaVec3 globalDir,
                          FmBase* onObject, FmBase* subAssembly)
{
  if (globalDir.length() < FmDB::getPositionTolerance())
    globalDir = FaVec3(1.0,0.0,0.0);

  // scale the direction normal
  globalDir *= FmDB::getActiveViewSettings()->getSymbolScale();

  FmTriad* triad = getTriadOn(onObject,createPos);
  FaVec3   from  = triad->getGlobalTranslation();

  if (lType == FmLoad::TORQUE)
    FFaMsg::list("Creating Torque.\n");
  else
    FFaMsg::list("Creating Force.\n");

  FmLoad* force = new FmLoad();
  if (subAssembly)
    force->setParentAssembly(subAssembly);
  else
    force->setParentAssembly(triad->getParentAssembly());
  force->setLoadType((FmLoad::LoadType)lType);
  force->connect(triad,FmDB::getEarthLink(),from,
                 FmDB::getEarthLink(),from+globalDir);

  triad->draw();
  force->draw();

  return force;
}


void Fedem::createSticker(FmBase* object, const FaVec3& createPoint)
{
  if (!object)
    return;

  FFaMsg::list("Creating Sticker.\n");

  FmSticker* sticker = new FmSticker(createPoint);
  sticker->setParentAssembly(object->getParentAssembly());
  sticker->connect(object);
  sticker->draw();
}


void Fedem::createSticker(const FaVec3& createPoint, FmBase* onObject)
{
  if (!onObject)
    return;

  if (onObject->isOfType(FmFreeJoint::getClassTypeID()))
    createSticker(onObject,createPoint);
  else if (onObject->isOfType(FmSMJointBase::getClassTypeID()))
    createSticker(static_cast<FmSMJointBase*>(onObject)->getSlaveTriad(),createPoint);
  else if (onObject->isOfType(FmIsPositionedBase::getClassTypeID()))
    createSticker(onObject,createPoint);
}


void Fedem::createGear(FmBase* first, FmBase* second)
{
  FmRevJoint* inputJoint = dynamic_cast<FmRevJoint*>(first);
  FmRevJoint* outputJoint = dynamic_cast<FmRevJoint*>(second);

  if (!inputJoint || !outputJoint)
    FFaMsg::list("ERROR: Both input and output joint must be selected.\n");
  else if (inputJoint == outputJoint)
    FFaMsg::list("ERROR: Different joints must be selected for input and output.\n");
  else if (inputJoint->getHPConnection())
    FFaMsg::list("ERROR: Input joint is already in use by a Gear.\n");
  else if (outputJoint->hasHPConnections())
    FFaMsg::list("ERROR: Output joint is already in use by a Gear.\n");
  else
  {
    FFaMsg::list("Creating Gear transmission.\n");
    FmGear* tmpGear = new FmGear();
    tmpGear->setParentAssembly(inputJoint->getCommonAncestor(outputJoint));
    tmpGear->connect(inputJoint,outputJoint);
    tmpGear->draw();
    return;
  }
  FFaMsg::list("       Could not create Gear.\n");
}


void Fedem::createRackPinion(FmBase* first, FmBase* second)
{
  FmRevJoint* inputJoint = dynamic_cast<FmRevJoint*>(first);
  FmPrismJoint* outputJoint = dynamic_cast<FmPrismJoint*>(second);

  if (!inputJoint || !outputJoint)
    FFaMsg::list("ERROR: Both input and output joint must be selected.\n");
  else if (inputJoint->getHPConnection())
    FFaMsg::list("ERROR: Input joint is already in use by a Gear.\n");
  else if (outputJoint->getHPConnection())
    FFaMsg::list("ERROR: Output joint is already in use by a Gear.\n");
  else
  {
    FFaMsg::list("Creating Rack and pinion transmission.\n");
    FmRackPinion* tmpRP = new FmRackPinion();
    tmpRP->setParentAssembly(inputJoint->getCommonAncestor(outputJoint));
    tmpRP->connect(inputJoint,outputJoint);
    tmpRP->draw();
    return;
  }
  FFaMsg::list("       Could not create Rack and pinion.\n");
}


FmSensorBase* Fedem::createSensor(FmIsMeasuredBase* object)
{
  if (!object)
  {
    ListUI <<"ERROR: Unspecified measured object.\n";
    return NULL;
  }

  FmSensorBase* sens = object->getSimpleSensor(true);
  ListUI <<"Creating "<< sens->getUserDescription() <<".\n";

  return sens;
}


FmSensorBase* Fedem::createSensor(FmIsMeasuredBase* first,
                                  FmIsMeasuredBase* second)
{
  if (!first || !second)
  {
    ListUI <<"ERROR: Unspecified measured object(s).\n";
    return NULL;
  }
  else if (first == second)
  {
    ListUI <<"ERROR: Relative sensors should be used on different objects.\n"
           <<"       Could not create relative sensor.\n";
    return NULL;
  }

  FmSensorBase* sens = first->getRelativeSensor(second,true);
  ListUI <<"Creating "<< sens->getUserDescription() <<".\n";

  return sens;
}


void Fedem::createTire(FmRevJoint* joint)
{
  if (!joint) return;

  ListUI <<"Creating Tire on "<< joint->getIdString() <<".\n";

  FmTire* tire = new FmTire();
  tire->setParentAssembly(joint->getParentAssembly());
  tire->bearingJoint = joint;
  tire->connect();
  tire->draw();
}


FmCamJoint* Fedem::createCamJoint(FmTriad* follower, FmBase* subAssembly)
{
  if (!follower)
  {
    ListUI <<"ERROR: Unspecified follower triad for Cam joint.\n";
    return NULL;
  }
  else if (follower->isSlaveTriad(true))
  {
    ListUI <<"ERROR: "<< follower->getIdString() <<" is already dependent.\n"
           <<"       It can therefore not be used as follower in a Cam Joint.\n";
    return NULL;
  }

  ListUI <<"Creating Cam joint.\n";

  FmCamJoint* cam = new FmCamJoint();
  cam->connect();
  cam->setAsSlaveTriad(follower);
  cam->setParentAssembly(subAssembly);
  cam->draw();

  return cam;
}


static void createSMJoint(FmSMJointBase* joint,
                          const FaVec3& createPoint,
                          const FaVec3* zAxisDir = NULL)
{
  ListUI <<"Creating "<< joint->getUITypeName() <<".\n";

  FmSticker* sticker = new FmSticker(createPoint);
  FmTriad* triad1 = new FmTriad();
  FmTriad* triad2 = new FmTriad();

  if (zAxisDir)
    triad1->setLocalCS(FaMat34(getCreationMX(*zAxisDir),createPoint));
  else
    triad1->setLocalCS(FaMat34(createPoint));
  triad2->setLocalCS(triad1->getLocalCS());

  joint->setAsMasterTriad(triad1);
  joint->setAsSlaveTriad(triad2);
  joint->updateLocation();

  triad1->addSticker(sticker);

  triad1->connect();
  triad2->connect();
  joint->connect();

  triad1->draw();
  triad2->draw();
  sticker->draw();
  joint->draw();
}


/*!
  Both joint triads use \a zAxisDir as their z-axis
  and the "up" direction of the joint is in negative g-direction.
*/

void Fedem::createRevJoint(const FaVec3& createPos, const FaVec3& zAxisDir)
{
  createSMJoint(new FmRevJoint, createPos, &zAxisDir);
}


void Fedem::createBallJoint(const FaVec3& createPos)
{
  createSMJoint(new FmBallJoint, createPos);
}


void Fedem::createRigidJoint(const FaVec3& createPos)
{
  createSMJoint(new FmRigidJoint, createPos);
}


void Fedem::createJoint(int jType,
                        const FaVec3& firstPt,
                        const FaVec3& lastPt,
                        const FaVec3& yAxisDir)
{
  FmMMJointBase* joint = NULL;
  FaVec3 zAxis(lastPt - firstPt);
  if (zAxis.length() < FmDB::getPositionTolerance())
    FFaMsg::list("ERROR: The selected end points are co-located.\n");
  else if (jType == FmCylJoint::getClassTypeID())
    joint = new FmCylJoint();
  else if (jType == FmPrismJoint::getClassTypeID())
    joint = new FmPrismJoint();

  if (joint)
    ListUI <<"Creating "<< joint->getUITypeName() <<".\n";
  else
    return;

  Fm1DMaster* line = new FmStraightMaster();
  FmTriad* firstTriad = new FmTriad();
  FmTriad* lastTriad = new FmTriad();
  FmTriad* depTriad = new FmTriad();

  FmSticker* firstSticker = new FmSticker(firstPt);
  FmSticker* lastSticker = new FmSticker(lastPt);

  FaMat34 orientationMatrix(getCreationMX(zAxis,&yAxisDir),firstPt);
  firstTriad->setLocalCS(orientationMatrix);
  firstTriad->addSticker(firstSticker);

  lastTriad->setLocalCS(orientationMatrix);
  lastTriad->setTranslation(lastPt);
  lastTriad->addSticker(lastSticker);

  depTriad->setLocalCS(orientationMatrix);
  depTriad->setTranslation(0.5*(firstPt+lastPt));

  joint->setLocalCS(depTriad->getLocalCS());
  joint->setAsSlaveTriad(depTriad);
  joint->setMaster(line);
  line->addTriad(firstTriad);
  line->addTriad(lastTriad);

  line->connect();
  firstTriad->connect();
  lastTriad->connect();
  depTriad->connect();
  joint->connect();

  firstTriad->draw();
  lastTriad->draw();
  depTriad->draw();
  firstSticker->draw();
  lastSticker->draw();
  joint->draw();
}


FmJointBase* Fedem::createJoint(int jType, FmBase* first, FmBase* second,
                                const FaVec3* posJnt, FmBase* subAssembly)
{
  FmSMJointBase* joint = NULL;
  FmTriad* triad2 = dynamic_cast<FmTriad*>(second);
  if (!triad2) {
    ListUI <<"ERROR: Unspecified dependent triad.\n";
    return joint;
  }
  else if (triad2->isAttached(FmDB::getEarthLink())) {
    ListUI <<"ERROR: The dependent triad can not be attached to ground.\n";
    return joint;
  }

  FmTriad* triad1 = NULL;
  FmRefPlane* refPlane = NULL;
  if (!first) {
    std::vector<FmRefPlane*> refPlanes;
    FmDB::getAllRefPlanes(refPlanes);
    if (!refPlanes.empty()) refPlane = refPlanes.front();
  }
  else if (!(triad1 = dynamic_cast<FmTriad*>(first)))
    refPlane = dynamic_cast<FmRefPlane*>(first);

  if (triad1) {
    if (triad2->isAttached(triad1->getOwnerLink(0))) {
      ListUI <<"ERROR: The dependent triad can not be on the same part"
             <<" as the independent triad.\n";
      return joint;
    }
  }
  else if (!refPlane) {
    ListUI <<"ERROR: No reference planes in the model!\n";
    return joint;
  }
  else if (posJnt)
    triad1 = new FmTriad(*posJnt);
  else
    triad1 = new FmTriad();

  if (jType == FmFreeJoint::getClassTypeID())
    joint = new FmFreeJoint();
  else if (jType == FmBallJoint::getClassTypeID())
    joint = new FmBallJoint();
  else if (jType == FmRevJoint::getClassTypeID())
    joint = new FmRevJoint();
  else if (jType == FmRigidJoint::getClassTypeID())
    joint = new FmRigidJoint();
  else
  {
    ListUI <<"ERROR: Unknown point joint type "<< jType <<".\n";
    return joint;
  }
  ListUI <<"Creating "<< joint->getUITypeName() <<".\n";

  joint->setAsMasterTriad(triad1);
  joint->setAsSlaveTriad(triad2);
  if (refPlane)
    refPlane->attach(triad1);
  if (subAssembly)
    joint->setParentAssembly(subAssembly);
  else if (!refPlane)
    joint->setParentAssembly(triad1->getCommonAncestor(triad2));

  joint->updateLocation();
  joint->connect();

  triad1->draw();
  triad2->draw();
  joint->draw();

  return joint;
}


FmJointBase* Fedem::createJoint(int jType, FmBase* first, FmBase* last,
                                const FaVec3& yAxisDir, FmBase* slider,
                                FmBase* subAssembly, char addBetweens)
{
  FmMMJointBase* joint = NULL;
  FmTriad* triad1 = dynamic_cast<FmTriad*>(first);
  FmTriad* triad2 = dynamic_cast<FmTriad*>(last);
  if (!triad1 || !triad2) {
    ListUI <<"ERROR: Unspecified independent joint triad(s).\n";
    return joint;
  }
  else if (triad1->getOwnerPart(0) != triad2->getOwnerPart(0)) {
    ListUI <<"ERROR: The two triads must be on the same part.\n";
    return joint;
  }

  FmTriad* triad3  = dynamic_cast<FmTriad*>(slider);
  if (triad3 && triad3->isAttached(triad1->getOwnerPart(0))) {
    ListUI <<"ERROR: The dependent triad can not be on the same part"
           <<" as the independent triads of the joint.\n";
    return joint;
  }

  // First check if the two triads already are used by other line joints
  Fm1DMaster* line = NULL;
  std::vector<FmModelMemberBase*> allObjs;
  FmDB::getAllOfType(allObjs,FmStraightMaster::getClassTypeID());
  for (FmModelMemberBase* obj : allObjs)
    if ((line = dynamic_cast<Fm1DMaster*>(obj)))
    {
      if (triad1 == line->getFirstTriad() && triad2 == line->getLastTriad())
      {
        std::string msg("The selected triads match the end triads of ");
        FmMMJointBase* other = NULL;
        if (line->hasReferringObjs(other))
          msg += other->getIdString(true);
        else
          msg += line->getIdString(true);
        msg += ".\nDo you want the new joint to use the same line object?";
        if (FFaMsg::dialog(msg,FFaMsg::YES_NO))
          break;
      }
      line = NULL;
    }

  // Define the joint coordinate system
  FaMat33 orientation;
  if (line)
    orientation = triad1->getOrientation();
  else
  {
    FaVec3& xAxis = orientation[VX];
    FaVec3& yAxis = orientation[VY];
    FaVec3& zAxis = orientation[VZ];

    zAxis = triad2->getGlobalTranslation() - triad1->getGlobalTranslation();
    double zLen = zAxis.length();
    if (zLen >= FmDB::getPositionTolerance())
      zAxis /= zLen;
    else {
      ListUI <<"ERROR: The two triads are too close to each other.\n";
      return joint;
    }

    // Checking for valid vectors:
    if (yAxisDir.length() >= FmDB::getPositionTolerance())
      yAxis = yAxisDir;
    else
      yAxis = FaVec3(0.0,0.0,1.0);

    // First test: Check the cross product so that it not is zero
    xAxis = yAxis^zAxis;
    double xLen = xAxis.length();
    if (xLen*zLen >= FmDB::getPositionTolerance())
      xAxis /= xLen;
    else
    {
      yAxis = FaVec3(1.0,0.0,0.0);
      // Second test if the first one failed. This is the final test.
      xAxis = yAxis^zAxis;
      if (xAxis.length() < FmDB::getPositionTolerance())
        yAxis = FaVec3(0.0,1.0,0.0);
      xAxis = (yAxis^zAxis).normalize();
    }

    yAxis = (zAxis^xAxis).normalize();

    // Modify the orientation of the joint triads
    triad1->setOrientation(orientation);
    triad2->setOrientation(orientation);
  }

  if (jType == FmCylJoint::getClassTypeID())
    joint = new FmCylJoint();
  else if (jType == FmPrismJoint::getClassTypeID())
    joint = new FmPrismJoint();
  else
  {
    ListUI <<"ERROR: Unknown line joint type "<< jType <<".\n";
    return joint;
  }
  ListUI <<"Creating "<< joint->getUITypeName() <<".\n";

  if (subAssembly)
    joint->setParentAssembly(subAssembly);
  else if (line && triad3)
    joint->setParentAssembly(line->getCommonAncestor(triad3));
  else if (line)
    joint->setParentAssembly(line->getParentAssembly());
  else
    joint->setParentAssembly(triad1->getCommonAncestor(triad2));

  if (!triad3) {
    triad3 = new FmTriad(0.5*(triad1->getGlobalTranslation() +
                              triad2->getGlobalTranslation()));
    triad3->setParentAssembly(joint->getParentAssembly());
  }
  triad3->setOrientation(orientation);
  joint->setLocalCS(triad3->getLocalCS());
  joint->setAsSlaveTriad(triad3);

  if (!line)
  {
    line = new FmStraightMaster();
    line->setParentAssembly(joint->getParentAssembly());
    line->addTriad(triad1);
    line->addTriad(triad2);
    line->connect();
  }

  joint->setMaster(line);
  triad3->connect();
  joint->connect();

  triad1->draw();
  triad2->draw();
  triad3->draw();
  joint->draw();

  // Check if the part connected to the line have other triads along the line
  // between the two end triads, and offer to add those as joint triads as well
  FmPart* part = triad1->getOwnerPart(0);
  if (!part) return joint;

  FaVec3 fstPos = triad1->getGlobalTranslation();
  FaVec3 linVec = triad2->getGlobalTranslation() - fstPos;
  double parTol = FmDB::getParallelTolerance();

  std::vector<FmTriad*>::iterator it;
  std::vector<FmTriad*> triads;
  part->getTriads(triads);
  for (it = triads.begin(); it != triads.end();)
    if (*it == triad1 || *it == triad2)
      triads.erase(it);
    else if (linVec.isParallell((*it)->getGlobalTranslation() - fstPos, parTol))
      ++it;
    else
      triads.erase(it);

  if (triads.empty() || !addBetweens)
    return joint;

  FFaNumStr msg("There are %d triads on the line between the ",triads.size());
  msg += "two end triads.\nSee Output List view for details."
    " Do you want to add these as joint triads also?";
  ListUI <<"\n"<< msg.substr(0,msg.find_first_of('.')) <<":";
  for (FmTriad* triad : triads)
    ListUI <<"\n\t"<< triad->getIdString(true)
           <<": "<< triad->getGlobalTranslation();
  ListUI <<"\n";

  bool ok = true;
  if (addBetweens == 1 || FFaMsg::dialog(msg,FFaMsg::YES_NO))
    for (FmTriad* triad : triads)
      ok &= line->addTriadOnPoint(triad->getGlobalTranslation());

  return ok ? joint : NULL;
}


static FmBase* getCommonParent(const std::vector<FmTriad*>& triads)
{
  if (triads.empty())
    return NULL;

  // Check if the triads belong to the same sub-assmebly
  FmBase* parent = triads.front()->getParentAssembly();
  for (FmTriad* triad : triads)
    if (triad->getParentAssembly() != parent)
      return NULL;

  return parent;
}


FmBeam* Fedem::createBeam(FmTriad* tr1, FmTriad* tr2, FmBase* subAssembly)
{
  if (!tr1 || !tr2)
  {
    ListUI <<"ERROR: Unspecified beam triad(s).\n";
    return NULL;
  }

  ListUI <<"Creating Beam element.\n";

  FmBeam* b = new FmBeam();
  b->setParentAssembly(subAssembly ? subAssembly : tr1->getCommonAncestor(tr2));
  b->connect(tr1,tr2);
  b->draw();
  tr1->draw();
  tr2->draw();

  return b;
}


FmModelMemberBase* Fedem::createBeams(const std::vector<FmTriad*>& triads,
                                      FmBase* subAssembly)
{
  if (triads.size() < 2)
    return NULL;
  else if (triads.size() == 2)
    return Fedem::createBeam(triads.front(),triads.back(),subAssembly);

  if (!subAssembly)
    subAssembly = getCommonParent(triads);

  ListUI <<"Creating Beamstring.\n";

  FmBeam* beam = NULL;
  for (size_t i = 1; i < triads.size(); i++)
  {
    beam = new FmBeam();
    beam->setParentAssembly(subAssembly);
    beam->connect(triads[i-1],triads[i]);
    beam->draw();
    triads[i-1]->draw();
  }
  triads.back()->draw();

  return beam;
}


FmModelMemberBase* Fedem::createPart(const std::vector<FmTriad*>& triads,
                                     FmBase* subAssembly)
{
  if (triads.empty())
    return NULL;

  if (!subAssembly)
    subAssembly = getCommonParent(triads);

  ListUI <<"Creating Generic part.\n";

  FmPart* part = new FmPart();
  part->setParentAssembly(subAssembly);
  part->connect();
  part->useGenericProperties.setValue(true);

  // Connect the triads
  FaVec3 cg;
  for (FmTriad* triad : triads)
  {
    triad->connect(part);
    cg += triad->getGlobalTranslation();
  }
  cg /= triads.size();

  // Must refer the CoG position to origin of parent assembly
  FmAssemblyBase* pAss = dynamic_cast<FmAssemblyBase*>(subAssembly);
  part->setPositionCG(pAss ? pAss->toLocal(cg) : cg);
  part->draw();

  return part;
}


FmModelMemberBase* Fedem::createUserElm(const std::vector<FmTriad*>& triads,
                                        FmBase* subAssembly)
{
  char eName[64];
  int eTypes[10];
  int nTypes = FiUserElmPlugin::instance()->getElementTypes(10,eTypes);
  int nTriad = triads.size();
  for (int i = 0; i < nTypes; i++)
    if (FiUserElmPlugin::instance()->getTypeName(eTypes[i],64,eName) == nTriad)
      return Fedem::createUserElm(eTypes[i],eName,triads,nTriad,subAssembly);

  ListUI <<" *** Error: No "<< nTriad <<"-noded user-defined element type.\n";
  return NULL;
}


/*!
  If \a nelnod equals 1, a user-defined element is created for each of the
  given \a triads. If \a nelnod equals 2, a chain of \a nelnod-1 elements
  is created connecting the given triads. If \a nelnod is larger than 2
  (and less than or equal to the size of \a triads), then one user-defined
  elements is created connected to the first \a nelnod triads.
*/

FmModelMemberBase* Fedem::createUserElm(int elmType, const char* typeName,
                                        const std::vector<FmTriad*>& triads,
                                        size_t nelnod, FmBase* subAssembly)
{
  FmUserDefinedElement* uelm = NULL;

  ListUI <<"Creating user-defined element(s) \""<< typeName <<"\".\n";

  for (size_t i = 0; i+nelnod-1 < triads.size(); i++)
  {
    uelm = new FmUserDefinedElement();
    uelm->setParentAssembly(subAssembly);
    uelm->connect();
    uelm->init(elmType,typeName,
               std::vector<FmTriad*>(triads.begin()+i,
                                     triads.begin()+i+nelnod));
    uelm->draw();
    triads[i]->draw();
    if (nelnod > 2) break;
  }

  if (nelnod == 2)
    triads.back()->draw();
  else if (nelnod > 2)
    for (size_t i = 1; i < nelnod; i++)
      triads[i]->draw();

  return uelm;
}


/*!
  This function creates a catenary curve of the specified length between the
  two triads consisting of \a nSegment 2-noded elements of the specified type.
*/

bool Fedem::createMooringLine(FmTriad* tr1, FmTriad* tr2,
                              double Length, int nSegments, int elmType,
                              FmBase* subAssembly)
{
  if (nSegments < 2 || !tr1 || !tr2)
    return false; // Need at least two segments

  FaVec3 X1 = tr1->getGlobalTranslation();
  FaVec3 X2 = tr2->getGlobalTranslation();
  if (Length < (X2-X1).length())
  {
    FFaNumStr msg1("The specified length %g is less than ",Length);
    FFaNumStr msg2("the distance %g betweeen the end triads.",(X2-X1).length());
    FFaMsg::dialog("Too short!\n"+msg1+msg2,FFaMsg::ERROR);
    return false;
  }

  // Find local coordinate system for calculation of mooring line shape
  FaMat33 Tlg;
  FaVec3 Zaxis = -FmDB::getGrav();
  if (Zaxis.isZero())
    Zaxis = Tlg[2]; // No gravity, assume global Z-direction instead
  else
    Tlg[2] = Zaxis.normalize(); // Local Z-axis
  Tlg[1] = Zaxis ^ (X2-X1); // Local Y-axis = (Z-axis) x (X2-X1)
  Tlg[1].normalize();
  Tlg[0] = Tlg[1] ^ Tlg[2]; // Local X-axis = (Y-axis) x (Z-axis)

  // Calculate the intermediate triad positions
  FaVec3 dX = Tlg.transpose()*(X2-X1);
  ListUI <<"Calculating chain shape dX="<< dX[0] <<" dZ="<< dX[2]
         <<" Length="<< Length <<" ("<< nSegments <<" segments)\n";
  std::vector<double> X(nSegments+1), Z(nSegments+1);
  if (getCableShape(nSegments,Length,dX[0],-dX[2],X,Z) < 0)
  {
    FFaMsg::dialog("Failed to calculate mooring line shape.",FFaMsg::ERROR);
    return false;
  }

  FmBase* parent = subAssembly ? subAssembly : tr1->getCommonAncestor(tr2);

  char typeName[64];
  if (elmType > 0) // Determine which element type to use
    if (FiUserElmPlugin::instance()->getTypeName(elmType,64,typeName) != 2)
      elmType = -1; // Not a 2-noded element, using Generic part instead

  // Lambda function for creating a generic part
  std::function<void(FmTriad*,FmTriad*)> newPart = [parent](FmTriad* t1, FmTriad* t2)
  {
    FmPart* part = new FmPart();
    part->setParentAssembly(parent);
    part->connect();
    part->useGenericProperties.setValue(true);
    t1->connect(part);
    t2->connect(part);
    FaVec3 cg = 0.5*(t1->getGlobalTranslation() + t2->getGlobalTranslation());
    // Must refer the CoG position to origin of parent assembly
    FmAssemblyBase* pAss = dynamic_cast<FmAssemblyBase*>(parent);
    part->setPositionCG(pAss ? pAss->toLocal(cg) : cg);
    part->draw();
  };

  // Lambda function for creating a beam element
  std::function<void(FmTriad*,FmTriad*)> newBeam = [parent](FmTriad* t1, FmTriad* t2)
  {
    FmBeam* beam = new FmBeam();
    beam->setParentAssembly(parent);
    beam->connect(t1,t2);
    beam->draw();
  };

  // Lambda function for creating a user-defined element
  std::function<void(FmTriad*,FmTriad*)> newElem = [parent,elmType,&typeName](FmTriad* t1, FmTriad* t2)
  {
    FmUserDefinedElement* uelm = new FmUserDefinedElement();
    uelm->setParentAssembly(parent);
    uelm->connect();
    uelm->init(elmType,typeName,{t1,t2});
    uelm->draw();
  };

  // Generate triads and elements for the mooring line segments
  for (int i = 1; i < nSegments; i++) {
    ListUI <<"Creating Mooring line triad at X="<< X[i] <<" Z="<< Z[i];
    FmTriad* tr3 = new FmTriad(X1 + Tlg[0]*X[i] + Tlg[2]*Z[i]);
    tr3->setParentAssembly(parent);
    tr3->connect();
    ListUI <<" ==> "<< tr3->getGlobalTranslation() <<"\n";
    if (elmType < 0)
      newPart(tr1,tr3);
    else if (elmType == 0)
      newBeam(tr1,tr3);
    else
      newElem(tr1,tr3);
    tr1->draw();
    tr1 = tr3;
  }

  // Generate the last element
  if (elmType < 0)
    newPart(tr1,tr2);
  else if (elmType == 0)
    newBeam(tr1,tr2);
  else
    newElem(tr1,tr2);
  tr1->draw();
  tr2->draw();
  return true;
}


FmSubAssembly* Fedem::createSubAssembly(const std::vector<FmModelMemberBase*>& objs,
                                        FmBase* subAssembly)
{
  FmStructAssembly* subAss = new FmStructAssembly();
  subAss->setParentAssembly(subAssembly);
  subAss->connect();
  ListUI <<" ==> Creating "<< subAss->getIdString();

  // Move given objects to the new subassembly
  for (FmModelMemberBase* obj : objs)
    if (obj && obj->moveTo(subAss))
    {
      ListUI <<"\n  -> Moving "<< obj->getIdString(true)
             <<" to "<< subAss->getIdString();
      if (obj->isOfType(FmPart::getClassTypeID()))
      {
        // Also move all element groups of the Part
        std::vector<FmElementGroupProxy*> groups;
        static_cast<FmPart*>(obj)->getElementGroups(groups);
        for (FmElementGroupProxy* group : groups)
          if (group->moveTo(subAss))
            ListUI <<"\n  -> Moving "<< group->getIdString(true)
                   <<" to "<< subAss->getIdString();
      }
      else if (obj->isOfType(FmEngine::getClassTypeID()))
      {
        // Also move the math function associated with the Engine
        FmEngine* engine = static_cast<FmEngine*>(obj);
        FmMathFuncBase* func = engine->getFunction();
        if (func && func->moveTo(subAss))
          ListUI <<"\n  -> Moving "<< func->getIdString(true)
                 <<" to "<< subAss->getIdString();

        // Also move the sensor(s) associated with the Engine
        FmSensorBase* sensor = NULL;
        size_t nArg = engine->getNoArgs();
        for (size_t i = 0; i < nArg; i++)
          if ((sensor = engine->getSensor(i)))
            if (sensor->moveTo(subAss))
              ListUI <<"\n  -> Moving "<< sensor->getIdString(true)
                     <<" to "<< subAss->getIdString();
      }
    }

  ListUI <<"\n";
  return subAss;
}
