// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmCreate.H
  \brief Global functions for creating positioned mechanism objects.
*/

#ifndef FM_CREATE_H
#define FM_CREATE_H

#include <cstddef>
#include <vector>

class FmTriad;
class FmRevJoint;
class FmCamJoint;
class FmJointBase;
class FmAxialSpring;
class FmAxialDamper;
class FmIsMeasuredBase;
class FmSensorBase;
class FmLoad;
class FmBeam;
class FmBase;
class FmModelMemberBase;
class FmSubAssembly;
class FaVec3;


namespace Fedem //! Fedem model database utilities
{
  //! \brief Creates a triad.
  //! \param[in] createPos Global position of the triad
  //! \param[in] onObject If this is a Part, the triad is attached to it.
  //! If it is a sub-assembly, that sub-assembly is set as the triad's parent.
  FmTriad* createTriad(const FaVec3& createPos, FmBase* onObject = NULL);

  //! \brief Creates a revolute joint object.
  //! \param[in] createPos Global joint position
  //! \param[in] zAxisDir The axis of revolution
  void createRevJoint(const FaVec3& createPos, const FaVec3& zAxisDir);
  //! \brief Creates a ball joint object.
  //! \param[in] createPos Global joint position
  void createBallJoint(const FaVec3& createPos);
  //! \brief Creates a rigid joint object.
  //! \param[in] createPos Global joint position
  void createRigidJoint(const FaVec3& createPos);
  //! \brief Creates a free joint object.
  //! \param[in] posJnt Global position of joint and independent triad
  //! \param[in] posDep Global position of dependent triad
  //! \param[in] zAxisDir Common Z-axis direction for the joint triads
  void createFreeJoint(const FaVec3& posJnt, const FaVec3& posDep,
                       const FaVec3* zAxisDir = NULL);
  //! \brief Creates a line joint object.
  //! \param[in] jType Type of joint to create (class type ID)
  //! \param[in] firstPoint Start point of line
  //! \param[in] lastPoint End point of line
  //! \param[in] yAxisDir Y-axis direction for the independent triads
  void createJoint(int jType,
                   const FaVec3& firstPoint, const FaVec3& lastPoint,
                   const FaVec3& yAxisDir);

  //! \brief Creates a point joint connected to two triads.
  //! \param[in] jType Type of joint to create (class type ID)
  //! \param[in] first The independent triad or reference plane
  //! \param[in] second The dependent triad
  //! \param[in] posJnt Global position of independent triad, if new
  //! \param[in] subAssembly Parent assembly of the joint
  FmJointBase* createJoint(int jType, FmBase* first, FmBase* second,
                           const FaVec3* posJnt = NULL,
                           FmBase* subAssembly = NULL);

  //! \brief Creates a line joint between two triads.
  //! \param[in] jType Type of joint to create (class type ID)
  //! \param[in] first Start triad of joint line
  //! \param[in] last End triad of joint line
  //! \param[in] yAxisDir Y-axis direction for the independent triads
  //! \param[in] slider Dependent triad of the joint
  //! \param[in] subAssembly Parent assembly of the joint
  //! \param[in] addBetweens Flag for adding other triads found along the line
  //! - 0 : Never do it
  //! - 1 : Always do it
  //! - 2 : Ask user if we should do it or not
  FmJointBase* createJoint(int jType, FmBase* first, FmBase* last,
                           const FaVec3& yAxisDir, FmBase* slider = NULL,
                           FmBase* subAssembly = NULL, char addBetweens = 2);

  //! \brief Creates a cam joint.
  //! \param[in] follower Follower triad
  //! \param[in] subAssembly Parent assembly of the cam joint
  FmCamJoint* createCamJoint(FmTriad* follower, FmBase* subAssembly = NULL);

  //! \brief Creates an axial spring between two triads.
  //! \param[in] first First triad the spring is acting between
  //! \param[in] second Second triad the spring is acting between
  //! \param[in] subAssembly Parent assembly of the spring
  FmAxialSpring* createAxialSpring(FmTriad* first, FmTriad* second,
                                   FmBase* subAssembly = NULL);
  //! \brief Creates an axial spring between two points on two objects
  //! \param[in] firstPt First point the spring is acting between
  //! \param[in] secondPt Second point the spring is acting between
  //! \param[in] first First object the spring is acting between
  //! \param[in] second Second object the spring is acting between
  void createAxialSpring(const FaVec3& firstPt, const FaVec3& secondPt,
                         FmBase* first, FmBase* second);

  //! \brief Creates an axial damper between two triads.
  //! \param[in] first First triad the damper is acting between
  //! \param[in] second Second triad the damper is acting between
  //! \param[in] subAssembly Parent assembly of the damper
  FmAxialDamper* createAxialDamper(FmTriad* first, FmTriad* second,
                                   FmBase* subAssembly = NULL);
  //! \brief Creates an axial damper between two points on two objects
  //! \param[in] firstPt First point the damper is acting between
  //! \param[in] secondPt Second point the damper is acting between
  //! \param[in] first First object the damper is acting between
  //! \param[in] second Second object the damper is acting between
  void createAxialDamper(const FaVec3& firstPt, const FaVec3& secondPt,
                         FmBase* first, FmBase* second);

  //! \brief Creates an external load object.
  //! \param[in] lType Type of load (0=force, 1=torque)
  //! \param[in] createPos Load attack point
  //! \param[in] globalDir Load direction vector
  //! \param[in] onObject Object the load is acting on
  //! \param[in] subAssembly Parent assembly of the load object
  FmLoad* createLoad(int lType, const FaVec3& createPos, FaVec3 globalDir,
                     FmBase* onObject = NULL, FmBase* subAssembly = NULL);

  //! \brief Creates a sticker on an object.
  //! \param[in] object Object to put the sticker on
  //! \param[in] createPoint Location of sticker
  void createSticker(FmBase* object, const FaVec3& createPoint);
  //! \brief Creates a sticker on an object.
  //! \param[in] createPoint Location of sticker
  //! \param[in] onObject Object to put the sticker on
  void createSticker(const FaVec3& createPoint, FmBase* onObject);

  //! \brief Creates a gear object.
  //! \param[in] first Input joint
  //! \param[in] second Output joint
  void createGear(FmBase* first, FmBase* second);
  //! \brief Creates a rack & pinion object.
  //! \param[in] first Input joint
  //! \param[in] second Output joint
  void createRackPinion(FmBase* first, FmBase* second);

  //! \brief Creates a simple sensor on an object.
  //! \param[in] object The object to measure
  FmSensorBase* createSensor(FmIsMeasuredBase* object);
  //! \brief Creates a relative sensor between two objects.
  //! \param[in] first The first object to measure between
  //! \param[in] second The second object to measure between
  FmSensorBase* createSensor(FmIsMeasuredBase* first, FmIsMeasuredBase* second);

  //! \brief Creates a tire object.
  //! \param[in] joint The revolute joint representing the wheel bearing
  void createTire(FmRevJoint* joint);

  //! \brief Creates a single beam element.
  //! \param[in] tr1 First end triad
  //! \param[in] tr2 Second end triad
  //! \param[in] subAssembly Parent assembly of the beam element
  FmBeam* createBeam(FmTriad* tr1, FmTriad* tr2, FmBase* subAssembly = NULL);

  //! \brief Creates a beam string.
  //! \param[in] triads List of triads connected to the created beam elements
  //! \param[in] subAssembly Parent assembly of the created beam elements
  FmModelMemberBase* createBeams(const std::vector<FmTriad*>& triads,
                                 FmBase* subAssembly = NULL);

  //! \brief Creates a generic part.
  //! \param[in] triads List of triads connected to the created part
  //! \param[in] subAssembly Parent assembly of the created part
  FmModelMemberBase* createPart(const std::vector<FmTriad*>& triads,
                                FmBase* subAssembly = NULL);

  //! \brief Creates a user-defined element.
  //! \param[in] triads List of triads connected to the created element
  //! \param[in] subAssembly Parent assembly of the created element
  FmModelMemberBase* createUserElm(const std::vector<FmTriad*>& triads,
                                   FmBase* subAssembly = NULL);

  //! \brief Creates user-defined elements.
  //! \param[in] elmType Element type ID
  //! \param[in] typeName Element type name
  //! \param[in] triads List of triads connected to the created element(s)
  //! \param[in] nelnod Number of element nodes for the created element(s)
  //! \param[in] subAssembly Parent assembly of the created element(s)
  FmModelMemberBase* createUserElm(int elmType, const char* typeName,
                                   const std::vector<FmTriad*>& triads,
                                   size_t nelnod, FmBase* subAssembly = NULL);

  //! \brief Creates a mooring line (macro object) between two triads.
  //! \param[in] tr1 Start triad of mooring line
  //! \param[in] tr2 Stop triad of mooring line
  //! \param[in] Length Total length of mooring line
  //! \param[in] nSegments Number of elements to divide the mooring line into
  //! \param[in] elmType The element type to use
  //! \param[in] subAssembly Parent assembly of the mooring line
  bool createMooringLine(FmTriad* tr1, FmTriad* tr2,
                         double Length, int nSegment, int elmType,
                         FmBase* subAssembly = NULL);

  //! \brief Creates sub-assembly containing the specified objects.
  //! \param[in] objs Objects to move to the new sub-assembly
  //! \param[in] subAssembly Parent assembly of the new sub-assembly
  FmSubAssembly* createSubAssembly(const std::vector<FmModelMemberBase*>& objs,
                                   FmBase* subAssembly = NULL);
}

#endif
