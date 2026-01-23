# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Library setup

set ( LIB_ID vpmDB )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )


# Check if the Qt package is available (not needed on Linux)
set ( NO_QT6 FALSE )
if ( "${USE_QT}" STREQUAL "Qt6" )
  find_package ( Qt6 COMPONENTS Core )
  if ( NOT Qt6_FOUND )
    set ( NO_QT6 TRUE )
  endif ( NOT Qt6_FOUND )
endif ( "${USE_QT}" STREQUAL "Qt6" )
set ( NO_QT5 FALSE )
if ( "${USE_QT}" STREQUAL "Qt5" OR NO_QT6 )
  find_package ( Qt5 COMPONENTS Core )
  if ( NOT Qt5_FOUND )
    set ( NO_QT5 TRUE )
  endif ( NOT Qt5_FOUND )
endif ( "${USE_QT}" STREQUAL "Qt5" OR NO_QT6 )
if ( "${USE_QT}" STREQUAL "Qt4" OR NO_QT5 )
  find_package ( Qt4 COMPONENTS QtCore )
endif ( "${USE_QT}" STREQUAL "Qt4" OR NO_QT5 )

if ( Qt6_FOUND )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_QT=6" )
elseif ( Qt5_FOUND )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_QT=5" )
elseif ( Qt4_FOUND )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_QT=4" )
elseif ( LINUX )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_DIRENT" )
endif ( Qt6_FOUND )

if ( USE_REFCOUNT )
  string ( APPEND CMAKE_CXX_FLAGS " -DFFL_REFCOUNT" )
endif ( USE_REFCOUNT )

if ( USE_MEMPOOL )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_MEMPOOL" )
endif ( USE_MEMPOOL )

if ( USE_PROFILER )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_PROFILER" )
endif ( USE_PROFILER )

if ( USE_VISUALS )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_VISUALS" )
endif ( USE_VISUALS )

if ( USE_CONNECTORS )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_CONNECTORS" )
endif ( USE_CONNECTORS )


## Files with header and source with same name
set ( COMPONENT_FILE_LIST  Fm1DMaster FmAnalysis FmAnimation FmArcSegmentMaster
                           FmAxialDamper FmAxialSpring FmBallJoint FmBase
                           FmBearingFriction FmCamFriction FmCamJoint
                           FmcElements FmcInput FmControlAdmin FmcOutput
                           FmCreate FmCtrlElementBase FmCtrlInputElementBase
                           FmCtrlLine FmCtrlOutputElementBase FmCurveSet
                           FmCylJoint FmDamperBase FmDB FmDofLoad FmDofMotion
                           FmElementGroupProxy FmEngine
                           FmfComplSinus FmfConstant FmfDelayedComplSinus
                           FmfDeviceFunction FmfDiracPuls FmfExternalFunction
                           FmFileReference FmfLimRamp FmfLinVar FmfLinVelVar
                           FmfMathExpr FmfMultiArgBase FmfMultiVarBase
                           FmFppOptions FmfRamp FmFreeJoint FmFrictionBase
                           FmfScale FmfSinusoidal FmfSmoothTraj FmfSpline
                           FmfSquarePuls FmfStep FmFuncAdmin FmFuncTree
                           FmfUserDefined FmfWaveSinus FmGageOptions
                           FmGear FmGenericDBObject FmGlobalViewSettings
                           FmGraph FmHasDOFsBase FmHPBase
                           FmIsControlledBase FmIsMeasuredBase
                           FmIsPlottedBase FmIsPositionedBase FmIsRenderedBase
                           FmJacket FmJointBase FmJointDamper FmJointMotion
                           FmJointSpring FmLink FmLoad FmMathFuncBase
                           FmMechanism FmMMJointBase FmModelFileQuery
                           FmModelMemberBase FmModesOptions FmParamObjectBase
                           FmPipeStringDataExporter FmPipeSurface
                           FmPrismaticFriction FmPrismJoint
                           FmRackPinion FmRefPlane FmRelativeSensor
                           FmResultBase FmResultStatusData FmRevJoint
                           FmRigidJoint FmRingStart FmRiser FmRoad
                           FmRotFriction FmSeaState FmSensorBase FmSimpleSensor
                           FmSimulationEvent FmSimulationModelBase
                           FmSMJointBase FmSoilPile FmSolverConvergence
                           FmSolverParser FmSpringBase FmSpringChar
                           FmSticker FmStraightMaster FmStrainRosette
                           FmSubAssembly FmTimeSensor FmTire FmTransFriction
                           FmTriad FmTurbine FmVesselMotion FmAirState
                           FmAssemblyBase FmStructAssembly FmBladeProperty
                           FmBeamProperty FmMaterialProperty
                           FmBeam FmPart FmUserDefinedElement
                           FmFileSys FmModelLoader FmSolverInput FmThreshold
                           FmModelExpOptions
)
if ( USE_EXT_CTRLSYS )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_EXTCTRL" )
  list ( APPEND COMPONENT_FILE_LIST FmExternalCtrlSys )
endif ( USE_EXT_CTRLSYS )

## Pure header files, i.e., header files without a corresponding source file
set ( HPP_FILE_LIST FmAllControlHeaders FmAllFunctionHeaders
                    FmArray FmColor FmModelMemberConnector FmQuery FmVTFType
)


foreach ( FILE ${COMPONENT_FILE_LIST} )
  list ( APPEND CPP_FILES ${FILE}.C )
  list ( APPEND HPP_FILES ${FILE}.H )
endforeach ( FILE ${COMPONENT_FILE_LIST} )

foreach ( FILE ${HPP_FILE_LIST} )
  list ( APPEND HPP_FILES ${FILE}.H )
endforeach ( FILE ${HPP_FILE_LIST} )


set ( FT_COMMON_LIBRARIES Admin FFlIOAdaptors FFlFEParts FFlLib
                          FFaAlgebra FFaContainers FFaDefinitions
                          FFaDynCalls FFaOS FFaTypeCheck )
if ( USE_CONNECTORS )
  list ( INSERT FT_COMMON_LIBRARIES 9 FFaGeometry )
endif ( USE_CONNECTORS )
if ( USE_CMDLINEARG )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_CMDLINEARG" )
  list ( INSERT FT_COMMON_LIBRARIES 5 FFaCmdLineArg )
endif ( USE_CMDLINEARG )

set ( FT_KERNEL_LIBRARIES FiDeviceFunctions FiUserElmPlugin
                          FFaFunctionLib FFaMathExpr )
if ( NO_FATIGUE )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_NO_FATIGUE" )
else ( NO_FATIGUE )
  list ( INSERT FT_KERNEL_LIBRARIES 2 FFpFatigue )
endif ( NO_FATIGUE )
set ( DEPENDENCY_LIST ${FT_KERNEL_LIBRARIES} ${FT_COMMON_LIBRARIES} )
if ( Coin_library )
  list ( INSERT DEPENDENCY_LIST 0 vpmDisplay )
  if ( DEFINED ENV{COIN_ROOT} )
    include_directories ( "$ENV{COIN_ROOT}/include" )
  endif ( DEFINED ENV{COIN_ROOT} )
endif ( Coin_library )
if ( TARGET ChainShape )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAVE_CHAINSHAPE" )
  list ( INSERT DEPENDENCY_LIST 1 ChainShape )
endif ( TARGET ChainShape )


message ( STATUS "Building library ${LIB_ID}" )
if ( Qt6_FOUND )
  list ( APPEND DEPENDENCY_LIST Qt6::Core )
  message ( STATUS "Configuring with Qt6" )
elseif ( Qt5_FOUND )
  list ( APPEND DEPENDENCY_LIST Qt5::Core )
  message ( STATUS "Configuring with Qt5" )
elseif ( Qt4_FOUND )
  list ( APPEND DEPENDENCY_LIST Qt4::QtCore )
  message ( STATUS "Configuring with Qt4" )
elseif ( NOT "${USE_QT}" STREQUAL "OFF" )
  message ( STATUS "Configuring without Qt since no Qt package is found" )
endif ( Qt6_FOUND )

add_library ( ${LIB_ID} ${CPP_FILES} ${HPP_FILES} )
target_link_libraries ( ${LIB_ID} ${DEPENDENCY_LIST} )
