# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

################################################################################
# This is the top-level cmake project file for the Fedem model database library.
################################################################################

cmake_minimum_required ( VERSION 2.8...3.10 )

# Project setup

set ( APPLICATION_ID fedemDB )
set ( DOMAIN_ID FEDEM )
set ( PACKAGE_ID MDB )
set ( LIB_ID FedemDB )
set ( LIB_ID_LIST Admin FFaLib FFlLib FFaFunctionLib FFaMathExpr
                  FiDeviceFunctions FiUserElmPlugin )

set ( USE_VISUALS ON ) # Visualization attribute classes must be included
set ( USE_CONNECTORS ON ) # Connector geometry items must be included
option ( USE_FORTRAN "Build with Fortran code included" OFF )
option ( USE_CHSHAPE "Use ChainShape library for mooring line calculation" OFF )
set ( USE_QT "OFF" CACHE STRING "Use Qt file system handling" )
set_property ( CACHE USE_QT PROPERTY STRINGS Qt6 Qt5 Qt4 OFF )
option ( USE_MEMPOOL "Use memory pool for heap allocation in FE library" OFF )
option ( USE_PROFILER "Use CPU and Memory profiler" OFF )
mark_as_advanced ( USE_FORTRAN USE_CHSHAPE USE_QT USE_MEMPOOL USE_PROFILER )

if ( USE_FORTRAN)
  project ( ${APPLICATION_ID} CXX C Fortran )
else ( USE_FORTRAN)
  project ( ${APPLICATION_ID} CXX C )
endif ( USE_FORTRAN)
message ( STATUS "Generating build project for ${PROJECT_SOURCE_DIR}" )
find_path ( _MODULES FedemConfig.cmake
            PATHS $ENV{CMAKE_MODULES}
           "${PROJECT_SOURCE_DIR}/fedem-foundation/cmake/Modules/"
          )
if ( _MODULES )
  message ( STATUS "NOTE : Using ${_MODULES}" )
  list ( APPEND CMAKE_MODULE_PATH ${_MODULES} )
else ( _MODULES )
  message ( STATUS "ERROR : Missing path to FedemConfig.cmake" )
  message ( FATAL_ERROR "Set environment variable CMAKE_MODULES and try again" )
endif ( _MODULES )
set ( _MODULES "${PROJECT_SOURCE_DIR}/cmake/Modules" )
if ( NOT "${USE_QT}" STREQUAL "OFF" AND IS_DIRECTORY ${_MODULES} )
  message ( STATUS "NOTE : Using ${_MODULES}" )
  list ( APPEND CMAKE_MODULE_PATH ${_MODULES} )
endif ( NOT "${USE_QT}" STREQUAL "OFF" AND IS_DIRECTORY ${_MODULES} )
unset ( _MODULES CACHE )

include ( CheckPFUnit )
include ( FedemConfig )

if ( BUILD_TESTS )

# Enable unit- and regression testing

  enable_testing ()
  set ( CTEST_OPTIONS --force-new-ctest-process --output-on-failure -O CTest.txt )
  if ( CMAKE_CONFIGURATION_TYPES )
    list ( APPEND CTEST_OPTIONS --build-config \"$<CONFIGURATION>\" )
  endif ( CMAKE_CONFIGURATION_TYPES )
  add_custom_target ( check COMMAND ${CMAKE_CTEST_COMMAND} ${CTEST_OPTIONS} )

  include ( GTest ) # Using the google test framework for C++ unit tests
  if ( USE_FORTRAN )
    include ( pFUnit ) # Using the pFUnit test framework for Fortran unit tests
  endif ( USE_FORTRAN )

endif ( BUILD_TESTS )

# Library setup

# Compile the FFlLib classes in the FTD namespace
string ( APPEND CMAKE_CXX_FLAGS " -DFF_NAMESPACE=FTD" )
# Compile the FFlLib classes with reference coutning to check for unused items
set ( USE_REFCOUNT true )

# Include dependent libraries from fedem-foundation
include_directories ( "${PROJECT_SOURCE_DIR}/fedem-foundation/src" )
foreach ( FOLDER ${LIB_ID_LIST} )
  add_subdirectory ( fedem-foundation/src/${FOLDER} "${CMAKE_CURRENT_BINARY_DIR}/${FOLDER}" )
endforeach ( FOLDER ${LIB_ID_LIST} )
if ( USE_FORTRAN AND USE_CHSHAPE )
  add_subdirectory ( chainShape )
endif ( USE_FORTRAN AND USE_CHSHAPE )

# Exclude some targets from sub-projects that are not needed here
set_target_properties ( FFaOperation
                        PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1 )

# Include the main library
set ( NO_FATIGUE true )
string ( APPEND CMAKE_CXX_FLAGS_DEBUG " -DFM_DEBUG=${FT_DEBUG}" )
include_directories ( "${CMAKE_CURRENT_SOURCE_DIR}" )
add_subdirectory ( vpmDB )
if ( GTest_FOUND )
  add_subdirectory ( assemblyCreators )
endif ( GTest_FOUND )

# Build and install the shared library
message ( STATUS "Building shared library ${LIB_ID}" )
add_library ( ${LIB_ID} SHARED FedemDB.C
              fedem-foundation/src/FFaLib/FFaCmdLineArg/FFaOptionFileCreator.C )
target_link_libraries ( ${LIB_ID} vpmDB )

install ( TARGETS ${LIB_ID}
          RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
          LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )


if ( BUILD_TESTS )

  # Perform the unit- and regression tests

  if ( GTest_FOUND )
    add_subdirectory ( test )
  endif ( GTest_FOUND )

  if ( TARGET ChainShape )
    add_subdirectory ( test/chainShape )
  endif ( TARGET ChainShape )

  # If the fedem-solvers repository has been cloned in a folder parallel to
  # either this one, or the parent folder of this, then set PYTHON_DIR pointing
  # to the root of the fedempy sources in that clone.
  # Otherwise, the installed version of fedempy will be used.
  get_filename_component ( _PARENT_DIR "${PROJECT_SOURCE_DIR}" PATH )
  if ( NOT IS_DIRECTORY "${_PARENT_DIR}/fedem-solvers" )
    get_filename_component ( _PARENT_DIR "${_PARENT_DIR}" PATH )
  endif ( NOT IS_DIRECTORY "${_PARENT_DIR}/fedem-solvers" )
  if ( IS_DIRECTORY "${_PARENT_DIR}/fedem-solvers" )
    set ( PYTHON_DIR "${_PARENT_DIR}/fedem-solvers/PythonAPI/src" )
    set ( TEST_DIR "${_PARENT_DIR}/fedem-solvers/solverTests/TimeDomain" )
  elseif ( DEFINED ENV{TEST_DIR} )
    set ( TEST_DIR "$ENV{TEST_DIR}" )
  endif ( IS_DIRECTORY "${_PARENT_DIR}/fedem-solvers" )

  if ( CMAKE_VERSION VERSION_LESS 3.12 )
    find_package ( PythonInterp 3 ) # enforce using python3
  else ( CMAKE_VERSION VERSION_LESS 3.12 )
    find_package ( Python3 )
    set ( PythonInterp_FOUND ${Python3_FOUND} )
    set ( PYTHON_EXECUTABLE ${Python3_EXECUTABLE} )
  endif ( CMAKE_VERSION VERSION_LESS 3.12 )
  if ( PythonInterp_FOUND )
    add_subdirectory ( test/fedempy )
  endif ( PythonInterp_FOUND )

endif ( BUILD_TESTS )
