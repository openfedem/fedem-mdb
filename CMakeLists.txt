# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

################################################################################
# This is the top-level cmake project file for the Fedem model database library.
################################################################################

cmake_minimum_required ( VERSION 2.8...3.5 )

# Project setup

set ( APPLICATION_ID fedemDB )
set ( DOMAIN_ID FEDEM )
set ( PACKAGE_ID MDB )
set ( LIB_ID FedemDB )
set ( LIB_ID_LIST Admin FFaLib FFlLib FFaFunctionLib FFaMathExpr
                  FiDeviceFunctions FiUserElmPlugin )

set ( USE_VISUALS ON ) # Visualization attribute classes must be included
option ( USE_FORTRAN "Build with Fortran code included" OFF )
option ( USE_QT "Use Qt file system handling" OFF )
option ( USE_MEMPOOL "Use memory pool for heap allocation in FE library" OFF )
option ( USE_PROFILER "Use CPU and Memory profiler" OFF )
mark_as_advanced ( USE_FORTRAN USE_QT USE_MEMPOOL USE_PROFILER )

if ( USE_FORTRAN)
  project ( ${APPLICATION_ID} CXX C Fortran )
else ( USE_FORTRAN)
  project ( ${APPLICATION_ID} CXX C )
endif ( USE_FORTRAN)
message ( STATUS "Generating build project for ${PROJECT_SOURCE_DIR}" )
get_filename_component ( PROJECT_PARENT_DIR "${PROJECT_SOURCE_DIR}" PATH )
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
unset ( _MODULES CACHE )

include ( CheckPFUnit )
include ( FedemConfig )


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
find_package ( PythonInterp 3 ) # Enforce using python3


# Library setup

# Include dependent libraries from fedem-foundation
include_directories ( "${PROJECT_SOURCE_DIR}/fedem-foundation/src" )
foreach ( FOLDER ${LIB_ID_LIST} )
  add_subdirectory ( fedem-foundation/src/${FOLDER} "${CMAKE_CURRENT_BINARY_DIR}/${FOLDER}" )
endforeach ( FOLDER ${LIB_ID_LIST} )
if ( USE_FORTRAN )
  add_subdirectory ( chainShape )
endif ( USE_FORTRAN )

# Exclude some targets from sub-projects that are not needed here
set_target_properties ( FFaOperation FFlVisualization
                        PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1 )

# Include the main library
set ( NO_FATIGUE true )
string ( APPEND CMAKE_CXX_FLAGS_DEBUG " -DFM_DEBUG=${FT_DEBUG}" )
include_directories ( "${CMAKE_CURRENT_SOURCE_DIR}" )
add_subdirectory ( vpmDB )

# Build the shared library
message ( STATUS "Building shared library ${LIB_ID}" )
add_library ( ${LIB_ID} SHARED FedemDB.C )
target_link_libraries ( ${LIB_ID} vpmDB )


# Perform the unit- and regression tests

if ( GTest_FOUND )
  add_subdirectory ( test )
endif ( GTest_FOUND )
if ( TARGET ChainShape )
  add_subdirectory ( test/chainShape )
endif ( TARGET ChainShape )
set ( PYTHON_DIR "${PROJECT_PARENT_DIR}/fedem-solvers/PythonAPI/src" )
if ( PythonInterp_FOUND AND IS_DIRECTORY "${PYTHON_DIR}" )
  add_subdirectory ( test/fedempy )
endif ( PythonInterp_FOUND AND IS_DIRECTORY "${PYTHON_DIR}" )
