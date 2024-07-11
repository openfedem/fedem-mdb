# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Build setup

set ( LIB_ID vpmDBTests )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )

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
if ( USE_MEMPOOL )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_MEMPOOL" )
endif ( USE_MEMPOOL )
if ( USE_VISUALS )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_USE_VISUALS" )
endif ( USE_VISUALS )


# Add some unit tests using gtest (executed via ctest)
add_executable ( test_FmPart test_FmPart.C )
add_cpp_test ( test_FmPart vpmDB )

add_executable ( test_FmFileSys test_FmFileSys.C ../vpmDB/FmFileSys.C )
add_cpp_test ( test_FmFileSys FFaOS )
if ( Qt6_FOUND )
  target_link_libraries ( test_FmFileSys Qt6::Core )
elseif ( Qt5_FOUND )
  target_link_libraries ( test_FmFileSys Qt5::Core )
elseif ( Qt4_FOUND )
  target_link_libraries ( test_FmFileSys Qt4::QtCore )
endif ( Qt6_FOUND )
