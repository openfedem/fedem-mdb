# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Build setup

set ( LIB_ID vpmDBTests )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )


if ( USE_QT )
  find_package ( Qt4 COMPONENTS QtCore )
endif ( USE_QT )

if ( Qt4_FOUND )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_QT" )
elseif ( LINUX )
  string ( APPEND CMAKE_CXX_FLAGS " -DFT_HAS_DIRENT" )
endif ( Qt4_FOUND )


# Add some unit tests using gtest (executed via ctest)
add_executable ( test_FmPart test_FmPart.C )
add_cpp_test ( test_FmPart vpmDB )

add_executable ( test_FmFileSys test_FmFileSys.C ../vpmDB/FmFileSys.C )
target_link_libraries ( test_FmFileSys FFaOS )
if ( Qt4_FOUND )
  target_link_libraries ( test_FmFileSys Qt4::QtCore )
endif ( Qt4_FOUND )
