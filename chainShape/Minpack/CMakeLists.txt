# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Library setup

set ( LIB_ID Minpack )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )

if ( FTENV_WARNINGS AND LINUX AND NOT USE_INTEL_FORTRAN )
  string ( APPEND CMAKE_Fortran_FLAGS " -Wno-compare-reals" )
endif ( FTENV_WARNINGS AND LINUX AND NOT USE_INTEL_FORTRAN )

file ( GLOB F77_SOURCE_FILES src/*.f )
add_library ( ${LIB_ID} STATIC ${F77_SOURCE_FILES} )
