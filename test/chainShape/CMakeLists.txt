# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Build setup

set ( LIB_ID ChainShapeTests )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )

include_directories ( ${PROJECT_BINARY_DIR}/chainShape )

if ( pFUnit_FOUND )
# Add some unit tests using pFUnit (executed via ctest)
  enable_fortran_tests ()
  add_fortran_test ( test_ChainShape testCS.pf ChainShape )
else ( pFUnit_FOUND )
  add_executable ( ChainShapeTest test.f90 )
  target_link_libraries ( ChainShapeTest ChainShape )
endif ( pFUnit_FOUND )
