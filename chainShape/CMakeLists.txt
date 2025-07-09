# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Library setup

set ( LIB_ID ChainShape )
set ( LIB_ID_LIST Minpack )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )

foreach ( FOLDER ${LIB_ID_LIST} )
  add_subdirectory ( ${FOLDER} )
endforeach ( FOLDER ${LIB_ID_LIST} )

# Build and install

file ( GLOB F90_SOURCE_FILES *.f90 )
add_library ( ${LIB_ID} SHARED ${F90_SOURCE_FILES} )
target_link_libraries ( ${LIB_ID} LINK_PRIVATE Minpack )

install ( TARGETS ${LIB_ID}
          RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
          LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" )
