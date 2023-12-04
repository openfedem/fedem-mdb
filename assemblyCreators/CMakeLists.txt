# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

# Library setup

set ( LIB_ID assemblyCreators )
set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_${LIB_ID} )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )


## Files with header and source with same name
set ( COMPONENT_FILE_LIST eventCreator turbineConverter )
## Pure header files, i.e., header files without a corresponding source file
set ( HPP_FILE_LIST assemblyCreators )
## Pure implementation files, i.e., source files without corresponding header
set ( CPP_FILE_LIST bladeCreator regsysCreator graphCreator jacketCreator )


foreach ( FILE ${COMPONENT_FILE_LIST} )
  list ( APPEND CPP_FILES ${FILE}.C )
  list ( APPEND HPP_FILES ${FILE}.H )
endforeach ( FILE ${COMPONENT_FILE_LIST} )

foreach ( FILE ${HPP_FILE_LIST} )
  list ( APPEND HPP_FILES ${FILE}.H )
endforeach ( FILE ${HPP_FILE_LIST} )

foreach ( FILE ${CPP_FILE_LIST} )
  list ( APPEND CPP_FILES ${FILE}.C )
endforeach ( FILE ${CPP_FILE_LIST} )


add_library ( ${LIB_ID} ${CPP_FILES} ${HPP_FILES} )
target_link_libraries ( ${LIB_ID} vpmDB FFlLib FFaDefinitions FFaString )
