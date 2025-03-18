# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

set ( UNIT_ID ${DOMAIN_ID}_${PACKAGE_ID}_fedempy )

message ( STATUS "INFORMATION : Processing unit ${UNIT_ID}" )

################################################################################
# Set up regression tests for the FEDEM model database shared object library.
# Each test uses a python driver (test_<name>.py) which is invoked by specifying
# ${PYTHON_EXECUTABLE} as the command in the add_test() calls, and with a
# generated wrapper that invokes the actual test driver scripts as argument.

if ( PYTHON_DIR )
  set ( PYTHONPATH "${PYTHON_DIR}${CMAKE_PATH_SEP}${CMAKE_CURRENT_SOURCE_DIR}" )
else ( PYTHON_DIR )
  set ( PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}" )
endif ( PYTHON_DIR )
if ( NOT CMAKE_BUILD_TYPE )
  # Build type not defined (Windows), assume Release build here
  message ( STATUS "NOTE : The fedempy tests will always use Release build" )
  set ( CMAKE_BUILD_TYPE Release )
endif ( NOT CMAKE_BUILD_TYPE )

################################################################################
# Generate a Python wrapper that loads the correct shared object library
# for current build configuration, while executing some modeling tests.
# The generated models will also be tried solved, but only if
# the environment variable FEDEM_SOLVER has been defined.
file ( GENERATE OUTPUT
       ${CMAKE_CURRENT_BINARY_DIR}/test_modeler_$<CONFIGURATION>.py
       CONTENT "from os import environ
environ['FEDEM_MDB']='$<TARGET_FILE:FedemDB>'
have_ude = True if 'UDELM_PLUGIN' in environ else False
do_solve = True if 'FEDEM_SOLVER' in environ else False
import test_modeler
import test_functions
import test_fepart
import test_springdamper
import test_joint
import test_revjoint
from test_userdefelm import main
if have_ude:
    main('ubar.fmm', environ['UDELM_PLUGIN'], do_solve)
" )
message ( STATUS "INFORMATION : Adding regression test fedempy_modeler" )
add_test ( fedempy_modeler ${PYTHON_EXECUTABLE} -B test_modeler_${CMAKE_BUILD_TYPE}.py )
set_tests_properties ( fedempy_modeler PROPERTIES ENVIRONMENT
                       "PYTHONPATH=${PYTHONPATH};SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}" )

################################################################################
# Generate a Python wrapper that loads the correct shared object library
# for current build configuration, while executing the no-code modeling test.
# The generated model will also be tried solved, but only if
# the environment variable FEDEM_SOLVER has been defined.
file ( GENERATE OUTPUT
       ${CMAKE_CURRENT_BINARY_DIR}/test_YAML_$<CONFIGURATION>.py
       CONTENT "from os import environ
from fedempy.yaml_parser import main
environ['FEDEM_MDB']='$<TARGET_FILE:FedemDB>'
do_solve = True if 'FEDEM_SOLVER' in environ else False
main('${CMAKE_CURRENT_SOURCE_DIR}/models/01-loader.yaml', '01-loader.json')
main('${CMAKE_CURRENT_SOURCE_DIR}/models/02-loader.yaml', '02-loader.json', do_solve)
" )
message ( STATUS "INFORMATION : Adding regression test fedempy_YAML" )
add_test ( fedempy_YAML ${PYTHON_EXECUTABLE} -B test_YAML_${CMAKE_BUILD_TYPE}.py )
set_tests_properties ( fedempy_YAML PROPERTIES ENVIRONMENT
                       "PYTHONPATH=${PYTHONPATH}" )

# Add a test comparing the two json-files of the two models that should be identical
add_test ( YAML-comp ${CMAKE_COMMAND} -E compare_files 01-loader.json 02-loader.json )
set_tests_properties ( YAML-comp PROPERTIES DEPENDS fedempy_YAML )

################################################################################
# Macro generating a Python wrapper that loads the correct shared object library
# for current build configuration, while executing the script test_${TEST_ID}.py
macro ( add_fedempy_test TEST_ID )
  file ( GENERATE OUTPUT
         ${CMAKE_CURRENT_BINARY_DIR}/test_${TEST_ID}_$<CONFIGURATION>.py
         CONTENT "from os import environ
environ['FEDEM_MDB']='$<TARGET_FILE:FedemDB>'
import test_${TEST_ID}
" )
  message ( STATUS "INFORMATION : Adding regression test fedempy_${TEST_ID}" )
  add_test ( fedempy_${TEST_ID} ${PYTHON_EXECUTABLE} -B test_${TEST_ID}_${CMAKE_BUILD_TYPE}.py --force-new )
  set_tests_properties ( fedempy_${TEST_ID} PROPERTIES ENVIRONMENT
                         "PYTHONPATH=${PYTHONPATH};${ARGN}" )
endmacro ( add_fedempy_test )

# The following tests will be active only if the variable TEST_DIR
# points to the root folder of the Time-domain regression tests,
# cloned from the fedem-solver-tests repository.
# They will not do anything if the environment variable FEDEM_SOLVER is not set.
if ( TEST_DIR )
  add_fedempy_test ( reducer TEST_DIR=${TEST_DIR} )
  add_fedempy_test ( solver TEST_DIR=${TEST_DIR} )
  set_tests_properties ( fedempy_solver PROPERTIES DEPENDS fedempy_reducer )
  add_fedempy_test ( solveSLA TEST_DIR=${TEST_DIR} )
  add_fedempy_test ( extfunc TEST_DIR=${TEST_DIR} )
  add_fedempy_test ( inverse TEST_DIR=${TEST_DIR} )
  add_fedempy_test ( Microbatch SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR};TEST_DIR=${TEST_DIR} )
  add_fedempy_test ( SLA SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR};TEST_DIR=${TEST_DIR} )
endif ( TEST_DIR )

# The remaining tests only depend of files within this repository.
add_fedempy_test ( Loader SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} )
set ( PYTHONPATH "${PYTHONPATH}${CMAKE_PATH_SEP}${CMAKE_CURRENT_SOURCE_DIR}/divergence" )
configure_file ( divergence/loader_01.fmm loader_01.fmm )
add_fedempy_test ( Divergence SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} )
add_fedempy_test ( StressExport )
set_tests_properties ( fedempy_StressExport PROPERTIES DEPENDS fedempy_Loader )
