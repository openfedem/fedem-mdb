// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file test_creators.C
  \brief Unit testing for the assembly creator methods.
*/

#include "gtest.h"
#include "assemblyCreators/turbineConverter.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmapsMain.H"

static std::string srcdir; //!< Full path of the source directory of this test


/*!
  \brief Main program for the unit test executable.
*/

int main (int argc, char** argv)
{
  // Initialize the google test module.
  // This will remove the gtest-specific values in argv.
  ::testing::InitGoogleTest(&argc,argv);

  // Extract the source directory of the tests
  // to use as prefix for loading external files
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i],"--srcdir=",9))
    {
      srcdir = argv[i]+9;
      std::cout <<"Note: Source directory = "<< srcdir << std::endl;
      if (srcdir.back() != '/') srcdir += '/';
      break;
    }

  // Initialize the Fedem mechanism database
  FmDB::init();

  // Invoke the google test driver
  int status = RUN_ALL_TESTS();

  // Clean up model-independent heap memory
  FmDB::removeInstances();
  return status;
}


TEST(TestCreator,turbine)
{
  const std::string bladeFile = srcdir + "models/Sample_5MW.fmm";

  // Create the default turbine model
  ASSERT_TRUE(FmDB::newMechanism() != NULL);
  ASSERT_TRUE(FmDB::getTurbineObject(-1) != NULL);
  ASSERT_TRUE(FWP::readBladeDesign(bladeFile,NULL) != NULL);
  ASSERT_TRUE(FWP::updateTurbine());

  // Clean up
  FmDB::eraseAll();
}
