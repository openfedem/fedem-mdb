// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file test_FmPart.C
  \brief Unit testing for the class FmPart.
*/

#include "gtest.h"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmFileSys.H"
#include "vpmDB/Icons/FmIconPixmapsMain.H"
#include "FFlLib/FFlLinkHandler.H"
#include "FFlLib/FFlFEParts/FFlNode.H"
#include "FFaLib/FFaCmdLineArg/FFaCmdLineArg.H"

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
  int numarg = 0;
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i],"--srcdir=",9))
    {
      srcdir = argv[i]+9;
      std::cout <<"Note: Source directory = "<< srcdir << std::endl;
      if (srcdir.back() != '/') srcdir += '/';
    }
    else if (++numarg < i)
      argv[numarg] = argv[i];

  // Initialize the Fedem mechanism database
  FFaCmdLineArg::init(numarg,argv);
  FFaCmdLineArg::instance()->addOption("reUseUserID",false,"Fill holes in user ID range");
  FmDB::init();

  // Invoke the google test driver
  int status = RUN_ALL_TESTS();

  FFaCmdLineArg::removeInstance();
  return status;
}


TEST(TestFmPart,setValidBaseFTLFile)
{
  FmMechanism* mech = new FmMechanism();
  ASSERT_TRUE(mech->connect());
  mech->modelLinkRepository.setValue(srcdir+"LinkDB");
  ASSERT_TRUE(FmFileSys::isDirectory(mech->modelLinkRepository.getValue()));

  FFlLinkHandler* linkA = new FFlLinkHandler();
  linkA->addNode(new FFlNode(1,1.0,0.0,0.0));
  linkA->resolve();

  FFlLinkHandler* linkB = new FFlLinkHandler();
  linkB->addNode(new FFlNode(1,2.0,0.0,0.0));
  linkB->resolve();

  FFlLinkHandler* linkC = new FFlLinkHandler();
  linkC->addNode(new FFlNode(1,3.0,0.0,0.0));
  linkC->resolve();

  auto&& getFTLName = [](const FmPart* part) { return part->baseFTLFile.getValue().c_str(); };

  FmPart* myPart = new FmPart("Part A");
  ASSERT_TRUE(myPart->connect());
  myPart->originalFEFile.setValue("peder/jalla.nas");
  myPart->setLinkHandler(linkA);
  myPart->setValidBaseFTLFile(0);
  EXPECT_STREQ(getFTLName(myPart),"jalla.ftl");

  myPart = new FmPart("Part B");
  ASSERT_TRUE(myPart->connect());
  myPart->originalFEFile.setValue("peder/jalla.nas");
  myPart->setLinkHandler(linkB);
  myPart->setValidBaseFTLFile(0);
  EXPECT_STREQ(getFTLName(myPart),"jalla_ftl1.ftl");

  myPart = new FmPart("Part C");
  ASSERT_TRUE(myPart->connect());
  myPart->originalFEFile.setValue("peder/jalla.nas");
  myPart->setLinkHandler(linkC);
  myPart->setValidBaseFTLFile(0);
  EXPECT_STREQ(getFTLName(myPart),"jalla_ftl2.ftl");

  myPart = new FmPart("Part D");
  ASSERT_TRUE(myPart->connect());
  myPart->originalFEFile.setValue("peder/jalla.nas");
  myPart->setValidBaseFTLFile(66);
  EXPECT_STREQ(getFTLName(myPart),"jalla_ftl1-2.ftl");

  myPart = new FmPart("Part E");
  ASSERT_TRUE(myPart->connect());
  myPart->originalFEFile.setValue("peder/jalla.nas");
  myPart->setValidBaseFTLFile(99);
  EXPECT_STREQ(getFTLName(myPart),"jalla_ftl1-3.ftl");

  // Clean up
  std::vector<FmPart*> allParts;
  FmDB::getAllParts(allParts);
  for (FmPart* part : allParts)
    part->erase();
  mech->erase();
}
