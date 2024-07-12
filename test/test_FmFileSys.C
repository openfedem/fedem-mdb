// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file test_FmFileSys.C
  \brief Unit testing for some file handling methods.
*/

#include "gtest.h"
#include "vpmDB/FmFileSys.H"
#include <iostream>


/*!
  \brief Removes the specified directory \a dirName recursively.
*/

static int removeDir (const char* dirName, bool delFile = false)
{
  std::cout <<"\nFmFileSys::removeDir(): "<< dirName
            <<" "<< std::boolalpha << delFile << std::endl;
  int ndel = FmFileSys::removeDir(dirName,delFile);
  if (ndel >= 0)
    std::cout <<"Success, "<< ndel <<" files deleted."<< std::endl;
  else
    std::cout <<"Failure to delete "<< -ndel <<" files."<< std::endl;

  return ndel < 0 ? ndel : 0;
}


/*!
  \brief Main program for the unit test executable.
*/

int main (int argc, char** argv)
{
  if (argc < 2 || !FmFileSys::isDirectory(argv[1]))
  {
    // Invoke the google test driver
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
  }

  // Stand-alone execution,
  // try to remove directory specified as command-line argument
  std::vector<std::string> files;
  FmFileSys::getFiles(files,argv[1],NULL,true);
  std::cout <<"FmFileSys::getFiles(): "<< argv[1];
  for (const std::string& fn : files) std::cout <<"\n\t"<< fn;
  return removeDir(argv[1], argc > 2 ? argv[2][0] == 'y' : true);
}


TEST(TestFmFileSys,removeDir)
{
  ASSERT_TRUE(FmFileSys::verifyDirectory("tmpDir"));
  ASSERT_TRUE(FmFileSys::verifyDirectory("tmpDir/subDir1"));
  ASSERT_TRUE(FmFileSys::verifyDirectory("tmpDir/subDir2"));
  ASSERT_TRUE(FmFileSys::verifyDirectory("tmpDir/subDir1/subDir3"));
  ASSERT_EQ(removeDir("tmpDir"),0);
}
