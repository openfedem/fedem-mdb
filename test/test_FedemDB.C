// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file test_FedemDB.C
  \brief Unit testing for FedemDB.
*/

#include "gtest.h"

extern "C" {
  void FmInit(const char* = NULL, const char* = NULL);
  void FmNew (const char* = NULL);
  bool FmOpen(const char* = NULL);
  bool FmSave(const char* = NULL);
  void FmClose(bool = true);
  int  FmCreateTriad(const char*, double, double, double,
                     double = 0.0, double = 0.0, double = 0.0, int = 0);
  bool FmAddMass(int, int, const double*, int = 0);
  int  FmCreatePart(const char*, int, int*);
  int  FmCreateJoint(const char*, int, int, int*, int);
  bool FmSolve(const char*, bool = true,
               const char* = NULL, const char* = NULL);
}

static std::string srcdir; //!< Full path of the source directory of this test


/*!
  \brief Opens a specified model file and saves with solver input generated.
*/

int loadTest (const char* fmmFile)
{
  // Open the model and save to current working directory
  std::string oldFmm(fmmFile);
  std::string newFmm = oldFmm.substr(oldFmm.find_last_of("/\\") + 1);
  if (!FmOpen(oldFmm.c_str())) return 1;
  if (!FmSave(newFmm.c_str())) return 2;
  // Write solver input and save updated model
  std::string newRDB = newFmm.substr(0,newFmm.find_last_of(".")) + "_RDB";
  if (!FmSolve(newRDB.c_str())) return 3;
  return FmSave() ? 0 : 4;
}

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
  const char* fmmFile = NULL;
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i],"--srcdir=",9) && srcdir.empty())
    {
      srcdir = argv[i]+9;
      std::cout <<"Note: Source directory = "<< srcdir << std::endl;
      if (srcdir.back() != '/') srcdir += '/';
    }
    else if (!strcmp(argv[i],"-f") && i+1 < argc)
      fmmFile = argv[++i];

  // Initialize the Fedem mechanism database
  FmInit();

  // Invoke the google test driver, or the load-and-save test
  int status = fmmFile ? loadTest(fmmFile) : RUN_ALL_TESTS();

  // Clean up heap memory
  FmClose();

  return status;
}


/*!
  \brief Unit test creating the simplest solvable model.
*/

TEST(TestFedemDB,Simplest)
{
  FmNew("simplest.fmm");

  double mass[4] = { 5.0, 1.1, 1.2, 1.3 };
  ASSERT_TRUE(FmAddMass(FmCreateTriad("T1",0.0,0.0,0.0),4,mass));
  ASSERT_TRUE(FmSave());
}


/*!
  \brief Unit test creating two prismatic joints close to each other.
*/

TEST(TestFedemDB,Prismatic)
{
  const int PRISMATIC = 11;

  FmNew("prismatic.fmm");

  std::vector<int> triads;
  char T[3] = "T0";
  char M[3] = "M0";
  double x = 0.0;
  for (int i = 0; i < 10; i++, x += 10.0, T[1]++)
    triads.push_back(FmCreateTriad(T, x, 0.0, 0.0));
  x = 0.0;
  for (int i = 0; i < 8; i++, x += 11.25, M[1]++)
    triads.push_back(FmCreateTriad(M, x, 0.25, 0.3));
  int s1 = FmCreateTriad("S1", 0.5, 0.0, 0.0);
  int s2 = FmCreateTriad("S2", 0.5, 0.25, 0.3);
  int m1[2] = { triads.front(), triads[9] };
  int m2[2] = { triads[10], triads.back() };
  ASSERT_GT(FmCreatePart("My part", triads.size(), triads.data()), 0);
  ASSERT_GT(FmCreateJoint("J1", PRISMATIC, s1, m1, 2), 0);
  ASSERT_GT(FmCreateJoint("J2", PRISMATIC, s2, m2, 2), 0);
  ASSERT_TRUE(FmSave());
}


//! \brief Class describing a parameterized unit test instance.
class TestCase : public testing::Test, public testing::WithParamInterface<const char*> {};


/*!
  \brief Unit test creating a dynamics solver RDB directory for a model.
*/

TEST_P(TestCase,SolverRDB)
{
  ASSERT_FALSE(srcdir.empty());

  // GetParam() will be substituted with the actual file name.
  std::string fmmFile = srcdir + GetParam();

  // Open the model and save to current working directory
  std::string newFmm = fmmFile.substr(fmmFile.find_last_of("/\\")+1);
  ASSERT_TRUE(FmOpen(fmmFile.c_str()));
  ASSERT_TRUE(FmSave(newFmm.c_str()));
  // Write solver input and save updated model
  std::string newRDB = newFmm.substr(0,newFmm.find_last_of(".")) + "_RDB";
  ASSERT_TRUE(FmSolve(newRDB.c_str()));
  ASSERT_TRUE(FmSave());
}


/*!
  \brief Instantiate the test over a list of file names.
*/

INSTANTIATE_TEST_CASE_P(TestParsing, TestCase,
    testing::Values("models/Gravemaskin.fmm"));
