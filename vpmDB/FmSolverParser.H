// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SOLVER_PARSER_H
#define FM_SOLVER_PARSER_H

#include <vector>
#include <string>
#include <cstdio>

class FmPart;
class FmJointBase;
class FmCamJoint;


class FmSolverParser
{
public:
  FmSolverParser() { myFile = NULL; }
  FmSolverParser(const char* fileName);
  ~FmSolverParser();

  int writeFullFile();

  static bool preSimuleCheck();

  // Use this to correct the relative paths printed by each entry,
  // set e.g. to "../../" to resolve for the model_RDB/response_#### directory
  void setRelPathCorrection(const std::string& path);
  void setRDBPath(const std::string& path) { myRDBPath = path; }

protected:
  FmSolverParser(const FmSolverParser&) = delete;
  FmSolverParser& operator=(const FmSolverParser&) = delete;

  int writeHeading();
  int writeEnvironment();
  int writeAdditionalMasses();
  int writeBeams(int& nextBaseId);
  int writeParts(std::vector<FmPart*>& gageParts);
  int writeRosettes(const std::vector<FmPart*>& gageParts);
  int writeSensors();
  int writeSprings();
  int writeJoints();

  int writeRotationJointVars(const char* jVarDefs,
			     FmJointBase* aJoint, std::vector<int>& iDof);
  int writeContactElement(FmCamJoint* aJoint);
  int writeFriction(FmJointBase* aJoint, const std::vector<int>& iDof);

  // Usable for all classes for which the printSolverEntry method is defined
  int writeAllOfType(int classTypeID);

private:
  std::string myRelPathCorrection;
  std::string myRDBPath;
  FILE*       myFile;
};

#endif
