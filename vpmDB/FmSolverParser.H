// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmSolverParser.H
  \brief Class managing writing the FEDEM dynamics solver input file.
*/

#ifndef FM_SOLVER_PARSER_H
#define FM_SOLVER_PARSER_H

#include <vector>
#include <string>
#include <cstdio>

class FmPart;
class FmJointBase;
class FmCamJoint;


/*!
  \brief Class managing writing the FEDEM dynamics solver input file.
*/

class FmSolverParser
{
public:
  //! \brief The default constructor opens the named solver input file.
  explicit FmSolverParser(const char* fileName = NULL);
  //! \brief The destructor closes the solver input file.
  ~FmSolverParser();

  //! \brief Writes the full solver input file for the current model.
  int writeFullFile();

  //! \brief Performs some consistency checking on the current model.
  static bool preSimuleCheck();

  //! \brief Assigns the #myRelPathCorrection member.
  //! \details Use this to correct the relative paths printed by each entry,
  //! set e.g. to "../../" to resolve for the model_RDB/response_#### directory.
  void setRelPathCorrection(const std::string& path);
  //! \brief Assigns the #myRDBPath member.
  void setRDBPath(const std::string& path) { myRDBPath = path; }

protected:
  //! \brief Deleted copy constructor.
  FmSolverParser(const FmSolverParser&) = delete;
  //! \brief Deleted assignment operator.
  FmSolverParser& operator=(const FmSolverParser&) = delete;

  //! \brief Writes the heading of the solver input file.
  int writeHeading();
  //! \brief Writes the environment record to the solver input file.
  int writeEnvironment();
  //! \brief Writes all additional masses to the solver input file.
  int writeAdditionalMasses();
  //! \brief Writes all beam elements to the solver input file.
  int writeBeams(int& nextBaseId);
  //! \brief Writes all generic and FE parts to the solver input file.
  int writeParts(std::vector<FmPart*>& gageParts);
  //! \brief Writes all strain rosettes to the solver input file.
  int writeRosettes(const std::vector<FmPart*>& gageParts);
  //! \brief Writes all sensors to the solver input file.
  int writeSensors();
  //! \brief Writes all axial- and joint-springs to the solver input file.
  int writeSprings();
  //! \brief Writes all joints to the solver input file.
  int writeJoints();

  //! \brief Writes a cam joint element to the solver input file.
  int writeContactElement(FmCamJoint* aJoint);
  //! \brief Writes friction data, if any, for a joint to the solver input file.
  int writeFriction(FmJointBase* aJoint, const std::vector<int>& iDof);

  //! \brief Writes solver input record for all objects of type \a classTypeID.
  //! \details This method is usable for all classes for which the
  //! FmModelMemberBase::printSolverEntry() method is implemented.
  int writeAllOfType(int classTypeID);

private:
  //! Absolute path to where the dynamics solver will be run from
  std::string myRDBPath;
  //! Path to the model file location relative to #myRDBPath
  std::string myRelPathCorrection;

  FILE* myFile; //!< Pointer to the solver input file being written
};

#endif
