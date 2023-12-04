// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmSolverInput.H
  \brief Global functions for managing creation of solver input files.
*/

#ifndef FM_SOLVER_INPUT_H
#define FM_SOLVER_INPUT_H

#include <string>
#include <vector>

class FmAnalysis;
class FmMechanism;
class FmPart;
class FmSimulationEvent;


namespace Fedem
{
  //! \brief Helper to check whether a named fmx-file is valid or not.
  //! \param[in] filename Absolute path to the fmx-file to check
  //! \param[in] wantCS Expected checksum value (ignore if zero)
  //! \param missingFile List of non-existing files (optional)
  //! \param wrongCS Counts files with checksum mismatch (optional)
  bool validFileCheck(const std::string& filename,
                      unsigned long int wantCS = 0,
                      std::string* missingFile = NULL,
                      int* wrongCS = NULL);

  //! \brief Checks the validity of the reduced matrix files of a part.
  //! \param[in] part The FE part to check matrix files for
  //! \param[in] needMassMatrix If \e true, mass matrix files are needed
  //! \param[in] checkingWith Flag indicating which files to check for.
  //! 'S': Files needed by the Dynamics Solver only.
  //! 'R': Files needed by Stress Recovery only.
  //! 'A': Check all files.
  //! \param[in] preparingForBatch If \e true, prepare for batch execution
  //! \param[in] wantCS Expected checksum value (ignore if zero)
  int checkReducerFiles(const FmPart* part,
                        bool needMassMatrix = false,
                        char checkingWhich = 'A',
                        bool preparingForBatch = false,
                        unsigned long int wantCS = 0);

  //! \brief Creates input files for the FE part reducer
  //! \param[in] analysis The analysis object of current model
  //! \param[in] mech The mechanism object of current model
  //! \param[in] part The FE part to create reduction input files for
  //! \param[in] solverName Name of reduction solver (fedem_reducer)
  //! \param[in] preparingForBatch If \e true, prepare for batch execution
  //! \param[in] wantCS Expected checksum value (ignore if zero)
  //! \return Path to execution folder on success, error message on failure
  std::string createReducerInput(FmAnalysis* analysis, FmMechanism* mech,
                                 FmPart* part, const std::string& solverName,
                                 bool preparingForBatch = false,
                                 unsigned long int wantCS = 0);

  //! \brief Creates input files for the dynamics solver
  //! \param[in] analysis The analysis object of current model
  //! \param[in] mech The mechanism object of current model
  //! \param[in] sev The simulation event (if any) to create input for
  //! \param[in] solverName Name of the dynamics solver (fedem_solver)
  //! \param[in] plugins Paths to solver plugin libraries
  //! \param[in] rdbPath List of absolute paths of RDB-folders to be used
  //! \param[in] preparingForBatch If \e true, prepare for batch execution
  //! \param[in] keepOldRes if \e true, any existing res-file is not overwritten
  //! \return Input file basename on success, error message on failure
  std::string createSolverInput(FmAnalysis* analysis,
                                FmMechanism* mech,
                                FmSimulationEvent* sev,
                                const std::string& solverName,
                                const std::vector<std::string>& plugins,
                                std::vector<std::string>& rdbPath,
                                bool preparingForBatch = false,
                                bool keepOldRes = true);
}

#endif
