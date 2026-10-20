// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmModelLoader.H
  \brief Global functions for managing the parsing of FEDEM model files.
*/

#ifndef FM_MODEL_LOADER_H
#define FM_MODEL_LOADER_H

#include <string>


namespace Fedem
{
  //! \brief Reads a model file to be used as template for a new, empty model.
  //! \param[in] newName The name of the new model file
  //! \param[in] defaultName Default name of the template model file.
  //! Used only if the FEDEM_TEMPLATE_FILE environment variable is not set.
  //! \param[in] writeLogFile Copy messages to a temporary log file, if \e true
  int loadTemplate(const std::string& newName,
                   const std::string& defaultName = "",
                   bool writeLogFile = false);

  //! \brief Reads a model file into a new model.
  //! \param[in] name The name of the model file to read
  //! \param[in] logName Name of the log-file for list view messages
  //! \param[in] ignoreFileVersion If &gt; 0, ignore incompatible file versions
  int loadModel(const std::string& name,
                const std::string& logName,
                char ignoreFileVersion = 0);

  //! \brief Reads all part data files, unless they are already loaded
  //! \param[in] forceLoad If \e true, force reload of all files
  bool loadParts(bool forceLoad = false);
}

#endif
