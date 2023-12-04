// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODEL_LOADER_H
#define FM_MODEL_LOADER_H

#include <string>


namespace Fedem
{
  int loadTemplate(const std::string& newName,
                   const std::string& defaultName = "",
                   bool writeLogFile = false);
  int loadModel(const std::string& name,
                const std::string& logName,
                char ignoreFileVersion = 0);
  bool loadParts(bool forceLoad = false);
}

#endif
