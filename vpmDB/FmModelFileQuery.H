// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODEL_FILE_QUERY_H
#define FM_MODEL_FILE_QUERY_H

#include <string>
#include <vector>


/*!
  \class FmModelFileQuery
  \brief Utility to query the contents of a model file by keyWord.

  Comes in handy if you want to check the contents of other
  model files than the one you are currently working with.

  \author Sven-Kaare Evenseth
*/

class FmModelFileQuery
{
public:
  FmModelFileQuery(const std::string& filePath = "") : myModelFile(filePath) {}

  std::vector<std::string> getEntries(const std::string& keyWord, bool all = true);
  std::string getFirstEntry(const std::string& keyWord);

private:
  std::string myModelFile;
};

#endif
