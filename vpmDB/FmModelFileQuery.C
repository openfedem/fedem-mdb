// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmModelFileQuery.H"

#include <iostream>
#include <fstream>


/*!
  \brief Returns the values of the provided keyword.

  If \a all is false, it will stop after the first occurance of \a keyWord.
  Returns empty vector if no model file is set or if the file cannot be read.

  \sa getFirstEntry
*/

std::vector<std::string> FmModelFileQuery::getEntries(const std::string& keyWord, bool all)
{
#ifdef FM_DEBUG
  std::cout <<"FmModelFileQuery::getEntries() "<< keyWord << std::endl;
#endif
  std::vector<std::string> ret;

  if (myModelFile.empty()) {
    std::cerr <<"FmModelFileQuery: No model file selected."<< std::endl;
    return ret;
  }

  std::ifstream stream(myModelFile.c_str());
  if (!stream) {
    std::cerr <<"FmModelFileQuery: Could not open "<< myModelFile << std::endl;
    return ret;
  }

  while (!stream.eof()) {

    char c;
    std::string retString, key;

    stream >> key;
    if (key == keyWord) {

#ifdef FM_DEBUG
      std::cout <<"--> Found match"<< std::endl;
#endif

      // Skipping leading whitespace and '='
      while (stream.get(c) &&( isspace(c) || (c == '=')));

      while (c != ';') {
	retString.append(1, c);
	stream.get(c);
      }
#ifdef FM_DEBUG
      std::cout <<"complete std::string "<< retString << std::endl;
#endif
      ret.push_back(retString);
      if (!all)
	break;
    }
  }

  return ret;
}


/*!
  Provided for convenience.
  Does the same as the above, and returns the first occurance of the keyWord.

  \sa getEntries
*/

std::string FmModelFileQuery::getFirstEntry(const std::string& keyWord)
{
  std::vector<std::string> tmp = this->getEntries(keyWord,false);
  return tmp.empty() ? std::string() : tmp.front();
}
