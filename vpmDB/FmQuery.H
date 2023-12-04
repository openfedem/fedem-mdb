// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_QUERY_H
#define FM_QUERY_H

#include "FFaLib/FFaDynCalls/FFaDynCB.H"
#include <map>

class FmModelMemberBase;


/*!
  \brief A Class to manage and store a Query.
  \details The results are not stored, only the "question".
*/

class FmQuery
{
public:
  // Map between Type id and bool to say whether object of type is wanted or not
  std::map<int,bool> typesToFind;

  FFaDynCB2<bool&,FmModelMemberBase*> verifyCB;

  void clear()
  {
    typesToFind.clear();
    verifyCB.erase();
  }
};

#endif
