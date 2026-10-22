// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmTypeCheck.H
  \brief Type checking control.
  \details This file contains some vpmDB specific macros for type checking.
  They are used instead of the plain type checking macros,
  when making new Mechanism database objects.
  \sa FFaTypeCheck.H
*/

#ifndef FM_TYPE_CHECK_H
#define FM_TYPE_CHECK_H

#include "FFaLib/FFaTypeCheck/FFaTypeCheck.H"


//! \brief Macro enabling type checking for a class.
#define Fmd_DB_HEADER_INIT()							\
										\
Fmd_HEADER_INIT();								\
public:										\
  static bool parentParse(const char* keyWord, std::istream& activeStatement, void* obj); \
  bool cloneInherited(FmBase* obj, int depth)

//! \brief Macro with type checking implementations.
#define Fmd_DB_SOURCE_INIT(clName, clType, prClType)				\
										\
bool clType::cloneInherited(FmBase* obj, int depth)				\
{										\
  bool status = true;								\
  if (clType::typeID != prClType::getClassTypeID())				\
    status = this->prClType::cloneInherited(obj,depth);				\
										\
  return this->clType::cloneLocal(obj,depth) && status;				\
}										\
										\
bool clType::parentParse(const char* keyWord, std::istream& activeStatement, void* obj) \
{ 										\
  return prClType::localParse(keyWord,activeStatement,static_cast<prClType*>(obj)); \
}										\
										\
Fmd_SOURCE_INIT(clName, clType, prClType)

#endif
