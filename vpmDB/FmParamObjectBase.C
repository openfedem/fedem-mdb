// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmParamObjectBase.H"
#include "vpmDB/FuncPixmaps/blank.xpm"


Fmd_DB_SOURCE_INIT(FcPARAM_OBJECT_BASE,FmParamObjectBase,FmStructPropertyBase);


FmParamObjectBase::FmParamObjectBase() : FmStructPropertyBase()
{
  Fmd_CONSTRUCTOR_INIT(FmParamObjectBase);
}


const char** FmParamObjectBase::getPixmap() const
{
  return blank;
}


bool FmParamObjectBase::localParse(const char* keyWord, std::istream& activeStatement,
				   FmParamObjectBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}
