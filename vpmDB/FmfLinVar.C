// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfLinVar.H"
#include "vpmDB/FuncPixmaps/linearvar.xpm"

#define BLOCK_SIZE 2


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfLIN_VAR, FmfLinVar, FmfMultiVarBase);


FmfLinVar::FmfLinVar()
{
  Fmd_CONSTRUCTOR_INIT(FmfLinVar);
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfLinVar::getPixmap() const
{
  return linearvar;
}


int FmfLinVar::getBlockSize() const
{
  return BLOCK_SIZE;
}


std::ostream& FmfLinVar::writeFMF(std::ostream& os)
{
  os <<"FUNC_LIN_VAR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


void FmfLinVar::addXYset(double X, double Y)
{
  long pos = binarySearch(X, BLOCK_SIZE);
  insertVal(X, pos);
  insertVal(Y, pos+1);
}


bool FmfLinVar::removeXYset(long place)
{
  removeVal(place*BLOCK_SIZE + 1);
  removeVal(place*BLOCK_SIZE);
  return true;
}


bool FmfLinVar::readAndConnect(std::istream& is, std::ostream&)
{
  FmfLinVar* obj = new FmfLinVar();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmfMultiVarBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmfLinVar::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmfLinVar::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfLinVar::getClassTypeID());
}
