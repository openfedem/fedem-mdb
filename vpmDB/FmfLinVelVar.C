// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfLinVelVar.H"
#include "vpmDB/FuncPixmaps/linderivvar.xpm"

#define BLOCK_SIZE 1


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfLIN_VEL_VAR, FmfLinVelVar, FmfMultiVarBase);

FmfLinVelVar::FmfLinVelVar()
{
  Fmd_CONSTRUCTOR_INIT(FmfLinVelVar);
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfLinVelVar::getPixmap() const
{
  return linderivvar;
}


int FmfLinVelVar::getBlockSize() const
{
  return BLOCK_SIZE;
}


std::ostream& FmfLinVelVar::writeFMF(std::ostream& os)
{
  os <<"FUNC_LIN_VEL_VAR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


void FmfLinVelVar::addIntervalBreak(double val)
{
  this->insertVal(val,this->binarySearch(val,BLOCK_SIZE));
}


bool FmfLinVelVar::removeIntervalBreak(long int pos)
{
  return this->removeVal(pos);
}


bool FmfLinVelVar::readAndConnect(std::istream& is, std::ostream&)
{
  FmfLinVelVar* obj = new FmfLinVelVar();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmMathFuncBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmfLinVelVar::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfLinVelVar::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfLinVelVar::getClassTypeID());
}


void FmfLinVelVar::getXAxisDomain(double&, double& stop) const
{
  if (this->getExtrapolationType() > FmfMultiVarBase::NONE) return;

  const DoubleVec& itsVals = this->myValues.getValue();
  if (itsVals.empty()) return;

  double max = itsVals.front();
  for (double val : itsVals)
    if (val > max) max = val;

  if (stop > max) stop = max;
}
