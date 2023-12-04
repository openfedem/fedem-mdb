// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfScale.H"
#include "vpmDB/FuncPixmaps/scale.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSCALE, FmfScale, FmMathFuncBase);

FmfScale::FmfScale(double defScale)
{
  Fmd_CONSTRUCTOR_INIT(FmfScale);

  FFA_FIELD_INIT(myScale, defScale, "SCALE");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfScale::getPixmap() const
{
  return scale;
}


void FmfScale::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Slope",Scale,FmfScale,retArray);
}


std::ostream& FmfScale::writeFMF(std::ostream& os)
{
  os <<"FUNC_SCALE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfScale::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myScale.getValue());
  return 0;
}


bool FmfScale::readAndConnect(std::istream& is, std::ostream&)
{
  FmfScale* obj = new FmfScale();

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


bool FmfScale::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfScale::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfScale::getClassTypeID());
}
