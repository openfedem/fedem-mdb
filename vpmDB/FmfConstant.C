// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmfConstant.H"
#include "vpmDB/FuncPixmaps/constant.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfCONSTANT, FmfConstant, FmMathFuncBase);

FmfConstant::FmfConstant(double defConst)
{
  Fmd_CONSTRUCTOR_INIT(FmfConstant);

  FFA_FIELD_INIT(myConstant, defConst, "CONSTANT");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfConstant::getPixmap() const
{
  return constant;
}


void FmfConstant::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Constant value",Constant,FmfConstant,retArray);
}


int FmfConstant::getExtrapolationType() const
{
  if (this->getFunctionUse() == GENERAL)
  {
    // Beta feature: Check for ramping of constant general functions
    std::vector<FmEngine*> engines;
    this->getEngines(engines);
    for (FmEngine* engine : engines)
      if (engine->getUserDescription().find("#ramp") != std::string::npos)
        return 3;
  }

  return 0;
}


std::ostream& FmfConstant::writeFMF(std::ostream& os)
{
  os <<"FUNC_CONSTANT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfConstant::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myConstant.getValue());
  return 0;
}


bool FmfConstant::readAndConnect(std::istream& is, std::ostream&)
{
  FmfConstant* obj = new FmfConstant();

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


bool FmfConstant::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfConstant::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfConstant::getClassTypeID());
}
