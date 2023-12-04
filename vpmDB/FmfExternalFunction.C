// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmfExternalFunction.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <algorithm>


Fmd_DB_SOURCE_INIT(FcfEXTERNAL_FUNCTION, FmfExternalFunction, FmMathFuncBase);


FmfExternalFunction::FmfExternalFunction()
{
  Fmd_CONSTRUCTOR_INIT(FmfExternalFunction);

  // Find the unique channel index of this function
  int idx = 1;
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,FmfExternalFunction::getClassTypeID());
  while (std::find_if(objs.begin(), objs.end(),
                      [idx](FmModelMemberBase* obj) {
                        return static_cast<FmfExternalFunction*>(obj)->channel.getValue() == idx;
                      }) != objs.end()) ++idx;

  FFA_FIELD_INIT(channel,idx,"CHANNEL_INDEX");
  FFA_FIELD_INIT(scale,1.0,"SCALE_FACTOR");
  FFA_FIELD_INIT(shift,0.0,"VERTICAL_SHIFT");
}


int FmfExternalFunction::getExtrapolationType() const
{
  if (this->getFunctionUse() == GENERAL)
  {
    // Beta feature: Switch off ramping for specific function
    std::vector<FmEngine*> engines;
    this->getEngines(engines);
    for (FmEngine* engine : engines)
      if (engine->getUserDescription().find("#noramp") != std::string::npos)
        return 0;
  }

  return 3; // By default all external functions are ramped
}


std::ostream& FmfExternalFunction::writeFMF(std::ostream& os)
{
  os <<"FUNC_EXTERNAL_FUNCTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfExternalFunction::printSolverData(FILE* fp)
{
  fprintf(fp,"  channel = %d\n", channel.getValue());
  fprintf(fp,"  realDataSize = 2\n");
  fprintf(fp,"  realData = %14.6e", shift.getValue());
  fprintf(fp,            " %14.6e\n", scale.getValue());

  return 0;
}


bool FmfExternalFunction::readAndConnect(std::istream& is, std::ostream&)
{
  FmfExternalFunction* obj = new FmfExternalFunction();

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


bool FmfExternalFunction::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfExternalFunction::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfExternalFunction::getClassTypeID());
}
