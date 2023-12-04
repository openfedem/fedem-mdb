// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfRamp.H"
#include "vpmDB/FuncPixmaps/ramp.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfRAMP, FmfRamp, FmMathFuncBase);

FmfRamp::FmfRamp()
{
  Fmd_CONSTRUCTOR_INIT(FmfRamp);

  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(mySlope                , 1.0, "SLOPE_OF_RAMP");
  FFA_FIELD_INIT(myDelay                , 0.0, "DELAY_OF_RAMP");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfRamp::getPixmap() const
{
  return ramp;
}


void FmfRamp::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Start displacement",AmplitudeDisplacement,FmfRamp,retArray);
  M_APPEND_PARAMS("Slope",Slope,FmfRamp,retArray);
  M_APPEND_PARAMS("Start of ramp",Delay,FmfRamp,retArray);
}


std::ostream& FmfRamp::writeFMF(std::ostream& os)
{
  os <<"FUNC_RAMP\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfRamp::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 3\n");
  fprintf(fp,"  realData=");
  fprintf(fp," %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp," %14.6e",   mySlope.getValue());
  fprintf(fp," %14.6e\n", myDelay.getValue());
  return 0;
}


bool FmfRamp::readAndConnect(std::istream& is, std::ostream&)
{
  FmfRamp* obj = new FmfRamp();

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


bool FmfRamp::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfRamp::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfRamp::getClassTypeID());
}
