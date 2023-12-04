// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfLimRamp.H"
#include "vpmDB/FuncPixmaps/limitedramp.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfLIM_RAMP, FmfLimRamp, FmMathFuncBase);

FmfLimRamp::FmfLimRamp()
{
  Fmd_CONSTRUCTOR_INIT(FmfLimRamp);

  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(mySlope                , 1.0, "SLOPE_OF_RAMP");
  FFA_FIELD_INIT(myDelay                , 0.0, "DELAY_OF_RAMP");
  FFA_FIELD_INIT(myRampEnd              , 5.0, "END_OF_RAMP");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfLimRamp::getPixmap() const
{
  return limitedramp;
}


void FmfLimRamp::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Start displacement",AmplitudeDisplacement,FmfLimRamp,retArray);
  M_APPEND_PARAMS("Slope",Slope,FmfLimRamp,retArray);
  M_APPEND_PARAMS("Start of ramp",Delay,FmfLimRamp,retArray);
  M_APPEND_PARAMS("End of ramp",RampEnd,FmfLimRamp,retArray);
}


std::ostream& FmfLimRamp::writeFMF(std::ostream& os)
{
  os <<"FUNC_LIM_RAMP\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfLimRamp::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 4\n");
  fprintf(fp,"  realData =");
  fprintf(fp," %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp," %14.6e",   mySlope.getValue());
  fprintf(fp," %14.6e",   myDelay.getValue());
  fprintf(fp," %14.6e\n", myRampEnd.getValue());
  return 0;
}


bool FmfLimRamp::readAndConnect(std::istream& is, std::ostream&)
{
  FmfLimRamp* obj = new FmfLimRamp();

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


bool FmfLimRamp::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfLimRamp::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfLimRamp::getClassTypeID());
}
