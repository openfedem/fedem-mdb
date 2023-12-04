// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfStep.H"
#include "vpmDB/FuncPixmaps/step.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSTEP, FmfStep, FmMathFuncBase);

FmfStep::FmfStep()
{
  Fmd_CONSTRUCTOR_INIT(FmfStep);

  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(myAmplitudeStep,         1.0, "AMPLITUDE_OF_STEP");
  FFA_FIELD_INIT(myDelayStep,             0.0, "DELAY_OF_STEP");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfStep::getPixmap() const
{
  return step;
}


void FmfStep::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Start displacement",AmplitudeDisplacement,FmfStep,retArray);
  M_APPEND_PARAMS("Amplitude",AmplitudeStep,FmfStep,retArray);
  M_APPEND_PARAMS("Start of step",DelayStep,FmfStep,retArray);
}


std::ostream& FmfStep::writeFMF(std::ostream& os)
{
  os <<"FUNC_STEP\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfStep::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 3\n");
  fprintf(fp,"  realData =");
  fprintf(fp," %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp," %14.6e",   myAmplitudeStep.getValue());
  fprintf(fp," %14.6e\n", myDelayStep.getValue());
  return 0;
}


bool FmfStep::readAndConnect(std::istream& is, std::ostream&)
{
  FmfStep* obj = new FmfStep();

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


bool FmfStep::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfStep::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfStep::getClassTypeID());
}
