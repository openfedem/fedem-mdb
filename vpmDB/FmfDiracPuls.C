// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfDiracPuls.H"
#include "vpmDB/FuncPixmaps/diracpuls.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfDIRAC_PULS, FmfDiracPuls, FmMathFuncBase);

FmfDiracPuls::FmfDiracPuls()
{
  Fmd_CONSTRUCTOR_INIT(FmfDiracPuls);

  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(myPulseAmplitude       , 1.0, "AMPLITUDE_OF_PULSE");
  FFA_FIELD_INIT(myPulseWidth           , 0.1, "WIDTH_OF_PULSE");
  FFA_FIELD_INIT(myDelay                , 1.0, "DELAY_OF_PULSE");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfDiracPuls::getPixmap() const
{
  return diracpuls;
}


void FmfDiracPuls::getFunctionVariables(std::vector<FmFuncVariable>& retArray, bool) const
{
  M_APPEND_PARAMS("Start displacement",AmplitudeDisplacement,FmfDiracPuls,retArray);
  M_APPEND_PARAMS("Amplitude",PulseAmplitude,FmfDiracPuls,retArray);
  M_APPEND_PARAMS("Width",PulseWidth,FmfDiracPuls,retArray);
  M_APPEND_PARAMS("Position",Delay,FmfDiracPuls,retArray);
}


std::ostream& FmfDiracPuls::writeFMF(std::ostream& os)
{
  os <<"FUNC_DIRAC_PULS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfDiracPuls::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 4\n");
  fprintf(fp,"  realData =");
  fprintf(fp," %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp," %14.6e",   myPulseAmplitude.getValue());
  fprintf(fp," %14.6e",   myPulseWidth.getValue());
  fprintf(fp," %14.6e\n", myDelay.getValue());
  return 0;
}


bool FmfDiracPuls::readAndConnect(std::istream& is, std::ostream&)
{
  FmfDiracPuls* obj = new FmfDiracPuls();

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


bool FmfDiracPuls::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfDiracPuls::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfDiracPuls::getClassTypeID());
}
