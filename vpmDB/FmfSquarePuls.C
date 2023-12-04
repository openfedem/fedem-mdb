// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FmfSquarePuls.H"
#include "vpmDB/FuncPixmaps/squarepuls.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSQUARE_PULS, FmfSquarePuls, FmMathFuncBase);

FmfSquarePuls::FmfSquarePuls()
{
  Fmd_CONSTRUCTOR_INIT(FmfSquarePuls);

  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(myAmplitude,             1.0, "AMPLITUDE_OF_PULSE");
  FFA_FIELD_INIT(myPeriod,                1.0, "PERIOD_OF_PULSE");
  FFA_FIELD_INIT(myPhaseAngle,            0.0, "PHASE_ANGLE");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfSquarePuls::getPixmap() const
{
  return squarepuls;
}


void FmfSquarePuls::getFunctionVariables(std::vector<FmFuncVariable>& retArray,
					 bool fortranPermuted) const
{
  if (!fortranPermuted)
    M_APPEND_PARAMS("Period",Period,FmfSquarePuls,retArray);
  M_APPEND_PARAMS("Mean value",AmplitudeDisplacement,FmfSquarePuls,retArray);
  M_APPEND_PARAMS("Amplitude",Amplitude,FmfSquarePuls,retArray);
  if (fortranPermuted)
    M_APPEND_PARAMS("Period",Period,FmfSquarePuls,retArray);
  M_APPEND_PARAMS("Phase angle",PhaseAngle,FmfSquarePuls,retArray);
}


std::ostream& FmfSquarePuls::writeFMF(std::ostream& os)
{
  os <<"FUNC_SQUARE_PULS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfSquarePuls::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 4\n");
  fprintf(fp,"  realData =");
  fprintf(fp," %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp," %14.6e",   myAmplitude.getValue());
  fprintf(fp," %14.6e",   myPeriod.getValue());
  fprintf(fp," %14.6e\n", myPhaseAngle.getValue());
  return 0;
}


bool FmfSquarePuls::readAndConnect(std::istream& is, std::ostream&)
{
  FmfSquarePuls* obj = new FmfSquarePuls();

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


bool FmfSquarePuls::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfSquarePuls::cloneLocal(FmBase* obj, int)
{
  if (obj->isOfType(FmfSinusoidal::getClassTypeID())) {
    FmfSinusoidal* sine = (FmfSinusoidal*)obj;
    this->setAmplitude(sine->getAmplitude());
    this->setPeriod(1.0/sine->getFrequency());
    this->setPhaseAngle(sine->getPeriodDelay()*2.0*M_PI);
  }
  else if (!obj->isOfType(FmfSquarePuls::getClassTypeID()))
    return false;

  return true;
}
