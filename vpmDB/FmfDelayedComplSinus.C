// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FuncPixmaps/delayedcomplsinus.xpm"
#include "vpmDB/FmfDelayedComplSinus.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfDELAYED_COMPL_SINUS, FmfDelayedComplSinus, FmMathFuncBase);

FmfDelayedComplSinus::FmfDelayedComplSinus()
{
  Fmd_CONSTRUCTOR_INIT(FmfDelayedComplSinus);

  FFA_FIELD_INIT(myFreqForWave1         , 1.0, "FREQUENCY_WAVE_1");
  FFA_FIELD_INIT(myFreqForWave2         , 2.0, "FREQUENCY_WAVE_2");
  FFA_FIELD_INIT(myPeriodDelayWave1     , 0.0, "PERIOD_DELAY_WAVE_1");
  FFA_FIELD_INIT(myPeriodDelayWave2     , 0.0, "PERIOD_DELAY_WAVE_2");
  FFA_FIELD_INIT(myAmplitudeWave1       , 1.0, "AMPLITUDE_WAVE_1");
  FFA_FIELD_INIT(myAmplitudeWave2       , 2.0, "AMPLITUDE_WAVE_2");
  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(myMinTime              , 5.0, "MIN_TIME");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfDelayedComplSinus::getPixmap() const
{
  return delayedcomplsinus;
}


void FmfDelayedComplSinus::getFunctionVariables(std::vector<FmFuncVariable>& retArray,
						bool fortranPermuted) const
{
  if (fortranPermuted)
  {
    M_APPEND_PARAMS("Frequency wave 1",FreqForWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Frequency wave 2",FreqForWave2,FmfDelayedComplSinus,retArray) ;
    M_APPEND_PARAMS("Delay wave 1 (fraction of period)",PeriodDelayWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Delay wave 2 (fraction of period)",PeriodDelayWave2,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Amplitude wave 1",AmplitudeWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Amplitude wave 2",AmplitudeWave2,FmfDelayedComplSinus,retArray);
  }
  else
  {
    M_APPEND_PARAMS("Frequency wave 1",FreqForWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Amplitude wave 1",AmplitudeWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Delay wave 1 (fraction of period)",PeriodDelayWave1,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Frequency wave 2",FreqForWave2,FmfDelayedComplSinus,retArray) ;
    M_APPEND_PARAMS("Amplitude wave 2",AmplitudeWave2,FmfDelayedComplSinus,retArray);
    M_APPEND_PARAMS("Delay wave 2 (fraction of period)",PeriodDelayWave2,FmfDelayedComplSinus,retArray);
  }
  M_APPEND_PARAMS("Mean value",AmplitudeDisplacement,FmfDelayedComplSinus,retArray);
  M_APPEND_PARAMS("Start",MinTime,FmfDelayedComplSinus,retArray);
}


std::ostream& FmfDelayedComplSinus::writeFMF(std::ostream& os)
{
  os <<"FUNC_DELAYED_COMPL_SINUS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfDelayedComplSinus::printSolverData(FILE* fp)
{
  fprintf(fp,"  realDataSize = 8\n");
  fprintf(fp,"  realData =");
  fprintf(fp,           " %14.6e",   myFreqForWave1.getValue());
  fprintf(fp,           " %14.6e",   myFreqForWave2.getValue());
  fprintf(fp,           " %14.6e",   myPeriodDelayWave1.getValue());
  fprintf(fp,           " %14.6e\n", myPeriodDelayWave2.getValue());
  fprintf(fp,"            %14.6e",   myAmplitudeWave1.getValue());
  fprintf(fp,           " %14.6e",   myAmplitudeWave2.getValue());
  fprintf(fp,           " %14.6e",   myAmplitudeDisplacement.getValue());
  fprintf(fp,           " %14.6e\n", myMinTime.getValue());
  return 0;
}


bool FmfDelayedComplSinus::readAndConnect(std::istream& is, std::ostream&)
{
  FmfDelayedComplSinus* obj = new FmfDelayedComplSinus();

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


bool FmfDelayedComplSinus::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfDelayedComplSinus::cloneLocal(FmBase* obj, int)
{
  if (obj->isOfType(FmfSinusoidal::getClassTypeID())) {
    FmfSinusoidal* sine = (FmfSinusoidal*)obj;
    this->setAmplitudeWave1(sine->getAmplitude());
    this->setFreqForWave1(sine->getFrequency());
    this->setPeriodDelayWave1(sine->getPeriodDelay());
  }
  else if (!obj->isOfType(FmfDelayedComplSinus::getClassTypeID()))
    return false;

  return true;
}
