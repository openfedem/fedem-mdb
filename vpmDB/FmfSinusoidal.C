// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaFunctionLib/FFaFunctionManager.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmfWaveSinus.H"
#include "vpmDB/FmfComplSinus.H"
#include "vpmDB/FmfDelayedComplSinus.H"
#include "vpmDB/FmfSquarePuls.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FuncPixmaps/sinus.xpm"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSINUSOIDAL, FmfSinusoidal, FmMathFuncBase);

FmfSinusoidal::FmfSinusoidal()
{
  Fmd_CONSTRUCTOR_INIT(FmfSinusoidal);

  FFA_FIELD_INIT(myFrequency            , 1.0, "FREQUENCY");
  FFA_FIELD_INIT(myPeriodDelay          , 0.0, "PERIOD_DELAY");
  FFA_FIELD_INIT(myAmplitude            , 1.0, "AMPLITUDE");
  FFA_FIELD_INIT(myAmplitudeDisplacement, 0.0, "AMPLITUDE_DISPLACEMENT");
  FFA_FIELD_INIT(myMaxTime              , 0.0, "MAX_TIME");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

static char isStreamlineFunction(const FmfSinusoidal* func)
{
  if (func->getFunctionUse() == FmMathFuncBase::WAVE_FUNCTION)
  {
    if (func->getUserDescription().find("#Stream") != std::string::npos)
      return 8; // Beta feature: Nonlinear streamline wave function
    else if (func->getUserDescription().find("#Stokes5") != std::string::npos)
      return 7; // Beta feature: 5th order Stokes wave function
  }

  return 0;
}


const char* FmfSinusoidal::getFunctionUIName() const
{
  switch (isStreamlineFunction(this)) {
  case  0: return "Sine";
  case  7: return "Stokes5th";
  default: return "Streamline";
  }
}


const char* FmfSinusoidal::getFunctionFsiName() const
{
  switch (isStreamlineFunction(this)) {
  case  0: return "SINUSOIDAL";
  case  7: return "WAVE_STOKES5";
  default: return "WAVE_STREAMLINE";
  }
}


const char** FmfSinusoidal::getPixmap() const
{
  return sinus;
}


void FmfSinusoidal::getFunctionVariables(std::vector<FmFuncVariable>& retArray,
                                         bool waveFuncPermuted) const
{
  if (waveFuncPermuted)
    M_APPEND_PARAMS("Amplitude",Amplitude,FmfSinusoidal,retArray);
  M_APPEND_PARAMS("Frequency",Frequency,FmfSinusoidal,retArray);
  M_APPEND_PARAMS("Delay (fraction of period)",PeriodDelay,FmfSinusoidal,retArray);
  if (!waveFuncPermuted)
    M_APPEND_PARAMS("Amplitude",Amplitude,FmfSinusoidal,retArray);
  M_APPEND_PARAMS("Mean value",AmplitudeDisplacement,FmfSinusoidal,retArray);
  M_APPEND_PARAMS("End",MaxTime,FmfSinusoidal,retArray);
}


bool FmfSinusoidal::hasSmartPoints() const
{
  return isStreamlineFunction(this) ? false : true;
}


void FmfSinusoidal::changedEvent()
{
  myExplData.clear(); // must regenerate the explicit function data
}


bool FmfSinusoidal::initGetValue()
{
  if (!myExplData.empty()) return true;

  if ((myExplType = isStreamlineFunction(this)))
  {
    // Higher order wave function
    myExplData.resize(55);
    myExplData[0] = 1.0/myFrequency.getValue();
    myExplData[1] = 2.0*myAmplitude.getValue();
    myExplData[2] = 2.0*M_PI*myPeriodDelay.getValue();
    double g = FmDB::getMechanismObject()->gravity.getValue().length();
    double d = FmDB::getSeaStateObject()->seaDepth.getValue();
    return FFaFunctionManager::initWaveFunction(myExplType,g,d,myExplData);
  }

  this->FmMathFuncBase::initGetValue();

  // Scale the angle parameters by 2*pi for wave function evaluation.
  // Note: This scaling is also flagged by setting ifunc(3) = 2 in
  // FFaFunctionManager::getValue(). The solver assumes the unscaled values.
  // Also notice the negative sign on EPS=myExplData[2]. It is due to different
  // definition of the function in explicitFunctionsModule (general functions)
  // and waveFunctionsModule (for sea surface evaluation) (kmo 15.07.2015)
  myExplData[1] *=  2.0*M_PI;
  myExplData[2] *= -2.0*M_PI;
  return true;
}


std::ostream& FmfSinusoidal::writeFMF(std::ostream& os)
{
  os <<"FUNC_SINUSOIDAL\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfSinusoidal::printSolverData(FILE* fp)
{
  if (this->getFunctionUse() == WAVE_FUNCTION)
    fprintf(fp,"  nArg = 2\n");

  fprintf(fp,"  realDataSize = %d\n", 5);
  fprintf(fp,"  realData =");
  if (isStreamlineFunction(this))
  {
    fprintf(fp," %14.6e", 1.0/myFrequency.getValue());
    fprintf(fp," %14.6e", 2.0*myAmplitude.getValue());
    fprintf(fp," %14.6e\n", 2.0*M_PI*myPeriodDelay.getValue());
  }
  else
  {
    fprintf(fp," %14.6e", myFrequency.getValue());
    fprintf(fp," %14.6e", myPeriodDelay.getValue());
    fprintf(fp," %14.6e", myAmplitude.getValue());
    fprintf(fp," %14.6e", myAmplitudeDisplacement.getValue());
    fprintf(fp,"\n             %14.6e\n", myMaxTime.getValue());
  }

  return 0;
}


bool FmfSinusoidal::readAndConnect(std::istream& is, std::ostream&)
{
  FmfSinusoidal* obj = new FmfSinusoidal();

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


bool FmfSinusoidal::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfSinusoidal::cloneLocal(FmBase* obj, int)
{
  if (obj->isOfType(FmfWaveSpectrum::getClassTypeID())) {
    // Default conversion of a wave spectrum function
    FmfWaveSpectrum* wave = (FmfWaveSpectrum*)obj;
    this->setAmplitude(0.5*wave->myHs.getValue());
    this->setFrequency(1.0/wave->myTp.getValue());
    this->setMaxTime(0.0);
  }
  else if (obj->isOfType(FmfComplSinus::getClassTypeID())) {
    FmfComplSinus* csine = (FmfComplSinus*)obj;
    this->setAmplitude(csine->getAmplitudeWave1());
    this->setFrequency(csine->getFreqForWave1());
    this->setPeriodDelay(csine->getPeriodDelayWave1());
  }
  else if (obj->isOfType(FmfDelayedComplSinus::getClassTypeID())) {
    FmfDelayedComplSinus* csine = (FmfDelayedComplSinus*)obj;
    this->setAmplitude(csine->getAmplitudeWave1());
    this->setFrequency(csine->getFreqForWave1());
    this->setPeriodDelay(csine->getPeriodDelayWave1());
  }
  else if (obj->isOfType(FmfSquarePuls::getClassTypeID())) {
    FmfSquarePuls* puls = (FmfSquarePuls*)obj;
    this->setAmplitude(puls->getAmplitude());
    this->setFrequency(1.0/puls->getPeriod());
    this->setPeriodDelay(puls->getPhaseAngle()*0.5/M_PI);
  }
  else if (!obj->isOfType(FmfSinusoidal::getClassTypeID()))
    return false;

  return true;
}
