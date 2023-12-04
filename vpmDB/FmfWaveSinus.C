// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaFunctionLib/FFaFunctionManager.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmfSinusoidal.H"
#include "vpmDB/FmfWaveSinus.H"
#include "vpmDB/FuncPixmaps/jonswap.xpm"


/**********************************************************************
 *
 * Class constructors
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfWAVE_SINUS, FmfWaveSinus, FmMathFuncBase);
Fmd_DB_SOURCE_INIT(FcfWAVE_SPECTRUM, FmfWaveSpectrum, FmfWaveSinus);


FmfWaveSinus::FmfWaveSinus(int dof)
{
  Fmd_CONSTRUCTOR_INIT(FmfWaveSinus);

  FFA_FIELD_INIT(lDof, dof, "LOCAL_DOF");
}


FmfWaveSpectrum::FmfWaveSpectrum()
{
  Fmd_CONSTRUCTOR_INIT(FmfWaveSpectrum);

  FFA_FIELD_INIT(myHs    ,             4.0, "SIGNIFICANT_WAVE_HEIGHT");
  FFA_FIELD_INIT(myTp    ,            14.0, "PEAK_PERIOD");
  FFA_FIELD_DEFAULT_INIT(myTrange,          "PERIOD_RANGE");
  FFA_FIELD_INIT(autoCalcTrange,      true, "AUTO_CALC_PERIOD_RANGE");
  FFA_FIELD_INIT(nComp   ,             400, "WAVE_COMPONENTS");
  FFA_FIELD_INIT(nDir    ,               1, "WAVE_DIRECTIONS");
  FFA_FIELD_INIT(sprExp  ,               2, "SPREADING_EXPONENT");
  FFA_FIELD_INIT(spectrum,         JONSWAP, "WAVE_SPECTRUM");
  FFA_FIELD_INIT(rndPhase,            true, "RANDOM_PHASE");
  FFA_FIELD_INIT(myRandomSeed,           0, "RANDOM_SEED");
  FFA_FIELD_INIT(myPeakedness,         3.3, "SPECTRAL_PEAKEDNESS");
  FFA_FIELD_INIT(autoCalcPeakedness, false, "AUTO_CALC_SPECTRAL_PEAKEDNESS");

  // Calculate default period range
  this->deriveTrange();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char* FmfWaveSpectrum::getFunctionUIName() const
{
  switch (spectrum.getValue()) {
  case JONSWAP: return "JONSWAP wave spectrum";
  case PiersonMoskowitz: return "Pierson-Moskowitz wave spectrum";
  }
  return "Wave spectrum";
}


const char* FmfWaveSinus::getFunctionFsiName() const
{
  FmEngine* engine = NULL;
  FmVesselMotion* owner = NULL;
  if (this->hasReferringObjs(engine,"myFunction"))
    if (engine->hasReferringObjs(owner,"motionEngine"))
      if (!owner->waveFunction.isNull())
	return "WAVE_SINUS";

  // Let this be a constant function if no wave function is attached
  return "CONSTANT";
}


const char* FmfWaveSpectrum::getFunctionFsiName() const
{
  if (this->getUserDescription().find("#EmbeddedStream") != std::string::npos)
    return "WAVE_EMBEDDED"; // Beta feature: Embedded streamline function

  return "WAVE_SINUS";
}


const char** FmfWaveSpectrum::getPixmap() const
{
  return jonswap_xpm;
}


void FmfWaveSpectrum::changedEvent()
{
  myExplData.clear(); // must regenerate explicit function data
}


bool FmfWaveSinus::initGetValue()
{
  myExplType = FFaFunctionManager::getTypeID(this->getFunctionFsiName());
  if (myExplData.size() > 2) return true;

  FFaMsg::list("ERROR: Internal wave function "+ this->getInfoString() +
	       " has not been initialized.\n",true);
  return false;
}


bool FmfWaveSpectrum::initGetValue()
{
  if (!myExplData.empty()) return true;

  FFaString fDesc = this->getUserDescription();
  int iop = 2*spectrum.getValue() + (rndPhase.getValue() ? 6 : 5);
  if (fDesc.hasSubString("#Old"))
    iop -= 4; // Using the old JONSWAP spectrum implementation

  myExplType = FFaFunctionManager::getTypeID(this->getFunctionFsiName());

  mainDirData.clear();

  myIntData = {
    myExplType,
    this->getExtrapolationType(),
    nComp.getValue(),
    nDir.getValue()
  };

  myExplData = {
    myHs.getValue(),
    myTp.getValue(),
    myTrange.getValue().second,
    myTrange.getValue().first,
    myPeakedness.getValue()
  };

  if (myExplType == 9)
  {
    double sdata[32];
    myIntData[3] = (fDesc.getDoublesAfter("#EmbeddedStream",32,sdata)-2)/3;
    if (myIntData[3] > 0)
    {
      // Insert embedded streamline waves at the specified locations
      myExplData.insert(myExplData.end(),sdata,sdata+2+3*myIntData[3]);
      double g = FmDB::getMechanismObject()->gravity.getValue().length();
      double d = FmDB::getSeaStateObject()->seaDepth.getValue();
      return FFaFunctionManager::initWaveFunction(iop,nComp.getValue(),
						  myRandomSeed.getValue(),g,d,
						  myIntData,myExplData);
    }
  }
  else if (fDesc.hasSubString("#OmegaRange"))
  {
    double sdata[2];
    if (fDesc.getDoublesAfter("#OmegaRange",2,sdata) == 2)
      if (sdata[0] >= 0.0 && sdata[1] > sdata[0])
      {
        // A negative value is used to flag that a frequency range is specified
        myExplData[2] =  sdata[0];
        myExplData[3] = -sdata[1];
      }
  }

  return FFaFunctionManager::initWaveFunction(iop,nComp.getValue(),
					      nDir.getValue(),
					      sprExp.getValue(),
					      myRandomSeed.getValue(),
					      myExplData);
}


double FmfWaveSpectrum::getValue(double x, int& ierr) const
{
  int nDir = myIntData.size() > 3 ? myIntData[3] : 1;
  if (nDir <= 1)
    return FFaFunctionManager::getValue(this->getBaseID(),
					myIntData,myExplData,x,ierr);

  // This wave function has spreading.
  // Evaluate the wave train along the X-axis only.
  if (mainDirData.empty())
  {
    size_t n = myExplData.size() / nDir;
    std::vector<double>::const_iterator it = myExplData.begin() + n*(nDir-1)/2;
    mainDirData.insert(mainDirData.begin(),it,it+n);
  }
  return FFaFunctionManager::getValue(this->getBaseID(),
				      myIntData,mainDirData,x,ierr);
}


double FmfWaveSpectrum::getValue(double g, double d,
                                 const FaVec3& X, double t) const
{
  return FFaFunctionManager::getWaveValue(myIntData,myExplData,
                                          g,d,X,t,myExplType);
}


std::ostream& FmfWaveSinus::writeFMF(std::ostream& os)
{
  os <<"FUNC_WAVE_SINUS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


std::ostream& FmfWaveSpectrum::writeFMF(std::ostream& os)
{
  os <<"FUNC_WAVE_SPECTRUM\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfWaveSinus::printSolverData(FILE* fp)
{
  FmEngine* engine = NULL;
  FmVesselMotion* owner = NULL;
  if (!this->hasReferringObjs(engine,"myFunction")) return 1;
  if (!engine->hasReferringObjs(owner,"motionEngine")) return 1;

  if (owner->waveFunction.isNull())
  {
    // Write out a constant zero function istead if no wave function attached
    ListUI <<" ==> WARNING: No wave function is attached to "
           << owner->getIdString(true) <<".\n     "<< engine->getIdString(true)
           <<" will therefore be identically zero.\n";
    fprintf(fp,"  realDataSize = 1\n");
    fprintf(fp,"  realData = 0.0\n");
    return 0;
  }

  std::string fileName = owner->getActualRAOFileName();
  FFaFilePath::makeItAbsolute(fileName,relPathCorrection);

  fprintf(fp,"  fileName = '%s'\n", fileName.c_str());
  fprintf(fp,"  channel = %d\n", lDof.getValue());
  fprintf(fp,"  waveDir = %d\n", owner->waveDir.getValue());
  fprintf(fp,"  waveId = %d\n", owner->waveFunction->getBaseID());

  return 0;
}


int FmfWaveSpectrum::printSolverData(FILE* fp)
{
  int nEmbStr = 0;
  double omega0 = 0.0, omega1 = 0.0, sdata[32];
  FFaString fDesc = this->getUserDescription();
  if (fDesc.hasSubString("#EmbeddedStream"))
  {
    nEmbStr = (fDesc.getDoublesAfter("#EmbeddedStream",30,sdata)-2)/3;
    if (nEmbStr > 0) fprintf(fp,"  channel = %d\n", nEmbStr);
  }
  else if (fDesc.hasSubString("#OmegaRange"))
    if (fDesc.getDoublesAfter("#OmegaRange",2,sdata) == 2)
      if (sdata[0] >= 0.0 && sdata[1] > sdata[0])
      {
        nEmbStr = -2;
        omega0 = sdata[0];
        omega1 = sdata[1];
      }

  if (nEmbStr >= 0)
  {
    double Thigh = myTrange.getValue().second;
    double Tlow  = myTrange.getValue().first;
    if (Tlow <= 0.0 || Tlow >= Thigh)
    {
      ListUI <<"ERROR: Invalid period range for "<< this->getIdString(true)
             <<"\n       Trange=["<< Tlow <<","<< Thigh <<"]\n";
      return 1;
    }
    else
    {
      omega0 = 2.0*M_PI/Thigh;
      omega1 = 2.0*M_PI/Tlow;
    }
  }

  int iop = 2*spectrum.getValue() + (rndPhase.getValue() ? 6 : 5);
  if (fDesc.hasSubString("#Old"))
    iop -= 4; // Using the old JONSWAP spectrum implementation

  int waveDirs = iop > 4 && nDir.getValue() > 1 ? nDir.getValue() : 1;
  if (waveDirs > 1 && sprExp.getValue() > 0) iop += 10*sprExp.getValue();
  double domega = (omega1-omega0)/nComp.getValue();
  fprintf(fp,"  realDataSize = %d\n", 3*nComp.getValue()*waveDirs);
  fprintf(fp,"  realData = %14.6e",   myHs.getValue());
  fprintf(fp,            " %14.6e",   myTp.getValue());
  fprintf(fp,            " %14.6e",   omega0);
  fprintf(fp,            " %14.6e",   domega);
  fprintf(fp,            " %14.6e\n", myPeakedness.getValue());
  if (nEmbStr > 0)
    fprintf(fp,"             %14.6e %14.6e\n", sdata[0],sdata[1]);
  for (int i = 0; i < nEmbStr; i++)
    fprintf(fp,"             %14.6e %14.6e %14.6e\n",
            sdata[3*i+2],sdata[3*i+3],sdata[3*i+4]);

  fprintf(fp,"  extrapolationType = %d\n", iop);
  fprintf(fp,"  waveDir = %d\n", waveDirs);
  fprintf(fp,"  seed = %d\n", myRandomSeed.getValue());

  return 0;
}


bool FmfWaveSinus::readAndConnect(std::istream& is, std::ostream&)
{
  FmMathFuncBase* obj = new FmfWaveSinus();

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


bool FmfWaveSpectrum::readAndConnect(std::istream& is, std::ostream&)
{
  FmfWaveSpectrum* obj = new FmfWaveSpectrum();

  // Obsolete fields
  FFaObsoleteField<double> longesT;
  FFaObsoleteField<double> shortesT;
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(longesT, "LONGEST_PERIOD", obj);
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(shortesT, "SHORTEST_PERIOD", obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
    {
      if (strcmp(keyWord,"MEAN_PERIOD") == 0)
        FmMathFuncBase::localParse("PEAK_PERIOD", activeStatement, obj);
      else
        FmMathFuncBase::localParse(keyWord, activeStatement, obj);
    }
  }

  FFA_OBSOLETE_FIELD_REMOVE("LONGEST_PERIOD", obj);
  FFA_OBSOLETE_FIELD_REMOVE("SHORTEST_PERIOD", obj);

  // Update from old model file
  if (longesT.wasOnFile() && shortesT.wasOnFile())
  {
    obj->myTrange.setValue(FmRange(shortesT.getValue(),longesT.getValue()));
    obj->autoCalcTrange.setValue(false);
  }

  obj->connect();
  return true;
}


bool FmfWaveSinus::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfWaveSpectrum::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfWaveSinus::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfWaveSinus::getClassTypeID());
}


bool FmfWaveSpectrum::cloneLocal(FmBase* obj, int)
{
  if (obj->isOfType(FmfSinusoidal::getClassTypeID())) {
    // Default conversion of a regular wave
    FmfSinusoidal* sine = (FmfSinusoidal*)obj;
    myHs.setValue(2.0*sine->getAmplitude());
    myTp.setValue(1.0/sine->getFrequency());
    this->deriveTrange();
  }
  else if (!obj->isOfType(FmfWaveSpectrum::getClassTypeID()))
    return false;

  return true;
}


const FmRange& FmfWaveSpectrum::deriveTrange()
{
  // Calculate Tmin and Tmax (ref. Paul Anton Letnes 2013-01-04, Bug #174)
  double m0cut = 0.0025;
  double Ag    = 1.0 - 0.287*log(myPeakedness.getValue());
  double Thigh = myTp.getValue() * pow(log(Ag/m0cut) / 1.25, 0.25);
  double Tlow  = myTp.getValue() * pow((4.0*m0cut) / (5.0*Ag), 0.25);

  myTrange.setValue(FmRange(Tlow,Thigh));
  return myTrange.getValue();
}


double FmfWaveSpectrum::deriveSpectralPeakedness()
{
  double gamma = 1.0;
  if (myHs.getValue() > 0.0)
  {
    double TpDivSqrtHs = myTp.getValue() / sqrt(myHs.getValue());
    if (TpDivSqrtHs <= 3.6)
      gamma = 5.0;
    else if (TpDivSqrtHs < 5.0)
      gamma = exp(5.75 - 1.15 * TpDivSqrtHs);
  }

  myPeakedness.setValue(gamma);
  return gamma;
}


void FmfWaveSpectrum::initAfterParse()
{
  // Note: Must auto-calculate spectral peakedness before period range,
  // because the latter also depends on the former
  if (autoCalcPeakedness.getValue())
    this->deriveSpectralPeakedness();

  if (autoCalcTrange.getValue())
    this->deriveTrange();
}
