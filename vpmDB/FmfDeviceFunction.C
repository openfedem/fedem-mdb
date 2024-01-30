// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaFunctionLib/FFaFunctionManager.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FiDeviceFunctions/FiDeviceFunctionFactory.H"
#include "vpmDB/FuncPixmaps/userDefinedWaveSpectrum.xpm"
#include "vpmDB/FmfDeviceFunction.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFileSys.H"


Fmd_DB_SOURCE_INIT(FcfDEVICE_FUNCTION, FmfDeviceFunction, FmMathFuncBase);


FmfDeviceFunction::FmfDeviceFunction(const char* fname, const char* cname)
{
  Fmd_CONSTRUCTOR_INIT(FmfDeviceFunction);

  FFA_FIELD_INIT(zeroAdjust,false,"ZERO_ADJUST");
  FFA_FIELD_INIT(verticalShift,0.0,"VERTICAL_SHIFT");
  FFA_FIELD_INIT(scaleFactor,1.0,"SCALE_FACTOR");
  FFA_FIELD_INIT(randomSeed,0,"RANDOM_SEED");

  FFA_FIELD_DEFAULT_INIT(deviceName,"DEVICE_NAME");
  FFA_FIELD_DEFAULT_INIT(channel,"CHANNEL");

  FFA_REFERENCE_FIELD_INIT(fileReferenceField,fileReference,"FILE_REFERENCE");
  fileReference.setPrintIfZero(false);

  if (fname) deviceName.setValue(fname);
  if (cname) channel.setValue(cname);

  fileInd = chanInd = -1;
  myHs = myTz = 0.0;
}


const char* FmfDeviceFunction::getFunctionUIName() const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return "User defined wave spectrum";

  return "Poly line from file";
}


const char** FmfDeviceFunction::getPixmap() const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return userDefinedWaveSpectrum_xpm;

  // No pixmap yet for "poly line form file" functions
  return this->FmMathFuncBase::getPixmap();
}


static int countSineWaves(const std::string& waveFile)
{
  FILE* fp = fopen(waveFile.c_str(),"r");
  if (!fp) return 0;

  // Count the number of sine waves in the file
  int nWave = 0;
  char buf[128];
  if (fgets(buf,128,fp))
  {
    // The first line does not count if it contains # columns
    if (strncmp(buf,"#ncol",5) && strncmp(buf,"#NCOL",5))
      nWave++;
    while (fgets(buf,128,fp))
      nWave++;
  }

  fclose(fp);

  return nWave;
}


bool FmfDeviceFunction::initGetValue()
{
  if (!this->checkFileValidity()) return false;

  std::string fileName(this->getActualDeviceName(true));

  if (this->isUsedAs(WAVE_FUNCTION))
  {
    // Count the number of sine waves in file
    int nWaves = countSineWaves(fileName);
    int rnSeed = randomSeed.getValue();
    myExplType = 4; // WAVE_SINUS_p
    if (!FFaFunctionManager::initWaveFunction(fileName,nWaves,rnSeed,myExplData))
      return false;

    // Calculate the significant wave height (Hs) and mean wave period (Tz)
    double m0 = 0.0, m2 = 0.0, dOmega = 0;
    for (int j = 0; j < nWaves; j++)
    {
      double Aj = myExplData[3*j];
      double omega = myExplData[3*j+1];
      if (j > 0)
	dOmega += omega - myExplData[3*j-2];
      m0 += Aj*Aj;
      m2 += omega*omega*Aj*Aj;
    }
    if (nWaves > 2)
      dOmega /= double(nWaves-1);
    m0 *= 0.5/dOmega;
    m2 *= 0.5/dOmega;
    myHs = 0.5*sqrt(m0);
    myTz = 2.0*M_PI*sqrt(m0/m2);
  }

  fileInd = FiDeviceFunctionFactory::instance()->open(fileName);
  chanInd = FiDeviceFunctionFactory::instance()->channelIndex(fileInd,channel.getValue());

  return true;
}


double FmfDeviceFunction::getValue(double x, int& ierr) const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return this->FmMathFuncBase::getValue(x,ierr);

  if (fileInd <= 0)
  {
    ierr = -1;
    return 0.0;
  }

  return FiDeviceFunctionFactory::instance()->getValue(fileInd,
						       x, ierr, chanInd,
						       zeroAdjust.getValue() ? 1 : 0,
						       verticalShift.getValue(),
						       scaleFactor.getValue());
}


int FmfDeviceFunction::getSmartPoints(double start, double stop,
				      std::vector<double>& x, std::vector<double>& y)
{
  if (start > stop)
    return -1;
  else if (!this->initGetValue())
    return -2;

  if (!FiDeviceFunctionFactory::instance()->getValues(fileInd,
						      start, stop, x, y, chanInd,
						      zeroAdjust.getValue() ? 1 : 0,
						      verticalShift.getValue(),
						      scaleFactor.getValue()))
    return -4;
  else
    return 0;
}


std::ostream& FmfDeviceFunction::writeFMF(std::ostream& os)
{
  os <<"FUNC_DEVICE_FUNCTION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmfDeviceFunction::isLegalSprDmpFunc() const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return false;

  return this->FmMathFuncBase::isLegalSprDmpFunc();
}


bool FmfDeviceFunction::hasSmartPoints() const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return false;

  return this->FmMathFuncBase::hasSmartPoints();
}


int FmfDeviceFunction::printSolverData(FILE* fp)
{
  int chanIdx = this->checkFileValidity();
  if (chanIdx == 0) return 1;

  std::string fileName(this->getActualDeviceName(false));
  if (FFaFilePath::isRelativePath(fileName) && !relPathCorrection.empty())
    fileName = relPathCorrection + fileName;
  fprintf(fp,"  fileName = '%s'\n", fileName.c_str());

  if (chanIdx > 0) fprintf(fp,"  channel = %d\n", chanIdx);

  if (chanIdx == -2)
  {
    int nWaves = countSineWaves(this->getActualDeviceName(true));
    int rnSeed = randomSeed.getValue();
    fprintf(fp,"  realDataSize = %d\n", 3*nWaves);
    fprintf(fp,"  seed = %d\n", rnSeed);
  }
  else
  {
    fprintf(fp,"  realDataSize = 3\n");
    fprintf(fp,"  realData = %14.6e", verticalShift.getValue());
    fprintf(fp,            " %14.6e", scaleFactor.getValue());
    fprintf(fp,            " %14.6e\n", zeroAdjust.getValue() ? 1.0 : 0.0);
  }
  return 0;
}


int FmfDeviceFunction::printSolverEntry(FILE* fp)
{
  if (!this->isUsedAs(WAVE_FUNCTION))
    return this->FmMathFuncBase::printSolverEntry(fp);

  fprintf(fp,"&FUNCTION\n");
  this->printID(fp);
  fprintf(fp,"  type = 'WAVE_SINUS'\n");
  int err = this->printSolverData(fp);
  fprintf(fp,"/\n\n");
  return err;
}


bool FmfDeviceFunction::readAndConnect(std::istream& is, std::ostream&)
{
  FmfDeviceFunction* obj = new FmfDeviceFunction();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmMathFuncBase::localParse(keyWord, activeStatement, obj);
  }

  FFaFilePath::checkName(obj->deviceName.getValue());
  if (obj->channel.getValue() == "Not set")
    obj->channel.setValue("");

  obj->connect();
  return true;
}


bool FmfDeviceFunction::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfDeviceFunction::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfDeviceFunction::getClassTypeID());
}


/*!
  \return channel index if multi-channel file, and the file is exists.
  \return 0 if no file, or invalid channel for a multi-channel file.
  \return -1 if single-channel file, and the file exists.
  \return -2 if wave data file, and the file exists.
*/

int FmfDeviceFunction::checkFileValidity()
{
  std::string fileName(this->getActualDeviceName(true));
  if (fileName.empty())
  {
    ListUI <<"ERROR: No file specified for "<< this->getIdString() <<".\n";
    return 0;
  }

  if (this->isUsedAs(WAVE_FUNCTION))
  {
    if (FmFileSys::isFile(fileName))
      return -2;

    ListUI <<"EXTERNAL FUNCTION ERROR: Could not open file "<< fileName <<".\n";
    return 0;
  }

  int chanIndex = -1;
  int fileType  = FiDeviceFunctionFactory::identify(fileName);
  int fileIndex = FiDeviceFunctionFactory::instance()->open(fileName);
  if (fileIndex <= 0)
  {
    ListUI <<"EXTERNAL FUNCTION ERROR: Could not open file "<< fileName <<".\n";
    return 0;
  }
  else if (fileType == ASC_MC_FILE || fileType == RPC_TH_FILE)
  {
    // TODO: Store chanIndex (and maybe fileIndex) as a class member(s),
    // to avoid file opening and channel searching more than neccesary.
    chanIndex = FiDeviceFunctionFactory::instance()->channelIndex(fileIndex,channel.getValue());
    if (chanIndex <= 0)
    {
      chanIndex = 0;
      ListUI <<"EXTERNAL FUNCTION ERROR: Could not locate channel '"
	     << channel.getValue()
	     <<"'\n                         in file "<< fileName <<".\n";
    }
  }

  FiDeviceFunctionFactory::instance()->close(fileIndex);
  return chanIndex;
}


std::string FmfDeviceFunction::getActualDeviceName(bool fullPath) const
{
  std::string fileName;
  if (fileReference.getPointer())
    fileName = fileReference->fileName.getValue();
  else
    fileName = deviceName.getValue();

  if (!fullPath || fileName.empty()) return fileName;

  std::string modelFilePath(FmDB::getMechanismObject()->getAbsModelFilePath());
  return FFaFilePath::makeItAbsolute(fileName,modelFilePath);
}


bool FmfDeviceFunction::getDevice(std::string& fileName, std::string& channelName) const
{
  fileName = deviceName.getValue();
  switch (FiDeviceFunctionFactory::identify(this->getActualDeviceName(true)))
    {
    case RPC_TH_FILE:
    case ASC_MC_FILE:
      if (channel.getValue().empty())
	channelName = "Not set";
      else
	channelName = channel.getValue();
      return true;
    default:
      channelName = "Not set";
      return false;
    }
}


bool FmfDeviceFunction::setDevice(const std::string& fileName, const std::string& channelName)
{
  bool changed = deviceName.setValue(fileName);
  if (channelName.empty() || channelName == "Not set")
    changed |= channel.setValue("");
  else
    changed |= channel.setValue(channelName);

  return changed;
}


bool FmfDeviceFunction::setFileReference(FmFileReference* ref)
{
  if (ref == fileReference.getPointer())
    return false;

  fileReference.setPointer(ref);
  return true;
}


bool FmfDeviceFunction::getChannelList(std::vector<std::string>& channels) const
{
  return FiDeviceFunctionFactory::getChannelList(this->getActualDeviceName(true),channels);
}


void FmfDeviceFunction::close()
{
  if (fileInd > 0)
    FiDeviceFunctionFactory::instance()->close(fileInd);
}
