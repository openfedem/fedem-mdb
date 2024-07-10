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

#define DFF FiDeviceFunctionFactory::instance()

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

  FFA_REFERENCE_FIELD_INIT(fileRefField,fileRef,"FILE_REFERENCE");
  fileRef.setPrintIfZero(false);

  if (fname) deviceName.setValue(fname);
  if (cname) channel.setValue(cname);

  fileInd = -1;
  chanInd = 0;
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
  if (this->checkFileValidity(true) == 0)
    return false;

  if (this->isUsedAs(WAVE_FUNCTION))
  {
    // Count the number of sine waves in file
    std::string fName(this->getActualDeviceName(true));
    int nWaves = countSineWaves(fName);
    int rnSeed = randomSeed.getValue();
    myExplType = 4; // WAVE_SINUS_p
    if (!FFaFunctionManager::initWaveFunction(fName,nWaves,rnSeed,myExplData))
      return false;

    // Calculate the significant wave height (Hs) and mean wave period (Tz)
    double m0 = 0.0, m2 = 0.0, dOmega = 0.0;
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
      dOmega /= static_cast<double>(nWaves-1);
    m0 *= 0.5/dOmega;
    m2 *= 0.5/dOmega;
    myHs = 0.5*sqrt(m0);
    myTz = 2.0*M_PI*sqrt(m0/m2);
  }

  return true;
}


double FmfDeviceFunction::getValue(double x, int& ierr) const
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return this->FmMathFuncBase::getValue(x,ierr);

  if (fileInd == 0)
  {
    ierr = -1;
    return 0.0;
  }

  return DFF->getValue(fileInd, x, ierr, chanInd,
                       zeroAdjust.getValue() ? 1 : 0,
                       verticalShift.getValue(),
                       scaleFactor.getValue());
}


int FmfDeviceFunction::getSmartPoints(double start, double stop,
                                      DoubleVec& x, DoubleVec& y)
{
  if (this->isUsedAs(WAVE_FUNCTION))
    return -4;

  if (start > stop)
    return -1;
  else if (!this->initGetValue())
    return -2;

  return DFF->getValues(fileInd, start, stop, x, y, chanInd,
                        zeroAdjust.getValue() ? 1 : 0,
                        verticalShift.getValue(),
                        scaleFactor.getValue()) ? 0 : -4;
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


int FmfDeviceFunction::checkFunctions()
{
  int errCount = 0;
  std::vector<FmfDeviceFunction*> allDevFuncs;
  FmDB::getAllDeviceFunctions(allDevFuncs);

  for (FmfDeviceFunction* func : allDevFuncs)
    if (func->checkFileValidity() == 0)
      errCount++;

  return errCount;
}


int FmfDeviceFunction::printSolverData(FILE* fp)
{
  std::string fileName(this->getActualDeviceName());
  if (FFaFilePath::isRelativePath(fileName) && !relPathCorrection.empty())
    fileName = relPathCorrection + fileName;
  fprintf(fp,"  fileName = '%s'\n", fileName.c_str());

  if (chanInd > 0)
    fprintf(fp,"  channel = %d\n", chanInd);

  if (chanInd == -2)
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
  \return channel index if multi-channel file, and the file exists.
  \return 0 if no file, or invalid channel for a multi-channel file.
  \return -1 if single-channel file, and the file exists.
  \return -2 if wave data file, and the file exists.
*/

int FmfDeviceFunction::checkFileValidity(bool keepOpen)
{
  if (fileInd >= 0)
    return chanInd;

  std::string fileName(this->getActualDeviceName(true));
  if (fileName.empty())
  {
    ListUI <<"ERROR: No file specified for "<< this->getIdString() <<".\n";
    return chanInd;
  }

  if (keepOpen || !this->isUsedAs(WAVE_FUNCTION))
    if ((fileInd = DFF->open(fileName)) <= 0)
    {
      ListUI <<"ERROR: Could not open device function file "<< fileName <<".\n";
      return chanInd;
    }

  if (this->isUsedAs(WAVE_FUNCTION))
  {
    if (keepOpen || FmFileSys::isFile(fileName))
      chanInd = -2;
    else
      ListUI <<"ERROR: Could not open wave function file "<< fileName <<".\n";
    return chanInd;
  }

  switch (FiDeviceFunctionFactory::identify(fileName))
    {
    case ASC_MC_FILE:
    case RPC_TH_FILE:
      if (!(chanInd = DFF->channelIndex(fileInd,channel.getValue())))
        ListUI <<"ERROR: Could not locate channel '"<< channel.getValue()
               <<"'\n       in device function file "<< fileName <<".\n";
      break;

    case DAC_FILE:
    case ASC_FILE:
    case EXT_FUNC:
      chanInd = -1; // existing single-channel file
      break;

    default:
      // Non-existing or invalid file type
      ListUI <<"ERROR: Invalid device function file "<< fileName <<".\n";
      break;
    }

  if (!keepOpen)
    this->close();

  return chanInd;
}


std::string FmfDeviceFunction::getActualDeviceName(bool fullPath) const
{
  std::string fileName;
  if (fileRef.getPointer())
    fileName = fileRef->fileName.getValue();
  else
    fileName = deviceName.getValue();

  if (!fullPath || fileName.empty())
    return fileName;

  std::string modelFilePath(FmDB::getMechanismObject()->getAbsModelFilePath());
  return FFaFilePath::makeItAbsolute(fileName,modelFilePath);
}


bool FmfDeviceFunction::getDevice(std::string& fileName,
                                  std::string& channelName) const
{
  fileName = deviceName.getValue();
  switch (FiDeviceFunctionFactory::identify(this->getActualDeviceName(true)))
    {
    case RPC_TH_FILE:
    case ASC_MC_FILE:
      channelName = channel.getValue().empty() ? "Not set" : channel.getValue();
      return true;
    default:
      channelName = "Not set";
      return false;
    }
}


bool FmfDeviceFunction::setDevice(const std::string& fileName,
                                  const std::string& channelName)
{
  bool changed = deviceName.setValue(fileName);
  if (channelName.empty() || channelName == "Not set")
    changed |= channel.setValue("");
  else
    changed |= channel.setValue(channelName);

  if (changed)
    this->close();

  return changed;
}


bool FmfDeviceFunction::setFileReference(FmFileReference* ref)
{
  if (ref == fileRef.getPointer())
    return false;

  fileRef.setPointer(ref);
  this->close();
  return true;
}


void FmfDeviceFunction::close()
{
  if (fileInd > 0)
    DFF->close(fileInd);

  fileInd = -1;
}


bool FmfDeviceFunction::getChannelList(Strings& channels) const
{
  if (fileInd > 0)
    return DFF->getChannelList(fileInd,channels);

  return FiDeviceFunctionFactory::getChannelList(this->getActualDeviceName(true),channels);
}
