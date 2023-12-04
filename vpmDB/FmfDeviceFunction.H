// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_DEVICE_FUNCTION_H
#define FMF_DEVICE_FUNCTION_H

#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmFileReference.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmfDeviceFunction : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfDeviceFunction(const char* fname = NULL,
                    const char* cname = NULL);

  virtual bool initGetValue();
  virtual double getValue(double x, int& ierr) const;
  virtual int getSmartPoints(double start, double stop,
			     std::vector<double>& x, std::vector<double>& y);

  virtual bool isLegalSprDmpFunc() const;
  virtual bool hasSmartPoints() const;

  virtual int printSolverEntry(FILE* fp);

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const;
  virtual const char* getFunctionFsiName() const { return "DEVICE_FUNCTION"; }
  virtual const char** getPixmap() const;

  int checkFileValidity(); // returns channel index, if valid multi-channel file

  std::string getActualDeviceName(bool fullPath = false) const;

  bool getDevice(std::string& fileName, std::string& channelName) const;
  bool setDevice(const std::string& fileName, const std::string& channelName = "Not set");

  FmFileReference* getFileReference() const { return fileReference.getPointer(); }
  bool setFileReference(FmFileReference* ref);

  bool getChannelList(std::vector<std::string>& channels) const;

  void close();

  bool isWaveFunction() const;
  double getHs() const { return myHs; }
  double getTz() const { return myTz; }

  FFaField<bool>   zeroAdjust;
  FFaField<double> verticalShift;
  FFaField<double> scaleFactor;
  FFaField<int>    randomSeed;
  FFaField<std::string> channel;
  FFaField<std::string> deviceName;

private:
  FFaReference<FmFileReference> fileReference;
  FFaField<FFaReferenceBase*>   fileReferenceField;

  int fileInd, chanInd; // used by getValue
  double myHs, myTz;    // user-defined wave function parameters

protected:
  virtual ~FmfDeviceFunction() {}

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
