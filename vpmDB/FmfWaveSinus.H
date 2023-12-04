// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_WAVE_SINUS_H
#define FMF_WAVE_SINUS_H

#include "vpmDB/FmMathFuncBase.H"

typedef std::pair<double,double> FmRange;


class FmfWaveSinus : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmfWaveSinus(int dof = 0);

  virtual bool initGetValue();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getFunctionUIName() const { return "Sine wave"; }
  virtual const char* getFunctionFsiName() const;

protected:
  virtual ~FmfWaveSinus() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual int printSolverData(FILE* fp);

public:
  FFaNoPrintField<int> lDof;
};


class FmfWaveSpectrum : public FmfWaveSinus
{
  Fmd_DB_HEADER_INIT();

public:
  FmfWaveSpectrum();

  virtual bool isSurfaceFunc() const { return nDir.getValue() > 1; }

  virtual bool initGetValue();
  virtual double getValue(double x, int& ierr) const;
  virtual double getValue(double g, double d, const FaVec3& X, double t) const;

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual void initAfterParse();

  const FmRange& deriveTrange();
  double deriveSpectralPeakedness();

  virtual const char* getFunctionUIName() const;
  virtual const char* getFunctionFsiName() const;
  virtual const char** getPixmap() const;

protected:
  virtual ~FmfWaveSpectrum() {}

  virtual void changedEvent();

  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual int printSolverData(FILE* fp);

public:
  enum FmSpectrum {
    PiersonMoskowitz = 0,
    JONSWAP = 1,
  };

private:
  std::vector<int> myIntData; // used by getValue
  mutable std::vector<double> mainDirData; // used by getValue

  FFaEnumMapping(FmSpectrum) {
    FFaEnumEntry(PiersonMoskowitz,"PiersonMoskowitz");
    FFaEnumEntry(JONSWAP,"JONSWAP");
    FFaEnumEntryEnd;
  };

public:
  FFaField<FmSpectrumEnum> spectrum;
  FFaField<int>            nComp;
  FFaField<int>            nDir;
  FFaField<int>            sprExp;
  FFaField<bool>           rndPhase;
  FFaField<int>            myRandomSeed;
  FFaField<double>         myHs;
  FFaField<double>         myTp;
  FFaField<double>         myPeakedness;
  FFaField<bool>           autoCalcPeakedness;
  FFaField<FmRange>        myTrange;
  FFaField<bool>           autoCalcTrange;
};

#endif
