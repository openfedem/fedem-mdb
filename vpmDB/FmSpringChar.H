// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SPRING_CHAR_H
#define FM_SPRING_CHAR_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaContainers/FFaReference.H"
#include "FFaLib/FFaString/FFaEnum.H"


class FmSpringChar : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmSpringChar();

  enum SpringCharUse {
    NONE,             //! Default,
    GENERAL,          //! General, for use with engine
    TRANSLATION,      //! Spring with translational dof
    ROTATION          //! Spring rotational dof
  };

protected:
  FFaEnumMapping(SpringCharUse) {
    FFaEnumEntry(NONE,       "NONE");
    FFaEnumEntry(GENERAL,    "GENERAL");
    FFaEnumEntry(TRANSLATION,"TRANSLATION");
    FFaEnumEntry(ROTATION,   "ROTATION");
    FFaEnumEntryEnd;
  };

  FFaField<SpringCharUseEnum> myUse;

public:
  void setSpringCharUse(SpringCharUse use) { myUse.setValue(use); }
  SpringCharUse getSpringCharUse() const { return myUse.getValue(); }

  FFaField<double>             springStiffness;
  FFaField<FFaReferenceBase*>  springFunctionField;
  FFaReference<FmMathFuncBase> springFunction;

  FFaField<bool>               deflectionMaxIsOn;
  FFaField<double>             deflectionMax;

  FFaField<bool>               deflectionMinIsOn;
  FFaField<double>             deflectionMin;

  FFaField<bool>               forceMaxIsOn;
  FFaField<double>             forceMax;

  FFaField<bool>               forceMinIsOn;
  FFaField<double>             forceMin;

  FFaField<bool>               yieldForceMaxIsOn;
  FFaField<double>             yieldForceMax;
  FFaField<FFaReferenceBase*>  yieldForceMaxEngineField;
  FFaReference<FmEngine>       yieldForceMaxEngine;

  FFaField<bool>               yieldForceMinIsOn;
  FFaField<double>             yieldForceMin;
  FFaField<FFaReferenceBase*>  yieldForceMinEngineField;
  FFaReference<FmEngine>       yieldForceMinEngine;

  FFaField<bool>               yieldDeflectionMaxIsOn;
  FFaField<double>             yieldDeflectionMax;

  bool hasFailure() const;
  bool hasYield() const;

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fd);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Spring Characteristics"; }

protected:
  virtual ~FmSpringChar();

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
