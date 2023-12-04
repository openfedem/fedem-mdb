// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * CLASS: FmDamperBase
 * AUTHOR: Jens LIEN
 * LAST MODIFIED: 14 jun 1995
 *
 * DESCRIPTION:
 * Parent class for the database entries.
 * Has an additional element for the Damper ring.
 ******************************************************************************/

#ifndef FM_DAMPER_BASE_H
#define FM_DAMPER_BASE_H

#include "vpmDB/FmIsControlledBase.H"
#include "vpmDB/FmMathFuncBase.H"


class FmDamperBase : public FmIsControlledBase
{
  Fmd_DB_HEADER_INIT();

protected:
  FmDamperBase();
  virtual ~FmDamperBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmDamperBase* obj);

public:
  // The constant damping coefficient, used only if damperFunction is none
  double getInitDamp() const { return myDampCoeff.getValue(); }
  bool   setInitDamp(double d) { return myDampCoeff.setValue(d); }

  // The damper function, used only in fedem >= v3.0
  FmMathFuncBase* getFunction() const { return damperFunction.getPointer(); }
  void setFunction(FmMathFuncBase* func) { damperFunction.setRef(func); }

  // The scale engine that scales the damper force/coefficient.
  // In fedem < v3.0 : The damper coefficient function directly
  // In fedem >= v3.0: A scale engine
  FmEngine* getDampEngine() const { return this->getEngine(); }
  void setDampEngine(FmEngine* func) { this->setEngine(func); }

  bool isForceFunc() const;

  // This is accessed by FmMathFuncBase only
  std::pair<bool,bool> isForceFuncFromFile() const;

  virtual int getDOF() const { return 1; }

  virtual int printSolverEntry(FILE* fp);

private:
  FFaField<double> myDampCoeff;

  FFaField<FFaReferenceBase*>  damperFunctionField;
  FFaReference<FmMathFuncBase> damperFunction;

public:
  // Indicator for whether the damper velocity is associated with
  // the deformational velocity of a matching spring
  FFaField<bool> isDefDamper;

private:
  FFaObsoleteField<bool> isForceFunction;
};

#endif
