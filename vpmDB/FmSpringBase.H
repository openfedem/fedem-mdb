// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * CLASS: FmSpringBase
 * AUTHOR: Jens LIEN
 * LAST MODIFIED: 14 jun 1995
 *
 * DESCRIPTION:
 * Parent class for the database entries.
 * Has an additional element for the Spring ring.
 ******************************************************************************/

#ifndef FM_SPRING_BASE_H
#define FM_SPRING_BASE_H

#include "vpmDB/FmIsControlledBase.H"
#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmSpringChar.H"


class FmSpringBase : public FmIsControlledBase
{
  Fmd_DB_HEADER_INIT();

protected:
  FmSpringBase();
  virtual ~FmSpringBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmSpringBase* obj);

public:
  // Initial length or deflection of spring
  double getInitLength() const;
  double getInitDeflection() const;
  bool   getInitLengthOrDefl(double& lenOrDefl) const;
  void   setInitLengthOrDefl(double lenOrDefl, bool isDeflection = false);

  // The engine that controls the stress-free length change in the spring
  FmEngine* getLengthEngine() const { return this->getEngine(); }
  void setLengthEngine(FmEngine* ctrl) { this->setEngine(ctrl); }

  // This is the constant spring stiffness,
  // unless the stiffness or force in the spring is given using a function
  double getInitStiff() const;
  bool   setInitStiff(double st) { return myStiffness.setValue(st); }

  // The function that controls the spring stiffness or force
  FmMathFuncBase*    getStiffFunction() const;
  FmModelMemberBase* getSpringCharOrStiffFunction() const;
  void setSpringCharOrStiffFunction(FmModelMemberBase* item);

  // The scale engine that scales the spring stiffness/force.
  FmEngine* getScaleEngine() const { return scaleEngine.getPointer(); }
  void setScaleEngine(FmEngine* func) { scaleEngine.setRef(func); }

  bool isForceFunc() const;

  // This is accessed by FmMathFuncBase only
  std::pair<bool,bool> isForceFuncFromFile() const;

  virtual int    getDOF() const { return 1; }
  virtual double getModelSpringLength() const = 0;

  virtual int printSolverEntry(FILE* fp);

protected:
  FFaField<double> myStiffness;
  FFaField<double> myInitLength;
  FFaField<bool>   useInitDeflection;

  FFaField<FFaReferenceBase*> myStiffFunctionField;
  FFaField<FFaReferenceBase*> mySpringCharField;
  FFaField<FFaReferenceBase*> scaleEngineField;

  FFaReference<FmMathFuncBase> myStiffFunction;
  FFaReference<FmSpringChar>   mySpringChar;
  FFaReference<FmEngine>       scaleEngine;

private:
  FFaObsoleteField<bool> isForceFunction;
};

#endif
