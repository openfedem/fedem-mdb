// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSpringBase.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include <cmath>


Fmd_DB_SOURCE_INIT(FcSPRING_BASE, FmSpringBase, FmIsControlledBase);


FmSpringBase::FmSpringBase()
{
  Fmd_CONSTRUCTOR_INIT(FmSpringBase);

  FFA_FIELD_INIT(isForceFunction, false, "IS_FORCE_FUNCTION");

  FFA_FIELD_INIT(myStiffness, 0.0, "STIFFNESS");
  FFA_FIELD_INIT(myInitLength, 0.0, "INIT_LENGTH");
  FFA_FIELD_INIT(useInitDeflection, true, "USE_INIT_DEFLECTION");

  FFA_REFERENCE_FIELD_INIT(myStiffFunctionField, myStiffFunction, "STIFF_FUNCTION");
  FFA_REFERENCE_FIELD_INIT(mySpringCharField, mySpringChar, "SPRING_CHAR");
  FFA_REFERENCE_FIELD_INIT(scaleEngineField, scaleEngine, "SCALE_ENGINE");
  myStiffFunction.setPrintIfZero(false);
  mySpringChar.setPrintIfZero(false);
  scaleEngine.setPrintIfZero(false);
}


bool FmSpringBase::isForceFunc() const
{
  FmMathFuncBase* sfunc = this->getStiffFunction();
  if (sfunc)
    switch (sfunc->getFunctionUse()) {
    case FmMathFuncBase::SPR_TRA_FORCE:
    case FmMathFuncBase::SPR_ROT_TORQUE:
      return true;
    default:
      break;
    }

  return false;
}


/*!
  \note Used by FmMathFuncBase only, to resolve functions from <= r3.2
  \return A pair, where \a first says if the variable was read,
  and \a second is the variable that was read from file.
  \a second is always false if not present on file.
*/

std::pair<bool,bool> FmSpringBase::isForceFuncFromFile() const
{
  if (isForceFunction.wasOnFile())
    return std::make_pair(true,isForceFunction.getValue());
  else
    return std::make_pair(false,false);
}


FmMathFuncBase* FmSpringBase::getStiffFunction() const
{
  if (mySpringChar.isNull())
    return myStiffFunction.getPointer();
  else
    return mySpringChar->springFunction.getPointer();
}


FmModelMemberBase* FmSpringBase::getSpringCharOrStiffFunction() const
{
  if (mySpringChar.isNull())
    return myStiffFunction.getPointer();
  else
    return mySpringChar.getPointer();
}


void FmSpringBase::setSpringCharOrStiffFunction(FmModelMemberBase* item)
{
  myStiffFunction.setRef(0);
  mySpringChar.setRef(0);

  if (!item)
    return;

  else if (item->isOfType(FmMathFuncBase::getClassTypeID()))
    myStiffFunction.setRef(static_cast<FmMathFuncBase*>(item));

  else if (item->isOfType(FmSpringChar::getClassTypeID()))
    mySpringChar.setRef(static_cast<FmSpringChar*>(item));
}


double FmSpringBase::getInitLength() const
{
  if (useInitDeflection.getValue())
    return this->getModelSpringLength() - myInitLength.getValue();
  else
    return myInitLength.getValue();
}

double FmSpringBase::getInitDeflection() const
{
  if (useInitDeflection.getValue())
    return myInitLength.getValue();
  else
    return this->getModelSpringLength() - myInitLength.getValue();
}

bool FmSpringBase::getInitLengthOrDefl(double& lenOrDefl) const
{
  lenOrDefl = myInitLength.getValue();
  return useInitDeflection.getValue();
}


double FmSpringBase::getInitStiff() const
{
  if (mySpringChar.isNull())
    return myStiffness.getValue();
  else
    return mySpringChar->springStiffness.getValue();
}


void FmSpringBase::setInitLengthOrDefl(double lenOrDefl, bool isDeflection)
{
  myInitLength.setValue(lenOrDefl);
  useInitDeflection.setValue(isDeflection);
}


bool FmSpringBase::localParse(const char* keyWord, std::istream& activeStatement,
			      FmSpringBase* obj)
{
  // Conversion of old keywords
  if (strcmp(keyWord,"INIT_STIFFNESS") == 0)
    return parentParse("STIFFNESS", activeStatement, obj);

  return parentParse(keyWord, activeStatement, obj);
}


bool FmSpringBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmSpringBase::getClassTypeID());
}


int FmSpringBase::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&SPRING_BASE\n");
  this->printID(fp);

  // Stress-free length engine
  int lenEngineId = this->getEngine() ? this->getEngine()->getBaseID() : 0;

  fprintf(fp,"  l0 =%17.9e, l1 = %3.1f, lengthEngineId = %d\n",
          this->getInitLength(), lenEngineId > 0 ? 1.0 : 0.0, lenEngineId);

  // Beta feature: Cyclic soil spring with linear unloading
  FFaString sDesc = this->getUserDescription();
  int cyclicSpring = 0;
  if (sDesc.hasSubString("#Cyclic"))
  {
    cyclicSpring = sDesc.getIntAfter("#Cyclic");
    if (cyclicSpring < 1) cyclicSpring = 1;
  }

  // Stiffness function part
  double s0 = 0.0;
  int springFuncId = 0;
  FmMathFuncBase* sfunc = this->getStiffFunction();
  if (sfunc)
  {
    springFuncId = sfunc->getBaseID();
    // Stiffness function is used, rule out constant stiffness
    // Unless an initial secant stiffness is provided
    if (cyclicSpring == 3)
      s0 = this->getInitStiff();
  }
  else
    s0 = this->getInitStiff();

  if (fabs(s0) > 1.0e-15)
    fprintf(fp, (springFuncId > 0 ? "  s0 =%17.9e," : "  s0 =%17.9e\n"), s0);
  else if (springFuncId > 0)
    fprintf(fp, " ");
  if (springFuncId > 0)
    fprintf(fp, " s1 = 1.0, %sFuncId = %d\n",
            this->isForceFunc() ? "force" : "stiff", springFuncId);

  // Beta feature: Possible engine-scaling of the stiffness function
  int scalePosId = sDesc.getIntAfter("#PosStiffScaleEngine");
  if (scalePosId > 0) FmEngine::betaFeatureEngines.insert(scalePosId);
  int scaleNegId = sDesc.getIntAfter("#NegStiffScaleEngine");
  if (scaleNegId > 0) FmEngine::betaFeatureEngines.insert(scaleNegId);

  if (!scaleEngine.isNull())
  {
    if (scalePosId <= 0) scalePosId = scaleEngine->getBaseID();
    if (scaleNegId <= 0) scaleNegId = scaleEngine->getBaseID();
  }

  if (scalePosId > 0 || scaleNegId > 0)
  {
    fprintf(fp,"  stiffScaleEnginePosId = %d\n", scalePosId);
    fprintf(fp,"  stiffScaleEngineNegId = %d\n", scaleNegId);
  }

  FmSpringChar* sprChar = mySpringChar.getPointer();
  if (sprChar)
  {
    if (sprChar->hasFailure())
      fprintf(fp,"  springFailureId = %d\n", sprChar->getBaseID());
    if (sprChar->hasYield())
      fprintf(fp,"  springYieldId = %d\n", sprChar->getBaseID());
  }

  if (cyclicSpring > 0)
    fprintf(fp,"  unLoadType = %d\n", cyclicSpring);

  // Variables to be saved:
  // 1 - Spring stiffness
  // 2 - Length
  // 3 - Deflection
  // 4 - Force
  // 5 - Energies
  this->writeSaveVar(fp,5);

  fprintf(fp,"/\n\n");
  return 0;
}
