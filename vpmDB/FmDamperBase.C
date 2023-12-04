// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmDamperBase.H"
#include <cmath>


Fmd_DB_SOURCE_INIT(FcDAMPER_BASE, FmDamperBase, FmIsControlledBase);


FmDamperBase::FmDamperBase()
{
  Fmd_CONSTRUCTOR_INIT(FmDamperBase);

  FFA_FIELD_INIT(isForceFunction, false, "IS_FORCE_FUNCTION");

  FFA_FIELD_INIT(myDampCoeff, 0.0, "DAMPING_COEFF");
  FFA_FIELD_INIT(isDefDamper, false, "IS_DEF_DAMPER");

  FFA_REFERENCE_FIELD_INIT(damperFunctionField, damperFunction, "DAMPER_FUNCTION");
  damperFunction.setPrintIfZero(false);
}


bool FmDamperBase::isForceFunc() const
{
  if (damperFunction.isNull())
    return false;

  FmMathFuncBase::FuncUse fu = damperFunction->getFunctionUse();

  return (fu == FmMathFuncBase::DA_TRA_FORCE ||
	  fu == FmMathFuncBase::DA_ROT_TORQUE);
}


std::pair<bool,bool> FmDamperBase::isForceFuncFromFile() const
{
  if (isForceFunction.wasOnFile())
    return std::make_pair(true,isForceFunction.getValue());
  else
    return std::make_pair(false,false);
}


bool FmDamperBase::localParse(const char* keyWord, std::istream& activeStatement,
			      FmDamperBase* obj)
{
  // Conversion of old keywords
  if (strcmp(keyWord,"INIT_DAMPING") == 0)
    return parentParse("DAMPING_COEFF", activeStatement, obj);

  return parentParse(keyWord, activeStatement, obj);
}


bool FmDamperBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmDamperBase::getClassTypeID());
}


int FmDamperBase::printSolverEntry(FILE* fp)
{
  double d0 = myDampCoeff.getValue();
  FmMathFuncBase* dampFunc = this->getFunction();
  if (dampFunc)
    fprintf(fp,"  d1 = 1.0, %sFuncId = %d\n",
            this->isForceFunc() ? "force" : "coeff",
            dampFunc->getBaseID());
  else if (fabs(d0) > 1.0e-15)
    fprintf(fp,"  d0 =%17.9e\n", d0);

  FmEngine* scaleFunc = this->getEngine();
  if (scaleFunc) // Engine scaling of the damping coefficient
    fprintf(fp,"  coeffScaleEngineId = %d\n", scaleFunc->getBaseID());

  if (isDefDamper.getValue())
    fprintf(fp,"  isDefDamper = .true.\n");

  // Variables to be saved:
  // 1 - Damper coefficient
  // 2 - Length
  // 3 - Velocity
  // 4 - Force
  // 5 - Energies
  this->writeSaveVar(fp,5);

  fprintf(fp,"/\n\n");
  return 0;
}
