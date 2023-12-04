// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmfMultiArgBase.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfMULTI_ARG_BASE, FmfMultiArgBase, FmMathFuncBase);

FmfMultiArgBase::FmfMultiArgBase()
{
  Fmd_CONSTRUCTOR_INIT(FmfMultiArgBase);
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmfMultiArgBase::initGetValue()
{
  if (this->getNoArgs() > 1 &&
      this->getFunctionUse() != FmMathFuncBase::WAVE_FUNCTION)
  {
    // Multi-argument functions can only be evaluated in the UI if all
    // arguments are functions of the same variable (for instance Time)
    FmSensorBase* sensor = NULL;
    std::vector<FmEngine*> engines;
    this->getEngines(engines);
    for (size_t i = 0; i < engines.size() && !sensor; i++)
      if ((sensor = engines[i]->getUniqueSensor()))
        if (!engines[i]->initGetValue())
          return false;

    if (!sensor)
    {
      ListUI <<"ERROR: Cannot evaluate "<< this->getIdString() <<"\n";
      return false;
    }
  }

  return this->initGetValueNoRecursion();
}


double FmfMultiArgBase::getValue(double x, int& ierr) const
{
  unsigned int nArg = this->getNoArgs();
  if (nArg == 1)
    return this->getValueNoRecursion(x,ierr);
  else if (nArg == 4 && this->getFunctionUse() == FmMathFuncBase::WAVE_FUNCTION)
  {
    // Treat the x-argument as time and evaluate at the location {0,0,0}
    DoubleVec args(4,0.0); args.back() = x;
    return this->getValue(args,ierr);
  }

  double y = 0.0;
  std::vector<FmEngine*> engines;
  this->getEngines(engines);
  for (size_t i = 0; i < engines.size(); i++)
    if (engines[i]->getUniqueSensor())
      if (engines[i]->getValue(x,y))
        return y;

  std::cerr <<" *** Failed to evaluate "<< this->getIdString() <<"\n";
  return y;
}


bool FmfMultiArgBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfMultiArgBase::getClassTypeID());
}
