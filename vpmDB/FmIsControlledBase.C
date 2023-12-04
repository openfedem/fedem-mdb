// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmIsControlledBase.H"


Fmd_DB_SOURCE_INIT(FcIS_CONTROLLED_BASE, FmIsControlledBase, FmIsPlottedBase);


FmIsControlledBase::FmIsControlledBase()
{
  Fmd_CONSTRUCTOR_INIT(FmIsControlledBase);

  FFA_REFERENCE_FIELD_INIT(myEngineField, myEngine, "ENGINE");
  myEngine.setPrintIfZero(false);
  //kmo: Not necessary, only for comparison with older model files
  //myEngine.setForcePrintType(true);
}


bool FmIsControlledBase::localParse(const char* keyWord, std::istream& activeStatement,
				    FmIsControlledBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmIsControlledBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmIsControlledBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmIsControlledBase* copyObj = static_cast<FmIsControlledBase*>(obj);
  this->setEngine(copyObj->getEngine());
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->setEngine(NULL);

  return true;
}
