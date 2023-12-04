// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaFunctionLib/FFaFunctionManager.H"

#include "vpmDB/FmfMultiVarBase.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfMULTI_VAR_BASE, FmfMultiVarBase, FmMathFuncBase);

FmfMultiVarBase::FmfMultiVarBase()
{
  Fmd_CONSTRUCTOR_INIT(FmfMultiVarBase);

  FFA_FIELD_DEFAULT_INIT(myValues, "VALUES");

  FFA_FIELD_INIT(extrapolationType, FLAT, "EXTRAPOLATION_TYPE");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmfMultiVarBase::removeVal(size_t place)
{
  DoubleVec& itsVals = myValues.getValue();
  if (place >= itsVals.size()) return false;

  itsVals.erase(itsVals.begin()+place);
  return true;
}


bool FmfMultiVarBase::insertVal(double val, size_t place)
{
  DoubleVec& itsVals = myValues.getValue();
  if (place > itsVals.size()) return false;

  itsVals.insert(itsVals.begin()+place,val);
  return true;
}


long int FmfMultiVarBase::binarySearch(double val, int blockSize) const
{
  const DoubleVec& itsVals = myValues.getValue();
  long int low = 0, high = itsVals.size();

  while (low < high)
    {
      long int mid = (low+high)/2;
      mid -= mid % blockSize;
      if (itsVals[mid] < val)
        low = mid + blockSize;
      else
        high = mid;
    }

  return low;
}


bool FmfMultiVarBase::initGetValue()
{
  myExplType = FFaFunctionManager::getTypeID(this->getFunctionFsiName());

  return !myValues.getValue().empty();
}


int FmfMultiVarBase::getExtrapolationType() const
{
  int flag = this->FmMathFuncBase::getExtrapolationType();
  return flag < 0 ? flag : (int)extrapolationType.getValue();
}


void FmfMultiVarBase::setExtrapolationType(int typeId)
{
  extrapolationType.setValue((FmfExtrapType)typeId);
}


void FmfMultiVarBase::getFirstValues(DoubleVec& toFill) const
{
  toFill.clear();

  const DoubleVec& itsVals = myValues.getValue();
  toFill.reserve(itsVals.size()/this->getBlockSize());
  for (size_t i = 0; i < itsVals.size(); i += this->getBlockSize())
    toFill.push_back(itsVals[i]);
}


void FmfMultiVarBase::getSecondValues(DoubleVec& toFill) const
{
  toFill.clear();
  if (this->getBlockSize() < 2) return;

  const DoubleVec& itsVals = myValues.getValue();
  toFill.reserve(itsVals.size()/this->getBlockSize());
  for (size_t i = 1; i < itsVals.size(); i += this->getBlockSize())
    toFill.push_back(itsVals[i]);
}


int FmfMultiVarBase::printSolverData(FILE* fp)
{
  const DoubleVec& itsVals = myValues.getValue();

  fprintf(fp,"  realDataSize = %u\n", (unsigned int)itsVals.size());
  fprintf(fp,"  realData =");
  for (size_t k = 0; k < itsVals.size(); k++)
    if (k % 6 == 0 && k > 0)
      fprintf(fp,"\n             %14.6e", itsVals[k]);
    else
      fprintf(fp," %14.6e", itsVals[k]);
  fprintf(fp,"\n");
  return 0;
}


bool FmfMultiVarBase::localParse(const char* keyWord,
                                 std::istream& activeStatement,
                                 FmfMultiVarBase* obj)
{
  bool retVal = true;
  if (!strcmp(keyWord,"VALS"))
  {
    // Old file format (R4.1.1 and earlier)
    retVal = parentParse("VALUES", activeStatement, obj);
    obj->removeVal(0); // Erase the first value (the array size)
  }
  else
    retVal = parentParse(keyWord, activeStatement, obj);

  return retVal;
}


bool FmfMultiVarBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfMultiVarBase::getClassTypeID());
}
