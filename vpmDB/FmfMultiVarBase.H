// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMF_MULTI_VAR_BASE_H
#define FMF_MULTI_VAR_BASE_H

#include "FFaLib/FFaString/FFaEnum.H"

#include "vpmDB/FmMathFuncBase.H"


class FmfMultiVarBase : public FmMathFuncBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual bool initGetValue();

  virtual void addXYset(double, double) {}
  virtual bool removeXYset(long int) { return false; }

  void getFirstValues(DoubleVec& toFill) const;
  void getSecondValues(DoubleVec& toFill) const;

  virtual int getExtrapolationType() const;
  void setExtrapolationType(int typeId);

  virtual int getBlockSize() const = 0;

  virtual const DoubleVec& getData() const { return myValues.getValue(); }
  virtual DoubleVec& getData() { return myValues.getValue(); }

protected:
  FmfMultiVarBase();
  virtual ~FmfMultiVarBase() {}

  size_t numValues() const { return myValues.getValue().size(); }

  bool removeVal(size_t place);
  bool insertVal(double val, size_t place);

  long int binarySearch(double val, int blockSize) const;

  virtual int printSolverData(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmfMultiVarBase* obj);

  enum FmfExtrapType {
    NONE   = 0,
    FLAT   = 1,
    LINEAR = 2
  };

private:
  FFaEnumMapping(FmfExtrapType) {
    FFaEnumEntry(NONE,   "None");
    FFaEnumEntry(FLAT,   "Flat");
    FFaEnumEntry(LINEAR, "Linear");
    FFaEnumEntryEnd;
  };

protected:
  FFaField<FmfExtrapTypeEnum> extrapolationType;
  FFaField<DoubleVec>         myValues;
};

#endif
