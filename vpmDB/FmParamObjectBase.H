// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PARAM_OBJECT_BASE_H
#define FM_PARAM_OBJECT_BASE_H

#include "vpmDB/FmSimulationModelBase.H"

class FmParamObjectBase;


class FmParameter
{
public:
  FmParameter() { setFcn = 0; getFcn = 0; }
  FmParameter(const char* name,
	      void (*sf) (FmParamObjectBase*, double),
	      double (*gf) (FmParamObjectBase*))
  {
    fpName = name;
    setFcn = sf;
    getFcn = gf;
  }

  std::string fpName;
  void (*setFcn) (FmParamObjectBase*, double);
  double (*getFcn) (FmParamObjectBase*);
};


class FmParamObjectBase : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual const char** getPixmap() const;

  virtual void getParameters(std::vector<FmParameter>& retArray) const = 0;

protected:
  FmParamObjectBase();
  virtual ~FmParamObjectBase() {}

  static bool localParse(const char* keyWord, std::istream& activeStatement,
			 FmParamObjectBase* obj);
};


#define M_MOD_PARAMS(VAR_BASE,CL_TYPE) \
public: \
  double get##VAR_BASE() const { return my##VAR_BASE.getValue(); } \
  void set##VAR_BASE(double var) { my##VAR_BASE.setValue(var); } \
  static void s_set##VAR_BASE(FmParamObjectBase* obj, double var) \
  { ((CL_TYPE*)obj)->set##VAR_BASE(var); } \
  static double s_get##VAR_BASE(FmParamObjectBase* obj) \
  { return ((CL_TYPE*)obj)->get##VAR_BASE(); } \
protected: \
  FFaField<double> my##VAR_BASE

#define M_APPEND_PARAMS(NAME,VAR_BASE,CL_TYPE,ARRAY) \
  ARRAY.push_back(FmParameter(NAME,CL_TYPE::s_set##VAR_BASE,CL_TYPE::s_get##VAR_BASE))

#endif
