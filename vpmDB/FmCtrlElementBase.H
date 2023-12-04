// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CTRL_ELEMENT_BASE_H
#define FM_CTRL_ELEMENT_BASE_H

#include "vpmDB/FmIsRenderedBase.H"

class FmCtrlElementBase;
class FmCtrlLine;


//! \brief Struct for control element coordinates.
struct Vec2 { float x = 0.0f, y = 0.0f; };

inline bool operator==(const Vec2& P, const Vec2& Q)
{
  return P.x == Q.x && P.y == Q.y;
}

inline std::ostream& operator<<(std::ostream& os, const Vec2& P)
{
  return os << P.x <<' '<< P.y;
}

inline std::istream& operator>>(std::istream& is, Vec2& P)
{
  return is >> P.x >> P.y;
}


struct ctrlVars
{
  ctrlVars() : myString(NULL), setFcn(NULL), getFcn(NULL) {}

  ctrlVars(const char* str,
           void(*sf)(FmCtrlElementBase*,double),
           double(*gf)(FmCtrlElementBase*))
  {
    myString = str;
    setFcn = sf;
    getFcn = gf;
  }

  const char* myString;
  void (*setFcn) (FmCtrlElementBase*, double);
  double (*getFcn) (FmCtrlElementBase*);
};


class FmCtrlElementBase : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

public:
  void setPosition(float x, float y) { myPosition.setValue({x,y}); }
  const Vec2& getPosition() const { return myPosition.getValue(); }

  bool isLeftRotated() const { return myLeftRotatedFlag.getValue(); }
  void flipRotation() { myLeftRotatedFlag.setValue(!myLeftRotatedFlag.getValue()); }

  virtual FmCtrlElementBase* copy() = 0;

  virtual void getElementVariables(std::vector<ctrlVars>&) const {}
  const char** getPixmap() const { return itsPixmap; }

  virtual int getNumInputPorts() const { return 0; }
  virtual int getNumOutputPorts() const { return 0; }

  virtual FmCtrlLine* getLine(int) const { return NULL; }
  virtual bool setLine(int, FmCtrlLine*) { return false; }
  virtual void updateLines(bool single);

  virtual bool releaseFromPort(FmCtrlLine*) { return false; }
  virtual int atWhatPort(const FmCtrlLine*) const { return 0; }

protected:
  FmCtrlElementBase();
  virtual ~FmCtrlElementBase();

  FFaField<Vec2> myPosition;
  FFaField<bool> myLeftRotatedFlag;

  const char** itsPixmap;

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmCtrlElementBase* obj);

  virtual void changedEvent();
};


#define M_MOD_CTRL_VARS(VAR_BASE,CL_TYPE)				\
public:									\
double get##VAR_BASE() const { return my##VAR_BASE.getValue(); }	\
  void set##VAR_BASE(double var) { my##VAR_BASE.setValue(var); }	\
  static void s_set##VAR_BASE(FmCtrlElementBase* obj, double var)	\
  { ((CL_TYPE*)obj)->set##VAR_BASE(var); }				\
  static double s_get##VAR_BASE(FmCtrlElementBase* obj)			\
  { return ((CL_TYPE*)obj)->get##VAR_BASE(); }				\
private:								\
  FFaField<double> my##VAR_BASE

#define M_APPEND_CTRL_VARS(NAME,VAR_BASE,CL_TYPE,ARRAY)			\
  ARRAY.push_back(ctrlVars(NAME,CL_TYPE::s_set##VAR_BASE,CL_TYPE::s_get##VAR_BASE))

#endif
