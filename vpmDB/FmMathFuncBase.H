// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MATH_FUNC_BASE_H
#define FM_MATH_FUNC_BASE_H

#include "vpmDB/FmParamObjectBase.H"
#include "FFaLib/FFaString/FFaEnum.H"

class FmEngine;
class FmCurveSet;
class FaVec3;

typedef FmParameter FmFuncVariable;


class FmMathFuncBase : public FmParamObjectBase
{
  Fmd_DB_HEADER_INIT();

public:
  //! Enum that indicates what the function is used for
  enum FuncUse {
    NONE,           //! Default (unused function)
    GENERAL,        //! General, for use with engine
    DRIVE_FILE,     //! Drive file
    ROAD_FUNCTION,  //! Road function
    WAVE_FUNCTION,  //! Wave function
    CURR_FUNCTION,  //! Current function
    SPR_TRA_STIFF,  //! Spring, stiffness - translational deflection
    SPR_TRA_FORCE,  //! Spring, force - translational deflection
    SPR_ROT_STIFF,  //! Spring, stiffness - angular deflection
    SPR_ROT_TORQUE, //! Spring, torque - angular deflection
    DA_TRA_COEFF,   //! Damper, damping coefficient - translational velocity
    DA_TRA_FORCE,   //! Damper, force - translational velocity
    DA_ROT_COEFF,   //! Damper, damping coefficient - angular velocity
    DA_ROT_TORQUE   //! Damper, torque - angular velocity
  };

private:
  FFaEnumMapping(FuncUse) {
    FFaEnumEntry( NONE, "NONE");
    FFaEnumEntry( GENERAL, "GENERAL");
    FFaEnumEntry( DRIVE_FILE, "DRIVE_FILE");
    FFaEnumEntry( ROAD_FUNCTION, "ROAD_FUNCTION");
    FFaEnumEntry( WAVE_FUNCTION, "WAVE_FUNCTION");
    FFaEnumEntry( CURR_FUNCTION, "CURR_FUNCTION");
    FFaEnumEntry( SPR_TRA_STIFF, "SPR_TRA_STIFF");
    FFaEnumEntry( SPR_TRA_FORCE, "SPR_TRA_FORCE");
    FFaEnumEntry( SPR_ROT_STIFF, "SPR_ROT_STIFF");
    FFaEnumEntry( SPR_ROT_TORQUE, "SPR_ROT_TORQUE");
    FFaEnumEntry( DA_TRA_COEFF, "DA_TRA_COEFF");
    FFaEnumEntry( DA_TRA_FORCE, "DA_TRA_FORCE");
    FFaEnumEntry( DA_ROT_COEFF, "DA_ROT_COEFF");
    FFaEnumEntry( DA_ROT_TORQUE, "DA_ROT_TORQUE");
    FFaEnumEntryEnd;
  };

public:
  void getEngines(std::vector<FmEngine*>& toFill) const;

  FmMathFuncBase* copy() const;

  virtual bool isListable() const;
  virtual bool isLegalSprDmpFunc() const;
  virtual bool hasSmartPoints() const;
  virtual bool isSurfaceFunc() const { return false; }

  virtual int printSolverEntry(FILE* fp);

  virtual const char* getFunctionFsiName() const;
  virtual const char* getFunctionUIName() const { return "Math function"; }
  virtual const char* getUITypeName() const;
  virtual std::string getInfoString() const;

  virtual bool initGetValue();
  virtual double getValue(double x, int& ierr) const;
  virtual double getValue(const DoubleVec& x, int& ierr) const
  { return this->getValue(x.front(),ierr); }
  virtual double getValue(double g, double d, const FaVec3& X, double t) const;

  virtual unsigned int getNoArgs() const { return 1; }
  virtual void setNoArgs(unsigned int) {}

  virtual int getSmartPoints(double start, double stop,
			     DoubleVec& x, DoubleVec& y);
  int getCurvePoints(double start, double stop, double inc,
		     DoubleVec& x, DoubleVec& y);
  virtual void getXAxisDomain(double&, double&) const {}

  virtual void getFunctionVariables(std::vector<FmFuncVariable>&, bool) const {}

  virtual void getParameters(std::vector<FmParameter>& toFill) const
  { this->getFunctionVariables(toFill,false); }

  virtual int getExtrapolationType() const;

  virtual const DoubleVec& getData() const { return myExplData; }
  virtual DoubleVec& getData() { return myExplData; }

  FmCurveSet* getPreviewCurve() const;

  bool    setFunctionUsage(int usage);
  bool    setFunctionUse(FuncUse newUse, bool checkUniqueID = false);
  FuncUse getFunctionUse() const { return myUse.getValue(); }

  virtual void onEventSwitched(const FmSimulationEvent*);

  static void resolveAfterRead();

protected:
  FmMathFuncBase();
  virtual ~FmMathFuncBase();

  virtual int printSolverData(FILE*) { return 1; }

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmMathFuncBase* obj);

  int       myExplType; // used by getValue
  DoubleVec myExplData; // used by getValue

private:
  FFaField<FuncUseEnum> myUse;
};

#endif
