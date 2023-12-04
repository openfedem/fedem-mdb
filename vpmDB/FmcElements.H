// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMC_ELEMENTS_H
#define FMC_ELEMENTS_H

#include "vpmDB/FmCtrlInputElementBase.H"


class Fmc1ordTF : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  Fmc1ordTF();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "1st Order transfer function"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~Fmc1ordTF() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,Fmc1ordTF);
  M_MOD_CTRL_VARS(T1,Fmc1ordTF);
  M_MOD_CTRL_VARS(T2,Fmc1ordTF);
};


class Fmc2ordTF : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  Fmc2ordTF();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "2nd Order transfer function"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~Fmc2ordTF() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,Fmc2ordTF);
  M_MOD_CTRL_VARS(T1,Fmc2ordTF);
  M_MOD_CTRL_VARS(T2,Fmc2ordTF);
  M_MOD_CTRL_VARS(T3,Fmc2ordTF);
  M_MOD_CTRL_VARS(T4,Fmc2ordTF);
};


class FmcAdder : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcAdder();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Adder"; }

protected:
  virtual ~FmcAdder() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};


class FmcAmplifier : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcAmplifier();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Amplifier"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcAmplifier() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Rate,FmcAmplifier);
};


class FmcCompConjPole : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcCompConjPole();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Complex conjugate poles"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcCompConjPole() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcCompConjPole);
  M_MOD_CTRL_VARS(ResFreq,FmcCompConjPole);
  M_MOD_CTRL_VARS(DampFac,FmcCompConjPole);
};


class FmcComparator : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcComparator();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Comparator"; }

protected:
  virtual ~FmcComparator() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};


class FmcDeadZone : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcDeadZone();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Dead Zone"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcDeadZone() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Left,FmcDeadZone);
  M_MOD_CTRL_VARS(Right,FmcDeadZone);
};


class FmcHysteresis : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcHysteresis();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Hysteresis"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcHysteresis() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Left,FmcHysteresis);
  M_MOD_CTRL_VARS(Right,FmcHysteresis);
  M_MOD_CTRL_VARS(Alpha,FmcHysteresis);
};


class FmcIntegrator : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcIntegrator();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Integrator"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcIntegrator() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(InitialValue,FmcIntegrator);
};


class FmcLimDerivator : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcLimDerivator();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Limited derivator"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcLimDerivator() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Tfd,FmcLimDerivator);
};


class FmcLimitation : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcLimitation();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Limitation"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcLimitation() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Lower,FmcLimitation);
  M_MOD_CTRL_VARS(Upper,FmcLimitation);
};


class FmcLogicalSwitch : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcLogicalSwitch();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Logical Switch"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcLogicalSwitch() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Lower,FmcLogicalSwitch);
  M_MOD_CTRL_VARS(Upper,FmcLogicalSwitch);
  M_MOD_CTRL_VARS(YOn,FmcLogicalSwitch);
};


class FmcMultiplier : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcMultiplier();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Multiplier"; }

protected:
  virtual ~FmcMultiplier() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};


class FmcPIlimD : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPIlimD();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "PI+lim D"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPIlimD() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPIlimD);
  M_MOD_CTRL_VARS(Ti,FmcPIlimD);
  M_MOD_CTRL_VARS(Td,FmcPIlimD);
  M_MOD_CTRL_VARS(Tfd,FmcPIlimD);
};


class FmcPd : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPd();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "PD"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPd() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPd);
  M_MOD_CTRL_VARS(Td,FmcPd);
};


class FmcPi : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPi();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "PI"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPi() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPi);
  M_MOD_CTRL_VARS(Ti,FmcPi);
};


class FmcPid : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPid();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "PID"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPid() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPid);
  M_MOD_CTRL_VARS(Ti,FmcPid);
  M_MOD_CTRL_VARS(Td,FmcPid);
};


class FmcPlimD : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPlimD();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "P+lim D"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPlimD() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPlimD);
  M_MOD_CTRL_VARS(Td,FmcPlimD);
  M_MOD_CTRL_VARS(Tfd,FmcPlimD);
};


class FmcPlimI : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPlimI();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "P+lim I"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPlimI() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPlimI);
  M_MOD_CTRL_VARS(Ti,FmcPlimI);
  M_MOD_CTRL_VARS(Tfi,FmcPlimI);
};


class FmcPlimIlimD : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPlimIlimD();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "P+lim I+lim D"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPlimIlimD() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcPlimIlimD);
  M_MOD_CTRL_VARS(Ti,FmcPlimIlimD);
  M_MOD_CTRL_VARS(Td,FmcPlimIlimD);
  M_MOD_CTRL_VARS(Tfi,FmcPlimIlimD);
  M_MOD_CTRL_VARS(Tfd,FmcPlimIlimD);
};


class FmcPower : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcPower();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Power element"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcPower() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Exponent,FmcPower);
};


class FmcRealPole : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcRealPole();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Real pole"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcRealPole() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Kp,FmcRealPole);
  M_MOD_CTRL_VARS(T1,FmcRealPole);
};


class FmcSampleHold : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcSampleHold();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Sample and hold"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcSampleHold() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Period,FmcSampleHold);
};


class FmcTimeDelay : public FmCtrlInputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcTimeDelay();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Time delay"; }

  virtual void getElementVariables(std::vector<ctrlVars>& vars) const;

protected:
  virtual ~FmcTimeDelay() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_CTRL_VARS(Delay,FmcTimeDelay);
};

#endif
