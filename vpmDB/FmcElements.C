// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmcElements.H"
#include "FFaLib/FFaString/FFaParse.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdCtrlElement.H"
#endif
#include "vpmDB/Icons/FmIconPixmaps.H"

Fmd_DB_SOURCE_INIT(FccFIRST_ORDTF, Fmc1ordTF, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccSEC_ORDTF, Fmc2ordTF, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccADDER, FmcAdder, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccAMPLIFIER, FmcAmplifier, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccCOMPCONJPOLE, FmcCompConjPole, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccCOMPARATOR, FmcComparator, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccDEAD_ZONE, FmcDeadZone, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccHYSTERESIS, FmcHysteresis, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccINTEGRATOR, FmcIntegrator, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccLIM_DERIVATOR, FmcLimDerivator, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccLIMITATION, FmcLimitation, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccLOGICAL_SWITCH, FmcLogicalSwitch, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccMULTIPLIER, FmcMultiplier, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPILIMD, FmcPIlimD, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPD, FmcPd, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPI, FmcPi, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPID, FmcPid, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPLIMD, FmcPlimD, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPLIMI, FmcPlimI, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPLIMILIMD, FmcPlimIlimD, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccPOWER, FmcPower, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccREAL_POLE, FmcRealPole, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccSAMPLE_HOLD, FmcSampleHold, FmCtrlInputElementBase);
Fmd_DB_SOURCE_INIT(FccTIME_DELAY, FmcTimeDelay, FmCtrlInputElementBase);

/******************************************************************************/

Fmc1ordTF::Fmc1ordTF()
{
  Fmd_CONSTRUCTOR_INIT(Fmc1ordTF);
  this->setPortCount(1);
  this->setStateVarCount(2);

  FFA_FIELD_INIT(myKp, 1.0, "KP");
  FFA_FIELD_INIT(myT1, 1.0, "T1");
  FFA_FIELD_INIT(myT2, 1.0, "T2");

  itsPixmap = ctrl1ordTFSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* Fmc1ordTF::copy()
{
  Fmc1ordTF* newObj = new Fmc1ordTF();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void Fmc1ordTF::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,Fmc1ordTF,retArray);
  M_APPEND_CTRL_VARS("T1",T1,Fmc1ordTF,retArray);
  M_APPEND_CTRL_VARS("T2",T2,Fmc1ordTF,retArray);
}


std::ostream& Fmc1ordTF::writeFMF(std::ostream& os)
{
  os <<"CONTROL_FIRST_ORDTF\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int Fmc1ordTF::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! 1st order Tranfer Function\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 43\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myT1.getValue(), myT2.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool Fmc1ordTF::readAndConnect(std::istream& is, std::ostream&)
{
  Fmc1ordTF* obj = new Fmc1ordTF();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool Fmc1ordTF::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool Fmc1ordTF::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(Fmc1ordTF::getClassTypeID());
}

/******************************************************************************/

Fmc2ordTF::Fmc2ordTF()
{
  Fmd_CONSTRUCTOR_INIT(Fmc2ordTF);
  this->setPortCount(1);
  this->setStateVarCount(4);

  FFA_FIELD_INIT(myKp, 1.0, "KP");
  FFA_FIELD_INIT(myT1, 1.0, "T1");
  FFA_FIELD_INIT(myT2, 1.0, "T2");
  FFA_FIELD_INIT(myT3, 1.0, "T3");
  FFA_FIELD_INIT(myT4, 1.0, "T4");

  itsPixmap = ctrl2ordTFSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* Fmc2ordTF::copy()
{
  Fmc2ordTF* newObj = new Fmc2ordTF();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void Fmc2ordTF::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,Fmc2ordTF,retArray);
  M_APPEND_CTRL_VARS("T1",T1,Fmc2ordTF,retArray);
  M_APPEND_CTRL_VARS("T2",T2,Fmc2ordTF,retArray);
  M_APPEND_CTRL_VARS("T3",T3,Fmc2ordTF,retArray);
  M_APPEND_CTRL_VARS("T4",T4,Fmc2ordTF,retArray);
}


std::ostream& Fmc2ordTF::writeFMF(std::ostream &os)
{
  os <<"CONTROL_SEC_ORDTF\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int Fmc2ordTF::printSolverEntry(FILE *fp)
{
  fprintf(fp,"! 2nd order Tranfer Function\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 44\n");
  fprintf(fp,"  nRealData = 5\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myT1.getValue(), myT2.getValue(),
	  myT3.getValue(), myT4.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool Fmc2ordTF::readAndConnect(std::istream& is, std::ostream&)
{
  Fmc2ordTF* obj = new Fmc2ordTF();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool Fmc2ordTF::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool Fmc2ordTF::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(Fmc2ordTF::getClassTypeID());
}

/******************************************************************************/

FmcAdder::FmcAdder()
{
  Fmd_CONSTRUCTOR_INIT(FmcAdder);
  this->setPortCount(2);
  this->setStateVarCount(0);

  itsPixmap = ctrlAdderSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcAdder::copy()
{
  FmcAdder* newObj = new FmcAdder();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


std::ostream& FmcAdder::writeFMF(std::ostream &os)
{
  os <<"CONTROL_ADDER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcAdder::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Adder\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 2\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", 1.0);
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcAdder::readAndConnect(std::istream& is, std::ostream&)
{
  FmcAdder* obj = new FmcAdder();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcAdder::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcAdder::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcAdder::getClassTypeID());
}

/******************************************************************************/

FmcAmplifier::FmcAmplifier()
{
  Fmd_CONSTRUCTOR_INIT(FmcAmplifier);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myRate, 1.0, "RATE");

  itsPixmap = ctrlAmplifierSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcAmplifier::copy()
{
  FmcAmplifier* newObj = new FmcAmplifier();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcAmplifier::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("K",Rate,FmcAmplifier,retArray);
}


std::ostream& FmcAmplifier::writeFMF(std::ostream& os)
{
  os <<"CONTROL_AMPLIFIER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcAmplifier::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Amplifier\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 3\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myRate.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcAmplifier::readAndConnect(std::istream& is, std::ostream&)
{
  FmcAmplifier* obj = new FmcAmplifier();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcAmplifier::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcAmplifier::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcAmplifier::getClassTypeID());
}

/******************************************************************************/

FmcCompConjPole::FmcCompConjPole()
{
  Fmd_CONSTRUCTOR_INIT(FmcCompConjPole);
  this->setPortCount(1);
  this->setStateVarCount(1);

  FFA_FIELD_INIT(myKp,      1.0, "KP");
  FFA_FIELD_INIT(myResFreq, 2.0, "RES_FREQ");
  FFA_FIELD_INIT(myDampFac, 1.0, "DAMP_FACTOR");

  itsPixmap = ctrlCompConjPoleSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcCompConjPole::copy()
{
  FmcCompConjPole* newObj = new FmcCompConjPole();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcCompConjPole::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("K",Kp,FmcCompConjPole,retArray);
  M_APPEND_CTRL_VARS("Undamped resonance freqency",ResFreq,FmcCompConjPole,retArray);
  M_APPEND_CTRL_VARS("Damping factor",DampFac,FmcCompConjPole,retArray);
}


std::ostream& FmcCompConjPole::writeFMF(std::ostream& os)
{
  os <<"CONTROL_COMPCONJPOLE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcCompConjPole::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Compl. conj. pole\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 42\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myResFreq.getValue(), myDampFac.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcCompConjPole::readAndConnect(std::istream& is, std::ostream&)
{
  FmcCompConjPole* obj = new FmcCompConjPole();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcCompConjPole::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcCompConjPole::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcCompConjPole::getClassTypeID());
}

/******************************************************************************/

FmcComparator::FmcComparator()
{
  Fmd_CONSTRUCTOR_INIT(FmcComparator);
  this->setPortCount(2);
  this->setStateVarCount(0);

  itsPixmap = ctrlComparatorSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcComparator::copy()
{
  FmcComparator* newObj = new FmcComparator();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


std::ostream& FmcComparator::writeFMF(std::ostream& os)
{
  os <<"CONTROL_COMPARATOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcComparator::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Comparator\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 1\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", 1.0);
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcComparator::readAndConnect(std::istream& is, std::ostream&)
{
  FmcComparator* obj = new FmcComparator();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcComparator::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcComparator::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcComparator::getClassTypeID());
}

/******************************************************************************/

FmcDeadZone::FmcDeadZone()
{
  Fmd_CONSTRUCTOR_INIT(FmcDeadZone);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myLeft, -0.5, "LEFT");
  FFA_FIELD_INIT(myRight, 0.5, "RIGHT");

  itsPixmap = ctrlDeadZoneSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcDeadZone::copy()
{
  FmcDeadZone* newObj = new FmcDeadZone();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcDeadZone::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Left",Left,FmcDeadZone,retArray);
  M_APPEND_CTRL_VARS("Right",Right,FmcDeadZone,retArray);
}


std::ostream& FmcDeadZone::writeFMF(std::ostream& os)
{
  os <<"CONTROL_DEAD_ZONE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcDeadZone::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Dead Zone\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 23\n");
  fprintf(fp,"  nRealData = 2\n");
  fprintf(fp,"  realData = %14.6e %14.6e\n",
	  myLeft.getValue(), myRight.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcDeadZone::readAndConnect(std::istream& is, std::ostream&)
{
  FmcDeadZone* obj = new FmcDeadZone();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcDeadZone::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcDeadZone::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcDeadZone::getClassTypeID());
}

/******************************************************************************/

FmcHysteresis::FmcHysteresis()
{
  Fmd_CONSTRUCTOR_INIT(FmcHysteresis);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myLeft, -0.5, "LEFT");
  FFA_FIELD_INIT(myRight, 0.5, "RIGHT");
  FFA_FIELD_INIT(myAlpha, 1.0, "ALPHA");

  itsPixmap = ctrlHysteresisSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcHysteresis::copy()
{
  FmcHysteresis *newObj = new FmcHysteresis();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcHysteresis::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Left",Left,FmcHysteresis,retArray);
  M_APPEND_CTRL_VARS("Right",Right,FmcHysteresis,retArray);
  M_APPEND_CTRL_VARS("Alpha",Alpha,FmcHysteresis,retArray);
}


std::ostream& FmcHysteresis::writeFMF(std::ostream &os)
{
  os <<"CONTROL_HYSTERESIS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcHysteresis::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Hysteresis\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 24\n");
  fprintf(fp,"  nRealData = 9\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e 0. 0. 0. 0. 0. 0.\n",
	  myLeft.getValue(), myRight.getValue(), myAlpha.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcHysteresis::readAndConnect(std::istream& is, std::ostream&)
{
  FmcHysteresis* obj = new FmcHysteresis();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcHysteresis::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcHysteresis::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcHysteresis::getClassTypeID());
}

/******************************************************************************/

FmcIntegrator::FmcIntegrator()
{
  Fmd_CONSTRUCTOR_INIT(FmcIntegrator);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myInitialValue, 0.0, "INITIAL_VALUE");

  itsPixmap = ctrlIntegratorSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcIntegrator::copy()
{
  FmcIntegrator *newObj = new FmcIntegrator();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcIntegrator::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("InitialValue",InitialValue,FmcIntegrator,retArray);
}


std::ostream& FmcIntegrator::writeFMF(std::ostream& os)
{
  os <<"CONTROL_INTEGRATOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcIntegrator::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Integrator\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 4\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myInitialValue.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcIntegrator::readAndConnect(std::istream& is, std::ostream&)
{
  FmcIntegrator *obj = new FmcIntegrator();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcIntegrator::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcIntegrator::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcIntegrator::getClassTypeID());
}

/******************************************************************************/

FmcLimDerivator::FmcLimDerivator()
{
  Fmd_CONSTRUCTOR_INIT(FmcLimDerivator);
  this->setPortCount(1);
  this->setStateVarCount(1);

  FFA_FIELD_INIT(myTfd, 1.0, "TFD");

  itsPixmap = ctrlLimDerivatorSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcLimDerivator::copy()
{
  FmcLimDerivator* newObj = new FmcLimDerivator();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcLimDerivator::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("T",Tfd,FmcLimDerivator,retArray);
}


std::ostream& FmcLimDerivator::writeFMF(std::ostream& os)
{
  os <<"CONTROL_LIM_DERIVATOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcLimDerivator::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Lim. derivator\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 5\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myTfd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcLimDerivator::readAndConnect(std::istream& is, std::ostream&)
{
  FmcLimDerivator* obj = new FmcLimDerivator();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcLimDerivator::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcLimDerivator::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcLimDerivator::getClassTypeID());
}

/******************************************************************************/

FmcLimitation::FmcLimitation()
{
  Fmd_CONSTRUCTOR_INIT(FmcLimitation);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myLower, -0.5, "LOWER");
  FFA_FIELD_INIT(myUpper,  0.5, "UPPER");

  itsPixmap = ctrlLimitationSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcLimitation::copy()
{
  FmcLimitation *newObj = new FmcLimitation();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcLimitation::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Upper",Upper,FmcLimitation,retArray);
  M_APPEND_CTRL_VARS("Lower",Lower,FmcLimitation,retArray);
}


std::ostream& FmcLimitation::writeFMF(std::ostream& os)
{
  os <<"CONTROL_LIMITATION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcLimitation::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Limitation\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 22\n");
  fprintf(fp,"  nRealData = 2\n");
  fprintf(fp,"  realData = %14.6e %14.6e\n",
	  myLower.getValue(), myUpper.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcLimitation::readAndConnect(std::istream& is, std::ostream&)
{
  FmcLimitation* obj = new FmcLimitation();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcLimitation::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcLimitation::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcLimitation::getClassTypeID());
}

/******************************************************************************/

FmcLogicalSwitch::FmcLogicalSwitch()
{
  Fmd_CONSTRUCTOR_INIT(FmcLogicalSwitch);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myLower, -0.5, "LOWER");
  FFA_FIELD_INIT(myUpper,  0.5, "UPPER");
  FFA_FIELD_INIT(myYOn,   1.0,  "Y_ON");

  itsPixmap = ctrlLogicalSwitchSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcLogicalSwitch::copy()
{
  FmcLogicalSwitch* newObj = new FmcLogicalSwitch();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcLogicalSwitch::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Upper",Upper,FmcLogicalSwitch,retArray);
  M_APPEND_CTRL_VARS("Lower",Lower,FmcLogicalSwitch,retArray);
  M_APPEND_CTRL_VARS("Y on",YOn,FmcLogicalSwitch,retArray);
}


std::ostream& FmcLogicalSwitch::writeFMF(std::ostream& os)
{
  os <<"CONTROL_LOGICAL_SWITCH\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcLogicalSwitch::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Logical Switch\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 21\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myYOn.getValue(), myLower.getValue(), myUpper.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcLogicalSwitch::readAndConnect(std::istream& is, std::ostream&)
{
  FmcLogicalSwitch *obj = new FmcLogicalSwitch();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcLogicalSwitch::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcLogicalSwitch::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcLogicalSwitch::getClassTypeID());
}

/******************************************************************************/

FmcMultiplier::FmcMultiplier()
{
  Fmd_CONSTRUCTOR_INIT(FmcMultiplier);
  this->setPortCount(2);
  this->setStateVarCount(0);

  itsPixmap = ctrlMultiplierSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcMultiplier::copy()
{
  FmcMultiplier* newObj = new FmcMultiplier();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


std::ostream& FmcMultiplier::writeFMF(std::ostream& os)
{
  os <<"CONTROL_MULTIPLIER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcMultiplier::printSolverEntry(FILE *fp)
{
  fprintf(fp,"! Multiplier\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 6\n");
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcMultiplier::readAndConnect(std::istream& is, std::ostream&)
{
  FmcMultiplier* obj = new FmcMultiplier();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcMultiplier::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcMultiplier::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcMultiplier::getClassTypeID());
}

/******************************************************************************/

FmcPIlimD::FmcPIlimD()
{
  Fmd_CONSTRUCTOR_INIT(FmcPIlimD);
  this->setPortCount(1);
  this->setStateVarCount(3);

  FFA_FIELD_INIT(myKp,     1.0, "KP");
  FFA_FIELD_INIT(myTi,  1000.0, "TI");
  FFA_FIELD_INIT(myTd,  0.0001, "TD");
  FFA_FIELD_INIT(myTfd, 0.0001, "TFD");

  itsPixmap = ctrlPIlimDSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPIlimD::copy()
{
  FmcPIlimD* newObj = new FmcPIlimD();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPIlimD::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp" ,Kp ,FmcPIlimD,retArray);
  M_APPEND_CTRL_VARS("Ti" ,Ti ,FmcPIlimD,retArray);
  M_APPEND_CTRL_VARS("Td" ,Td ,FmcPIlimD,retArray);
  M_APPEND_CTRL_VARS("Tfd",Tfd,FmcPIlimD,retArray);
}


std::ostream& FmcPIlimD::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PILIMD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPIlimD::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PIlimD\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 36\n");
  fprintf(fp,"  nRealData = 4\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myTi.getValue(), myTd.getValue(), myTfd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPIlimD::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPIlimD* obj = new FmcPIlimD();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPIlimD::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPIlimD::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPIlimD::getClassTypeID());
}

/******************************************************************************/

FmcPd::FmcPd()
{
  Fmd_CONSTRUCTOR_INIT(FmcPd);
  this->setPortCount(1);
  this->setStateVarCount(2);

  FFA_FIELD_INIT(myKp, 1.0, "KP");
  FFA_FIELD_INIT(myTd, 0.0, "TD");

  itsPixmap = ctrlPdSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPd::copy()
{
  FmcPd* newObj = new FmcPd();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPd::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,FmcPd,retArray);
  M_APPEND_CTRL_VARS("Td",Td,FmcPd,retArray);
}


std::ostream& FmcPd::writeFMF(std::ostream &os)
{
  os <<"CONTROL_PD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPd::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PD\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 33\n");
  fprintf(fp,"  nRealData = 2\n");
  fprintf(fp,"  realData = %14.6e %14.6e\n", myKp.getValue(), myTd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPd::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPd* obj = new FmcPd();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPd::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPd::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPd::getClassTypeID());
}

/******************************************************************************/

FmcPi::FmcPi()
{
  Fmd_CONSTRUCTOR_INIT(FmcPi);
  this->setPortCount(1);
  this->setStateVarCount(1);

  FFA_FIELD_INIT(myKp,    1.0, "KP");
  FFA_FIELD_INIT(myTi, 1000.0, "TI");

  itsPixmap = ctrlPiSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPi::copy()
{
  FmcPi* newObj = new FmcPi();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPi::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,FmcPi,retArray);
  M_APPEND_CTRL_VARS("Ti",Ti,FmcPi,retArray);
}


std::ostream& FmcPi::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PI\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPi::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PI\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 31\n");
  fprintf(fp,"  nRealData = 2\n");
  fprintf(fp,"  realData = %14.6e %14.6e\n", myKp.getValue(), myTi.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPi::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPi* obj = new FmcPi();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPi::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPi::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPi::getClassTypeID());
}

/******************************************************************************/

FmcPid::FmcPid()
{
  Fmd_CONSTRUCTOR_INIT(FmcPid);
  this->setPortCount(1);
  this->setStateVarCount(3);

  FFA_FIELD_INIT(myKp,    1.0, "KP");
  FFA_FIELD_INIT(myTi, 1000.0, "TI");
  FFA_FIELD_INIT(myTd,    0.0, "TD");

  itsPixmap = ctrlPidSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPid::copy()
{
  FmcPid* newObj = new FmcPid();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPid::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,FmcPid,retArray);
  M_APPEND_CTRL_VARS("Ti",Ti,FmcPid,retArray);
  M_APPEND_CTRL_VARS("Td",Td,FmcPid,retArray);
}


std::ostream& FmcPid::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PID\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPid::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PID\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 35\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myTi.getValue(), myTd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPid::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPid* obj = new FmcPid();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPid::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPid::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPid::getClassTypeID());
}

/******************************************************************************/

FmcPlimD::FmcPlimD()
{
  Fmd_CONSTRUCTOR_INIT(FmcPlimD);
  this->setPortCount(1);
  this->setStateVarCount(2);

  FFA_FIELD_INIT(myKp,     1.0, "KP");
  FFA_FIELD_INIT(myTd,  0.0001, "TD");
  FFA_FIELD_INIT(myTfd, 0.0001, "TFD");

  itsPixmap = ctrlPlimDSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPlimD::copy()
{
  FmcPlimD* newObj = new FmcPlimD();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPlimD::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,FmcPlimD,retArray);
  M_APPEND_CTRL_VARS("Td",Td,FmcPlimD,retArray);
  M_APPEND_CTRL_VARS("Tfd",Tfd,FmcPlimD,retArray);
}


std::ostream& FmcPlimD::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PLIMD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPlimD::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PlimD\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 34\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myTfd.getValue(), myTd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPlimD::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPlimD *obj = new FmcPlimD();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPlimD::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPlimD::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPlimD::getClassTypeID());
}

/******************************************************************************/

FmcPlimI::FmcPlimI()
{
  Fmd_CONSTRUCTOR_INIT(FmcPlimI);
  this->setPortCount(1);
  this->setStateVarCount(2);

  FFA_FIELD_INIT(myKp,     1.0, "KP");
  FFA_FIELD_INIT(myTi,  1000.0, "TD");
  FFA_FIELD_INIT(myTfi, 1000.0, "TFI");

  itsPixmap = ctrlPlimISymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPlimI::copy()
{
  FmcPlimI* newObj = new FmcPlimI();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPlimI::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp",Kp,FmcPlimI,retArray);
  M_APPEND_CTRL_VARS("Ti",Ti,FmcPlimI,retArray);
  M_APPEND_CTRL_VARS("Tfi",Tfi,FmcPlimI,retArray);
}


std::ostream& FmcPlimI::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PLIMI\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPlimI::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! PlimI\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 32\n");
  fprintf(fp,"  nRealData = 3\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myTi.getValue(), myTfi.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPlimI::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPlimI* obj = new FmcPlimI();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPlimI::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPlimI::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPlimI::getClassTypeID());
}

/******************************************************************************/

FmcPlimIlimD::FmcPlimIlimD()
{
  Fmd_CONSTRUCTOR_INIT(FmcPlimIlimD);
  this->setPortCount(1);
  this->setStateVarCount(3);

  FFA_FIELD_INIT(myKp,     1.0, "KP");
  FFA_FIELD_INIT(myTi,  1000.0, "TI");
  FFA_FIELD_INIT(myTd,  0.0001, "TD");
  FFA_FIELD_INIT(myTfi, 1000.0, "TFI");
  FFA_FIELD_INIT(myTfd, 0.0001, "TFD");

  itsPixmap = ctrlPlimIlimDSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPlimIlimD::copy()
{
  FmcPlimIlimD* newObj = new FmcPlimIlimD();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPlimIlimD::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Kp" ,Kp ,FmcPlimIlimD,retArray);
  M_APPEND_CTRL_VARS("Ti" ,Ti ,FmcPlimIlimD,retArray);
  M_APPEND_CTRL_VARS("Tfi",Tfi,FmcPlimIlimD,retArray);
  M_APPEND_CTRL_VARS("Td" ,Td ,FmcPlimIlimD,retArray);
  M_APPEND_CTRL_VARS("Tfd",Tfd,FmcPlimIlimD,retArray);
}


std::ostream& FmcPlimIlimD::writeFMF(std::ostream& os)
{
  os <<"CONTROL_PLIMILIMD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPlimIlimD::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! P lim.I lim.D\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 37\n");
  fprintf(fp,"  nRealData = 5\n");
  fprintf(fp,"  realData = %14.6e %14.6e %14.6e %14.6e %14.6e\n",
	  myKp.getValue(), myTi.getValue(), myTd.getValue(),
	  myTfi.getValue(), myTfd.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPlimIlimD::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPlimIlimD* obj = new FmcPlimIlimD();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPlimIlimD::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPlimIlimD::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPlimIlimD::getClassTypeID());
}

/******************************************************************************/

FmcPower::FmcPower()
{
  Fmd_CONSTRUCTOR_INIT(FmcPower);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myExponent, 1.0, "EXPONENT");

  itsPixmap = ctrlPowerSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcPower::copy()
{
  FmcPower* newObj = new FmcPower();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcPower::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Exponent",Exponent,FmcPower,retArray);
}


std::ostream& FmcPower::writeFMF(std::ostream &os)
{
  os <<"CONTROL_POWER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcPower::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Power\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 7\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myExponent.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcPower::readAndConnect(std::istream& is, std::ostream&)
{
  FmcPower* obj = new FmcPower();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcPower::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcPower::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcPower::getClassTypeID());
}

/******************************************************************************/

FmcRealPole::FmcRealPole()
{
  Fmd_CONSTRUCTOR_INIT(FmcRealPole);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myKp, 1.0, "KP");
  FFA_FIELD_INIT(myT1, 1.0, "T1");

  itsPixmap = ctrlRealPoleSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcRealPole::copy()
{
  FmcRealPole* newObj = new FmcRealPole();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcRealPole::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("K",Kp,FmcRealPole,retArray);
  M_APPEND_CTRL_VARS("T",T1,FmcRealPole,retArray);
}


std::ostream& FmcRealPole::writeFMF(std::ostream& os)
{
  os <<"CONTROL_REAL_POLE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcRealPole::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Real Pole\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 41\n");
  fprintf(fp,"  nRealData = 2\n");
  fprintf(fp,"  realData = %14.6e %14.6e\n", myKp.getValue(), myT1.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcRealPole::readAndConnect(std::istream& is, std::ostream&)
{
  FmcRealPole* obj = new FmcRealPole();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcRealPole::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcRealPole::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcRealPole::getClassTypeID());
}

/******************************************************************************/

FmcSampleHold::FmcSampleHold()
{
  Fmd_CONSTRUCTOR_INIT(FmcSampleHold);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myPeriod, 1.0, "PERIOD");

  itsPixmap = ctrlSampleHoldSymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcSampleHold::copy()
{
  FmcSampleHold* newObj = new FmcSampleHold();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcSampleHold::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("Sample period",Period,FmcSampleHold,retArray);
}


std::ostream& FmcSampleHold::writeFMF(std::ostream& os)
{
  os <<"CONTROL_SAMPLE_HOLD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcSampleHold::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! SampleHold\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 12\n");
  fprintf(fp,"  nRealData = 6\n");
  fprintf(fp,"  realData = %14.6e 0. 0. 0. 0. 0.\n", myPeriod.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcSampleHold::readAndConnect(std::istream& is, std::ostream&)
{
  FmcSampleHold* obj = new FmcSampleHold();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcSampleHold::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcSampleHold::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcSampleHold::getClassTypeID());
}

/******************************************************************************/

FmcTimeDelay::FmcTimeDelay()
{
  Fmd_CONSTRUCTOR_INIT(FmcTimeDelay);
  this->setPortCount(1);
  this->setStateVarCount(0);

  FFA_FIELD_INIT(myDelay, 1.0, "DELAY");

  itsPixmap = ctrlTimeDelaySymbol_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmCtrlElementBase* FmcTimeDelay::copy()
{
  FmcTimeDelay* newObj = new FmcTimeDelay();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


void FmcTimeDelay::getElementVariables(std::vector<ctrlVars>& retArray) const
{
  M_APPEND_CTRL_VARS("T",Delay,FmcTimeDelay,retArray);
}


std::ostream& FmcTimeDelay::writeFMF(std::ostream& os)
{
  os <<"CONTROL_TIME_DELAY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcTimeDelay::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! Time delay\n");
  fprintf(fp,"&CONTROL_ELEMENT\n");
  this->printID(fp);
  fprintf(fp,"  type = 11\n");
  fprintf(fp,"  nRealData = 1\n");
  fprintf(fp,"  realData = %14.6e\n", myDelay.getValue());
  this->printSolverTopology(fp);
  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcTimeDelay::readAndConnect(std::istream& is, std::ostream&)
{
  FmcTimeDelay* obj = new FmcTimeDelay();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	FmCtrlInputElementBase::localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmcTimeDelay::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcTimeDelay::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcTimeDelay::getClassTypeID());
}
