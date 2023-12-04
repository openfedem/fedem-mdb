// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmfUserDefined.H"
#include "FFaFunctionLib/FFaUserFuncPlugin.H"
#include "FFaFunctionLib/FFaFunctionManager.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


static void setPar01(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(1,v); }
static void setPar02(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(2,v); }
static void setPar03(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(3,v); }
static void setPar04(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(4,v); }
static void setPar05(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(5,v); }
static void setPar06(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(6,v); }
static void setPar07(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(7,v); }
static void setPar08(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(8,v); }
static void setPar09(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(9,v); }
static void setPar10(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(10,v); }
static void setPar11(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(11,v); }
static void setPar12(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(12,v); }
static void setPar13(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(13,v); }
static void setPar14(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(14,v); }
static void setPar15(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(15,v); }
static void setPar16(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(16,v); }
static void setPar17(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(17,v); }
static void setPar18(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(18,v); }
static void setPar19(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(19,v); }
static void setPar20(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(20,v); }
static void setPar21(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(21,v); }
static void setPar22(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(22,v); }
static void setPar23(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(23,v); }
static void setPar24(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(24,v); }
static void setPar25(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(25,v); }
static void setPar26(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(26,v); }
static void setPar27(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(27,v); }
static void setPar28(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(28,v); }
static void setPar29(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(29,v); }
static void setPar30(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(30,v); }
static void setPar31(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(31,v); }
static void setPar32(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(32,v); }
static void setPar33(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(33,v); }
static void setPar34(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(34,v); }
static void setPar35(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(35,v); }
static void setPar36(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(36,v); }
static void setPar37(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(37,v); }
static void setPar38(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(38,v); }
static void setPar39(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(39,v); }
static void setPar40(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(40,v); }
static void setPar41(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(41,v); }
static void setPar42(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(42,v); }
static void setPar43(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(43,v); }
static void setPar44(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(44,v); }
static void setPar45(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(45,v); }
static void setPar46(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(46,v); }
static void setPar47(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(47,v); }
static void setPar48(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(48,v); }
static void setPar49(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(49,v); }
static void setPar50(FmParamObjectBase* f, double v) { static_cast<FmfUserDefined*>(f)->setPar(50,v); }

static double getPar01(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(1); }
static double getPar02(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(2); }
static double getPar03(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(3); }
static double getPar04(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(4); }
static double getPar05(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(5); }
static double getPar06(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(6); }
static double getPar07(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(7); }
static double getPar08(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(8); }
static double getPar09(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(9); }
static double getPar10(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(10); }
static double getPar11(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(11); }
static double getPar12(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(12); }
static double getPar13(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(13); }
static double getPar14(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(14); }
static double getPar15(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(15); }
static double getPar16(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(16); }
static double getPar17(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(17); }
static double getPar18(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(18); }
static double getPar19(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(19); }
static double getPar20(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(20); }
static double getPar21(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(21); }
static double getPar22(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(22); }
static double getPar23(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(23); }
static double getPar24(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(24); }
static double getPar25(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(25); }
static double getPar26(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(26); }
static double getPar27(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(27); }
static double getPar28(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(28); }
static double getPar29(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(29); }
static double getPar30(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(30); }
static double getPar31(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(31); }
static double getPar32(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(32); }
static double getPar33(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(33); }
static double getPar34(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(34); }
static double getPar35(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(35); }
static double getPar36(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(36); }
static double getPar37(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(37); }
static double getPar38(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(38); }
static double getPar39(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(39); }
static double getPar40(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(40); }
static double getPar41(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(41); }
static double getPar42(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(42); }
static double getPar43(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(43); }
static double getPar44(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(44); }
static double getPar45(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(45); }
static double getPar46(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(46); }
static double getPar47(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(47); }
static double getPar48(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(48); }
static double getPar49(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(49); }
static double getPar50(FmParamObjectBase* f) { return static_cast<FmfUserDefined*>(f)->getPar(50); }


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfUSER_DEFINED, FmfUserDefined, FmfMultiArgBase);


FmfUserDefined::FmfUserDefined()
{
  Fmd_CONSTRUCTOR_INIT(FmfUserDefined);

  FFA_FIELD_INIT(myFuncId, 0, "FUNCTION_ID");
  FFA_FIELD_INIT(myFuncNo, 0, "FUNCTION_NO");

  FFA_FIELD_DEFAULT_INIT(myParameters, "PARAMETERS");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmfUserDefined::getFunctionVariables(std::vector<FmFuncVariable>& var, bool) const
{
  char par[64];
  int ipar = 0, fId = myFuncId.getValue();
  while (FFaUserFuncPlugin::instance()->getParName(fId,++ipar,64,par) > 0)
    switch (ipar) {
    case  1: var.push_back(FmParameter(par,setPar01,getPar01)); break;
    case  2: var.push_back(FmParameter(par,setPar02,getPar02)); break;
    case  3: var.push_back(FmParameter(par,setPar03,getPar03)); break;
    case  4: var.push_back(FmParameter(par,setPar04,getPar04)); break;
    case  5: var.push_back(FmParameter(par,setPar05,getPar05)); break;
    case  6: var.push_back(FmParameter(par,setPar06,getPar06)); break;
    case  7: var.push_back(FmParameter(par,setPar07,getPar07)); break;
    case  8: var.push_back(FmParameter(par,setPar08,getPar08)); break;
    case  9: var.push_back(FmParameter(par,setPar09,getPar09)); break;
    case 10: var.push_back(FmParameter(par,setPar10,getPar10)); break;
    case 11: var.push_back(FmParameter(par,setPar11,getPar11)); break;
    case 12: var.push_back(FmParameter(par,setPar12,getPar12)); break;
    case 13: var.push_back(FmParameter(par,setPar13,getPar13)); break;
    case 14: var.push_back(FmParameter(par,setPar14,getPar14)); break;
    case 15: var.push_back(FmParameter(par,setPar15,getPar15)); break;
    case 16: var.push_back(FmParameter(par,setPar16,getPar16)); break;
    case 17: var.push_back(FmParameter(par,setPar17,getPar17)); break;
    case 18: var.push_back(FmParameter(par,setPar18,getPar18)); break;
    case 19: var.push_back(FmParameter(par,setPar19,getPar19)); break;
    case 20: var.push_back(FmParameter(par,setPar20,getPar20)); break;
    case 21: var.push_back(FmParameter(par,setPar21,getPar21)); break;
    case 22: var.push_back(FmParameter(par,setPar22,getPar22)); break;
    case 23: var.push_back(FmParameter(par,setPar23,getPar23)); break;
    case 24: var.push_back(FmParameter(par,setPar24,getPar24)); break;
    case 25: var.push_back(FmParameter(par,setPar25,getPar25)); break;
    case 26: var.push_back(FmParameter(par,setPar26,getPar26)); break;
    case 27: var.push_back(FmParameter(par,setPar27,getPar27)); break;
    case 28: var.push_back(FmParameter(par,setPar28,getPar28)); break;
    case 29: var.push_back(FmParameter(par,setPar29,getPar29)); break;
    case 30: var.push_back(FmParameter(par,setPar30,getPar30)); break;
    case 31: var.push_back(FmParameter(par,setPar31,getPar31)); break;
    case 32: var.push_back(FmParameter(par,setPar32,getPar32)); break;
    case 33: var.push_back(FmParameter(par,setPar33,getPar33)); break;
    case 34: var.push_back(FmParameter(par,setPar34,getPar34)); break;
    case 35: var.push_back(FmParameter(par,setPar35,getPar35)); break;
    case 36: var.push_back(FmParameter(par,setPar36,getPar36)); break;
    case 37: var.push_back(FmParameter(par,setPar37,getPar37)); break;
    case 38: var.push_back(FmParameter(par,setPar38,getPar38)); break;
    case 39: var.push_back(FmParameter(par,setPar39,getPar39)); break;
    case 40: var.push_back(FmParameter(par,setPar40,getPar40)); break;
    case 41: var.push_back(FmParameter(par,setPar41,getPar41)); break;
    case 42: var.push_back(FmParameter(par,setPar42,getPar42)); break;
    case 43: var.push_back(FmParameter(par,setPar43,getPar43)); break;
    case 44: var.push_back(FmParameter(par,setPar44,getPar44)); break;
    case 45: var.push_back(FmParameter(par,setPar45,getPar45)); break;
    case 46: var.push_back(FmParameter(par,setPar46,getPar46)); break;
    case 47: var.push_back(FmParameter(par,setPar47,getPar47)); break;
    case 48: var.push_back(FmParameter(par,setPar48,getPar48)); break;
    case 49: var.push_back(FmParameter(par,setPar49,getPar49)); break;
    case 50: var.push_back(FmParameter(par,setPar50,getPar50)); break;
    }

  if (ipar > 50)
    ListUI <<"===> ERROR: User-defined function (ID="<< fId
	   <<") is defined with "<< ipar <<" parameters.\n"
	   <<"            Only the 50 first parameters will be used.\n";
}


void FmfUserDefined::setPar(int iPar, double v)
{
  if (iPar < 1) return;

  int nPar = myParameters.getValue().size();
  if (iPar > nPar)
  {
    myParameters.getValue().resize(iPar);
    for (int i = nPar; i < iPar-1; i++)
      myParameters.getValue()[i] = FFaUserFuncPlugin::instance()->getDefaultParVal(myFuncId.getValue(),1+i);
  }
  myParameters.getValue()[iPar-1] = v;
}


double FmfUserDefined::getPar(int iPar) const
{
  if (iPar < 1)
    return 0.0;
  else if (iPar <= (int)myParameters.getValue().size())
    return myParameters.getValue()[iPar-1];

  return FFaUserFuncPlugin::instance()->getDefaultParVal(myFuncId.getValue(),iPar);
}


const char** FmfUserDefined::getPixmap() const
{
  const char** pix = FFaUserFuncPlugin::instance()->getPixmap(myFuncId.getValue());
  return pix ? pix : this->FmfMultiArgBase::getPixmap();
}


bool FmfUserDefined::isSurfaceFunc() const
{
  if (this->getFunctionUse() != FmMathFuncBase::WAVE_FUNCTION)
    return false;

  return FFaUserFuncPlugin::instance()->getFuncName(myFuncId.getValue()) == 4;
}


bool FmfUserDefined::initGetValueNoRecursion()
{
  int n = FFaUserFuncPlugin::instance()->getParName(myFuncId.getValue());
  int nPar = myParameters.getValue().size();
  if (n > nPar)
  {
    myParameters.getValue().resize(n);
    for (int i = nPar; i < n; i++)
      myParameters.getValue()[i] = FFaUserFuncPlugin::instance()->getDefaultParVal(myFuncId.getValue(),1+i);
  }

  if (this->getFunctionUse() == FmMathFuncBase::WAVE_FUNCTION)
    if (FFaUserFuncPlugin::instance()->getFuncName(myFuncId.getValue()) == 4)
      myExplType = FFaFunctionManager::getTypeID(this->getFunctionFsiName())
                 + myFuncId.getValue()*100;

  return true;
}


double FmfUserDefined::getValueNoRecursion(double x, int& ierr) const
{
  static DoubleVec args(10,0.0); args.front() = x;
  return FFaUserFuncPlugin::instance()->getValue(this->getBaseID(),
						 myFuncId.getValue(),
						 &myParameters.getValue().front(),
						 &args.front(), ierr);
}


double FmfUserDefined::getValue(const DoubleVec& x, int& ierr) const
{
  return FFaUserFuncPlugin::instance()->getValue(this->getBaseID(),
						 myFuncId.getValue(),
						 &myParameters.getValue().front(),
						 &x.front(), ierr);
}


unsigned int FmfUserDefined::getNoArgs() const
{
  int n = FFaUserFuncPlugin::instance()->getFuncName(myFuncId.getValue());
  return n > 0 ? n : 0;
}


#define MAX_UDF 400

bool FmfUserDefined::setFuncId(int fId)
{
  int funcId[MAX_UDF];
  int nFunc = FFaUserFuncPlugin::instance()->getFuncs(MAX_UDF,funcId);
  for (int i = 0; i < nFunc; i++)
    if (fId == funcId[i])
    {
      myFuncId.setValue(fId);
      myFuncNo.setValue(i+1);
      return true;
    }

  ListUI <<"===> ERROR: No function with ID "<< fId
	 <<" exist in the user-defined function plug-in.\n";
  return false;
}


bool FmfUserDefined::setFuncNo(int fNo)
{
  int funcId[MAX_UDF];
  if (fNo > FFaUserFuncPlugin::instance()->getFuncs(MAX_UDF,funcId))
    return false;

  myFuncNo.setValue(fNo);
  myFuncId.setValue(funcId[fNo-1]);
  return true;
}


std::ostream& FmfUserDefined::writeFMF(std::ostream& os)
{
  os <<"FUNC_USER_DEFINED\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfUserDefined::printSolverData(FILE* fp)
{
  const DoubleVec& itsVals = myParameters.getValue();
  fprintf(fp,"  channel = %d\n", myFuncId.getValue());
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


bool FmfUserDefined::readAndConnect(std::istream& is, std::ostream&)
{
  FmfUserDefined* obj = new FmfUserDefined();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmMathFuncBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmfUserDefined::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfUserDefined::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfUserDefined::getClassTypeID());
}
