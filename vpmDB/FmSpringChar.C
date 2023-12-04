// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSpringChar.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"


Fmd_DB_SOURCE_INIT(FcSPRING_CHAR, FmSpringChar, FmStructPropertyBase);


FmSpringChar::FmSpringChar()
{
  Fmd_CONSTRUCTOR_INIT(FmSpringChar);

  FFA_FIELD_INIT(myUse, TRANSLATION, "SPRING_CHAR_USE");

  FFA_FIELD_INIT(springStiffness, 0.0, "SPRING_STIFFNESS");
  FFA_REFERENCE_FIELD_INIT(springFunctionField, springFunction, "SPRING_FUNCTION");
  springFunction.setPrintIfZero(false);

  FFA_FIELD_INIT(deflectionMaxIsOn, false, "DEFLECTION_MAX_IS_ON");
  FFA_FIELD_INIT(deflectionMax,       0.0, "DEFLECTION_MAX");
  FFA_FIELD_INIT(deflectionMinIsOn, false, "DEFLECTION_MIN_IS_ON");
  FFA_FIELD_INIT(deflectionMin,       0.0, "DEFLECTION_MIN");
  FFA_FIELD_INIT(forceMaxIsOn,      false, "FORCE_MAX_IS_ON");
  FFA_FIELD_INIT(forceMax,            0.0, "FORCE_MAX");
  FFA_FIELD_INIT(forceMinIsOn,      false, "FORCE_MIN_IS_ON");
  FFA_FIELD_INIT(forceMin,            0.0, "FORCE_MIN");

  FFA_FIELD_INIT(yieldForceMaxIsOn, false, "YIELD_FORCE_MAX_IS_ON");
  FFA_FIELD_INIT(yieldForceMax,       0.0, "YIELD_FORCE_MAX");
  FFA_REFERENCE_FIELD_INIT(yieldForceMaxEngineField, yieldForceMaxEngine, "YIELD_FORCE_MAX_ENGINE");
  yieldForceMaxEngine.setPrintIfZero(false);

  FFA_FIELD_INIT(yieldForceMinIsOn, false, "YIELD_FORCE_MIN_IS_ON");
  FFA_FIELD_INIT(yieldForceMin,       0.0, "YIELD_FORCE_MIN");
  FFA_REFERENCE_FIELD_INIT(yieldForceMinEngineField, yieldForceMinEngine, "YIELD_FORCE_MIN_ENGINE");
  yieldForceMinEngine.setPrintIfZero(false);

  FFA_FIELD_INIT(yieldDeflectionMaxIsOn, false, "YIELD_DEFLECTION_MAX_IS_ON");
  FFA_FIELD_INIT(yieldDeflectionMax,       0.0, "YIELD_DEFLECTION_MAX");
}


FmSpringChar::~FmSpringChar()
{
  this->disconnect();
}


bool FmSpringChar::hasFailure() const
{
  if (deflectionMaxIsOn.getValue() || deflectionMinIsOn.getValue())
    return true;
  else if (forceMaxIsOn.getValue() || forceMinIsOn.getValue())
    return true;
  else
    return false;
}


bool FmSpringChar::hasYield() const
{
  if (yieldForceMaxIsOn.getValue() || yieldForceMinIsOn.getValue())
    return true;
  else
    return false;
}


std::ostream& FmSpringChar::writeFMF(std::ostream& os)
{
  os <<"SPRING_CHAR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmSpringChar::readAndConnect(std::istream& is, std::ostream&)
{
  FmSpringChar* obj = new FmSpringChar();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmSpringChar::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmSpringChar::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmSpringChar::getClassTypeID());
}


int FmSpringChar::printSolverEntry(FILE* fp)
{
  if (this->hasFailure())
  {
    fprintf(fp,"&SPRING_FAILURE\n");
    this->printID(fp);
    if (this->deflectionMaxIsOn.getValue())
      fprintf(fp, "  deflectionMax = %17.9e\n", this->deflectionMax.getValue());
    if (this->deflectionMinIsOn.getValue())
      fprintf(fp, "  deflectionMin = %17.9e\n", this->deflectionMin.getValue());
    if (this->forceMaxIsOn.getValue())
      fprintf(fp, "  forceMax = %17.9e\n", this->forceMax.getValue());
    if (this->forceMinIsOn.getValue())
      fprintf(fp, "  forceMin = %17.9e\n", this->forceMin.getValue());
    if (FFaString(this->getUserDescription()).hasSubString("#FailAll"))
      fprintf(fp, "  compFailure = .true.\n");
    fprintf(fp,"/\n\n");
  }

  if (this->hasYield())
  {
    fprintf(fp,"&SPRING_YIELD\n");
    this->printID(fp);
    if (this->yieldForceMaxIsOn.getValue())
    {
      if (this->yieldForceMaxEngine)
	fprintf(fp, "  yieldForceMaxEngine = %d\n", this->yieldForceMaxEngine->getBaseID());
      else
	fprintf(fp, "  yieldForceMax = %17.9e\n", this->yieldForceMax.getValue());
    }
    if (this->yieldForceMinIsOn.getValue())
    {
      if (this->yieldForceMinEngine)
	fprintf(fp, "  yieldForceMinEngine = %d\n", this->yieldForceMinEngine->getBaseID());
      else
	fprintf(fp, "  yieldForceMin = %17.9e\n", this->yieldForceMin.getValue());
    }
    if (this->yieldDeflectionMaxIsOn.getValue())
      fprintf(fp, "  yieldDeflectionAbsMax = %17.9e\n", this->yieldDeflectionMax.getValue());

    fprintf(fp,"/\n\n");
  }

  return 0;
}
