// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmfMathExpr.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaMathExpr/FFaMathExprFactory.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfMATH_EXPRESSION, FmfMathExpr, FmMathFuncBase);

FmfMathExpr::FmfMathExpr(const char* expr)
{
  Fmd_CONSTRUCTOR_INIT(FmfMathExpr);

  FFA_FIELD_DEFAULT_INIT(expression, "EXPRESSION");
  FFA_FIELD_INIT(numArg, 1, "NUM_ARG");

  if (expr) expression.setValue(expr);
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmfMathExpr::initGetValueNoRecursion()
{
  if (FFaMathExprFactory::instance()->create(this->getBaseID(),
					     expression.getValue(),
					     numArg.getValue()) > 0)
    return true;

  ListUI <<"ERROR: Invalid expression for "<< this->getIdString()
	 <<":\n'"<< expression.getValue() <<"'.\n";
  return false;
}


double FmfMathExpr::getValueNoRecursion(double x, int& ierr) const
{
  return FFaMathExprFactory::instance()->getValue(this->getBaseID(),x,ierr);
}


double FmfMathExpr::getValue(const std::vector<double>& x, int& ierr) const
{
  return FFaMathExprFactory::instance()->getValue(this->getBaseID(),
						  &x.front(),ierr);
}


std::ostream& FmfMathExpr::writeFMF(std::ostream& os)
{
  os <<"FUNC_MATH_EXPRESSION\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmfMathExpr::printSolverData(FILE* fp)
{
  int err = 0;
  if (expression.getValue().empty())
  {
    err = 1;
    ListUI <<"ERROR: Empty expression for "<< this->getIdString() <<"\n";
  }
  else
    fprintf(fp,"  expression = '%s'\n", expression.getValue().c_str());

  fprintf(fp,"  nArg = %d\n", numArg.getValue());
  return err;
}


bool FmfMathExpr::readAndConnect(std::istream& is, std::ostream&)
{
  FmfMathExpr* obj = new FmfMathExpr();

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


bool FmfMathExpr::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfMathExpr::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfMathExpr::getClassTypeID());
}
