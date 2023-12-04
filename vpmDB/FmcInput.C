// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmcInput.H"
#include "vpmDB/FmCtrlLine.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdCtrlElement.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FccINPUT, FmcInput, FmCtrlOutputElementBase);

FmcInput::FmcInput()
{
  Fmd_CONSTRUCTOR_INIT(FmcInput);

  FFA_REFERENCE_FIELD_INIT(myEngineField, myEngine, "ENGINE");

  itsPixmap = ctrlElemIn_xpm;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlElement(this);
#endif
}


FmcInput::~FmcInput()
{
  if (myEngine.isNull())
    return;

  std::vector<FmModelMemberBase*> controlled;
  myEngine->getReferringObjs(controlled);
  if (controlled.size() == 1 && controlled.front() == this)
    myEngine->erase();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmcInput::getListViewPixmap() const
{
  if (!myEngine.isNull())
    if (myEngine->isExternalFunc())
      return external_xpm;

  return NULL;
}


FmCtrlElementBase* FmcInput::copy()
{
  FmcInput* newObj = new FmcInput();
  newObj->clone(this,FmBase::SHALLOW);
  newObj->makeCopyDescr();
  return newObj;
}


std::ostream& FmcInput::writeFMF(std::ostream& os)
{
  os <<"CONTROL_INPUT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


int FmcInput::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&CONTROL_INPUT\n");
  this->printID(fp);

  std::vector<FmCtrlLine*> lines;
  this->getLines(lines);
  if (!lines.empty())
    fprintf(fp,"  iVar = %d\n", lines.front()->getControlVarNo());

  if (!myEngine.isNull())
    fprintf(fp,"  inEngineID = %d\n", myEngine->getBaseID());

  fprintf(fp,"/\n\n");
  return 0;
}


bool FmcInput::readAndConnect(std::istream& is, std::ostream&)
{
  FmcInput* obj = new FmcInput();

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


bool FmcInput::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmcInput::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmcInput::getClassTypeID());
}
