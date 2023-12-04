// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmCtrlInputElementBase.H"
#include "vpmDB/FmCtrlLine.H"
#include <algorithm>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FccINPUT_ELEMENT_BASE, FmCtrlInputElementBase, FmCtrlOutputElementBase);

FmCtrlInputElementBase::FmCtrlInputElementBase()
{
  Fmd_CONSTRUCTOR_INIT(FmCtrlInputElementBase);
}


FmCtrlInputElementBase::~FmCtrlInputElementBase()
{
  for (FmCtrlLine* line : itsInputs)
    if (line) line->erase();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

FmCtrlLine* FmCtrlInputElementBase::getLine(int portNo) const
{
  return --portNo < 0 || portNo >= (int)itsInputs.size() ? NULL : itsInputs[portNo];
}


bool FmCtrlInputElementBase::setLine(int portNo, FmCtrlLine* line)
{
  if (--portNo < 0 || portNo >= (int)itsInputs.size())
    return false;

  if (itsInputs[portNo])
    itsInputs[portNo]->setEndElement(NULL);

  itsInputs[portNo] = line;
  line->setEndElement(this);
  return true;
}


FmCtrlLine* FmCtrlInputElementBase::releaseFromPort(int portNo)
{
  if (--portNo < 0 || portNo >= (int)itsInputs.size())
    return NULL;

  FmCtrlLine* tmpLine = itsInputs[portNo];
  if (tmpLine)
    tmpLine->setEndElement(NULL);

  itsInputs[portNo] = NULL;
  return tmpLine;
}


bool FmCtrlInputElementBase::releaseFromPort(FmCtrlLine* line)
{
  if (line)
    for (FmCtrlLine*& input : itsInputs)
      if (input == line)
      {
        line->setEndElement(NULL);
        input = NULL;
        return true;
      }

  return false;
}


int FmCtrlInputElementBase::atWhatPort(const FmCtrlLine* line) const
{
  std::vector<FmCtrlLine*>::const_iterator it = std::find(itsInputs.begin(),itsInputs.end(),line);
  return it == itsInputs.end() ? -1 : (it-itsInputs.begin()) + 1;
}


bool FmCtrlInputElementBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmCtrlInputElementBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmCtrlInputElementBase* copyObj = static_cast<FmCtrlInputElementBase*>(obj);
  if (copyObj->getNumInputPorts() != this->getNumInputPorts())
    return false;

  FmCtrlLine* tpt;
  for (int i = 1; i <= copyObj->getNumInputPorts(); i++)
    if ((tpt = copyObj->releaseFromPort(i)))
      this->setLine(i,tpt);

  return true;
}


bool FmCtrlInputElementBase::localParse(const char* keyWord, std::istream& activeStatement,
                                        FmCtrlInputElementBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


void FmCtrlInputElementBase::printSolverTopology(FILE* fp)
{
  // input ports
  fprintf(fp,"  variables =");
  for (FmCtrlLine* line : itsInputs)
    fprintf(fp," %d", line->getControlVarNo());
  fprintf(fp,"  ");

  // state variables
  this->printVariables(fp);

  // output variable
  std::vector<FmCtrlLine*> lines;
  this->getLines(lines);
  if (!lines.empty())
    fprintf(fp," %d\n", lines.front()->getControlVarNo());
}
