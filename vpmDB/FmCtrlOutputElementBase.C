// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmCtrlOutputElementBase.H"
#include "vpmDB/FmCtrlLine.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FccOUTPUT_ELEMENT_BASE, FmCtrlOutputElementBase, FmCtrlElementBase);


FmCtrlOutputElementBase::FmCtrlOutputElementBase()
{
  Fmd_CONSTRUCTOR_INIT(FmCtrlOutputElementBase);
}


FmCtrlOutputElementBase::~FmCtrlOutputElementBase()
{
  std::vector<FmCtrlLine*> allLines;
  this->getLines(allLines);
  for (FmCtrlLine* line : allLines)
    line->erase();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmCtrlOutputElementBase::setOutputVarNo(int no)
{
  std::vector<FmCtrlLine*> allLines;
  this->getLines(allLines);
  for (FmCtrlLine* line : allLines)
    line->setControlVarNo(no);
}


int FmCtrlOutputElementBase::renumberLocalVariables(int input)
{
  for (int& stateVar : itsStateVariables)
    stateVar = input++;
  this->setOutputVarNo(input++);
  return input;
}


void FmCtrlOutputElementBase::printVariables(FILE* fp)
{
  for (int& stateVar : itsStateVariables)
    fprintf(fp," %d", stateVar);
  fprintf(fp,"  ");
}


bool FmCtrlOutputElementBase::hasCtrlLines() const
{
  FmCtrlLine* line = NULL;
  return this->hasReferringObjs(line,"myStartCtrlBlock");
}


void FmCtrlOutputElementBase::getLines(std::vector<FmCtrlLine*>& lines) const
{
  lines.clear();
  this->getReferringObjs(lines,"myStartCtrlBlock");
}


/*!
  If \a single is \e true, the two nearest line segments
  are updated and the others are not changed.
  This is used when only one element is moved.
  If \a single is \e false, then all line segments in the line are updated,
  this is used when a group of elements are moved.
*/

void FmCtrlOutputElementBase::updateLines(bool single)
{
  std::vector<FmCtrlLine*> allLines;
  this->getLines(allLines);
  for (FmCtrlLine* line : allLines)
  {
    if (single)
    {
      DoubleVec lengths = line->getTotLengthArray();
      line->setLengthArray(DoubleVec(lengths.begin()+2,lengths.end()));
      line->setFirstUndefSegment(1);
    }
    line->draw();
  }

  this->FmCtrlElementBase::updateLines(single);
}


bool FmCtrlOutputElementBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmCtrlOutputElementBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmCtrlOutputElementBase* copyObj = static_cast<FmCtrlOutputElementBase*>(obj);

  std::vector<FmCtrlLine*> allLines;
  copyObj->getLines(allLines);
  for (FmCtrlLine* line : allLines)
    line->setStartElement(this);

  return true;
}


bool FmCtrlOutputElementBase::localParse(const char* keyWord, std::istream& activeStatement,
					 FmCtrlOutputElementBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}
