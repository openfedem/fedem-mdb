// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmRingStart.C
  \brief Class representing the start (and end) of a linked list of objects.
*/

#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


//! \cond DO_NOT_DOCUMENT
Fmd_SOURCE_INIT(FcRING_START, FmRingStart, FmModelMemberBase);
//! \endcond


FmRingStart::FmRingStart(const std::string& uistr, const char** pixmap,
                         bool doPrintHeader) : FmModelMemberBase(true)
{
  Fmd_CONSTRUCTOR_INIT(FmRingStart);

  myRingMemberType = -1;
  myUIString = uistr;
  myPixmap = pixmap;
  myParent = NULL;

  static int sortNumber = 0;
  mySortNumber = ++sortNumber;
  myPrintHeader = doPrintHeader;
}


void FmRingStart::setUITypeName(const std::string& name, const char** pixmap)
{
  myUIString = name;
  if (pixmap) myPixmap = pixmap;
}


bool FmRingStart::hasRingMembers(bool noChildren) const
{
  if (noChildren || myChildren.empty())
    return this->getNext() != const_cast<FmRingStart*>(this);

  for (FmRingStart* child : myChildren)
    if (child->hasRingMembers())
      return true;

  return false;
}


int FmRingStart::countRingMembers() const
{
  FmRingStart* last = const_cast<FmRingStart*>(this);

  int count = 0;
  for (FmBase* p = this->getNext(); p != last; p = p->getNext())
    count++;

  return count;
}


void FmRingStart::displayRingMembers() const
{
  if (!this->hasRingMembers(true)) return;

  FmRingStart* last = const_cast<FmRingStart*>(this);

  if (myRingMemberType == FmPart::getClassTypeID())
  {
    int partNumber = 0;
    int numOfParts = this->countRingMembers();
    FFaMsg::enableSubSteps(numOfParts);
    FFaMsg::enableProgress(numOfParts);
    for (FmBase* p = this->getNext(); p != last; p = p->getNext())
    {
      FFaMsg::setSubStep(++partNumber);
      FFaMsg::setSubTask(static_cast<FmPart*>(p)->getBaseFTLName());
      FFaMsg::setProgress(partNumber);
      p->draw();
    }
    FFaMsg::disableProgress();
    FFaMsg::disableSubSteps();
    FFaMsg::setSubTask("");
  }
  else
    for (FmBase* p = this->getNext(); p != last; p = p->getNext())
      p->draw();
}


bool FmRingStart::eraseRingMembers(bool showProgress)
{
  if (!this->hasRingMembers(true)) return false;

  if (showProgress)
  {
    FFaMsg::enableSubSteps(this->countRingMembers());
    FFaMsg::setSubTask(this->getUITypeName());
  }

  int count = 0;
  while (this->getNext() != const_cast<FmRingStart*>(this))
    if (this->getNext()->erase())
      if (showProgress)
        FFaMsg::setSubStep(++count);

  if (showProgress)
  {
    FFaMsg::disableSubSteps();
    FFaMsg::setSubTask("");
  }

  return true;
}


/*!
  Returns the UI-type name of the children of this ring start object.
*/

const char* FmRingStart::getChildrenUITypeName() const
{
  if (!myChildren.empty())
    return myChildren.front()->getUITypeName();
  else if (this->getNext() != const_cast<FmRingStart*>(this))
    return this->getNext()->getUITypeName();
  else
    return NULL;
}


void FmRingStart::addChild(FmRingStart* child)
{
  myChildren.push_back(child);
}


void FmRingStart::setParent(FmRingStart* parent)
{
  myParent = parent;
  myParent->addChild(this);
}


FmRingStart* FmRingStart::searchFuncHead(int funcUse) const
{
  for (FmRingStart* child : myChildren)
    if (FmRingStart* found = child->searchFuncHead(funcUse); found)
      return found;

  return NULL;
}
