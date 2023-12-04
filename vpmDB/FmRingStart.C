// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_SOURCE_INIT(FcRING_START, FmRingStart, FmModelMemberBase);


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


bool FmRingStart::hasRingMembers() const
{
  if (myChildren.empty())
    return (this->getNext() != (FmBase*)this) ? true : false;

  for (size_t i = 0; i < myChildren.size(); i++)
    if (myChildren[i]->hasRingMembers())
      return true;

  return false;
}


int FmRingStart::countRingMembers() const
{
  int count = 0;
  for (FmBase* p = this->getNext(); p != (FmBase*)this; p = p->getNext())
    count++;

  return count;
}


void FmRingStart::displayRingMembers() const
{
  if (myRingMemberType == FmPart::getClassTypeID())
  {
    int partNumber = 0;
    FFaMsg::enableSubSteps(this->countRingMembers());
    for (FmBase* p = this->getNext(); p != (FmBase*)this; p = p->getNext())
    {
      FFaMsg::setSubStep(++partNumber);
      FFaMsg::setSubTask(static_cast<FmPart*>(p)->getBaseFTLName());
      p->drawObject();
    }
    FFaMsg::disableSubSteps();
    FFaMsg::setSubTask("");
  }
  else
    for (FmBase* p = this->getNext(); p != (FmBase*)this; p = p->getNext())
      p->drawObject();
}


bool FmRingStart::eraseRingMembers(bool showProgress)
{
  int count = this->countRingMembers();
  if (count == 0) return false;

  if (showProgress)
  {
    FFaMsg::enableSubSteps(count);
    FFaMsg::setSubTask(this->getUITypeName());
    count = 0;
  }

  while (this->getNext() != (FmBase*)this)
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
  else if (this->getNext() != (FmBase*)this)
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
  FmRingStart* found = NULL;
  for (size_t i = 0; i < myChildren.size() && !found; i++)
    found = myChildren[i]->searchFuncHead(funcUse);

  return found;
}
