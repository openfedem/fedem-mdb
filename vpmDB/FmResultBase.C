// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmResultBase.H"
#include "vpmDB/FmAnimation.H"
#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmGraph.H"


Fmd_DB_SOURCE_INIT(FcRESULT_BASE, FmResultBase, FmModelMemberBase);


FmResultBase::FmResultBase()
{
  Fmd_CONSTRUCTOR_INIT(FmResultBase);
}


bool FmResultBase::localParse(const char* keyWord, std::istream& activeStatement,
                              FmResultBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmResultBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmResultBase::getClassTypeID());
}


void FmResultBase::mainConnectedEvent()
{
  if (this->getUserDescription().empty())
    this->setUserDescription(std::string("New ") + this->getUITypeName());
}


/*!
  Returns a new result object of the same type as \a this.
  If \a cloneDepth > FmBase::NOTHING the field values are also copied.
*/

FmResultBase* FmResultBase::copy(int cloneDepth) const
{
  FmResultBase* newObj = NULL;
  if (this->isOfType(FmAnimation::getClassTypeID()))
    newObj = new FmAnimation();
  else if (this->isOfType(FmGraph::getClassTypeID()))
    newObj = new FmGraph();
  else if (this->isOfType(FmCurveSet::getClassTypeID()))
    newObj = new FmCurveSet();

  if (newObj && cloneDepth > FmBase::NOTHING) {
    newObj->clone(const_cast<FmResultBase*>(this),cloneDepth);
    newObj->setUserDescription("Copy of " + this->getInfoString());
  }

  return newObj;
}
