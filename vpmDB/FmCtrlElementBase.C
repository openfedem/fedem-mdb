// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmCtrlElementBase.H"
#include "vpmDB/FmCtrlLine.H"

extern const char* blank[];

Fmd_DB_SOURCE_INIT(FccELEMENT_BASE, FmCtrlElementBase, FmIsRenderedBase);


FmCtrlElementBase::FmCtrlElementBase()
{
  Fmd_CONSTRUCTOR_INIT(FmCtrlElementBase);

  FFA_FIELD_INIT(myLeftRotatedFlag, false, "LEFT_ORIENTATED");
  FFA_FIELD_DEFAULT_INIT(myPosition, "POSITION");

  itsPixmap = blank;
}


FmCtrlElementBase::~FmCtrlElementBase()
{
  this->disconnect();
}


void FmCtrlElementBase::changedEvent()
{
  this->updateDisplayDetails();
}


bool FmCtrlElementBase::localParse(const char* keyWord, std::istream& activeStatement,
                                   FmCtrlElementBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmCtrlElementBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmCtrlElementBase::getClassTypeID());
}


/*!
  If \a single is \e true, the two nearest line segments
  are updated and the others are not changed.
  This is used when only one element is moved.
  If \a single is \e false, then all line segments in the line are updated,
  this is used when a group of elements are moved.
*/

void FmCtrlElementBase::updateLines(bool single)
{
  FmCtrlLine* line = NULL;
  for (int inpPort = 1; inpPort <= this->getNumInputPorts(); inpPort++)
    if ((line = this->getLine(inpPort)))
    {
      if (single)
      {
        DoubleVec lengths = line->getTotLengthArray();
        int numSegments = line->getNumberOfSegments();
        line->setLengthArray(DoubleVec(lengths.begin(),lengths.begin()+numSegments-2));
        line->setFirstUndefSegment(numSegments-1);
      }
      line->draw();
    }
}
