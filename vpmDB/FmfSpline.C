// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmfSpline.H"
#include "vpmDB/FuncPixmaps/spline.xpm"

#define BLOCK_SIZE 2


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcfSPLINE, FmfSpline, FmfMultiVarBase);

FmfSpline::FmfSpline()
{
  Fmd_CONSTRUCTOR_INIT(FmfSpline);

  itsSplineICODE = 0;
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

const char** FmfSpline::getPixmap() const
{
  return spline;
}


int FmfSpline::getBlockSize() const
{
  return BLOCK_SIZE;
}


std::ostream& FmfSpline::writeFMF(std::ostream& os)
{
  os <<"FUNC_SPLINE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


void FmfSpline::addXYset(double X, double Y)
{
  long int pos = this->binarySearch(X, BLOCK_SIZE);
  this->insertVal(X, pos);
  this->insertVal(Y, pos+1);
}


void FmfSpline::setSplineICODE(int code)
{
  itsSplineICODE = code;
}


bool FmfSpline::removeXYset(long int place)
{
  this->removeVal(place*BLOCK_SIZE + 1);
  this->removeVal(place*BLOCK_SIZE);
  return true;
}


void FmfSpline::setAllSplineICODE(bool)
{
  std::vector<FmfSpline*> allSplines;
  FmDB::getAllSplines(allSplines);
  if (allSplines.empty()) return;

  // build and sort the array of splines
  std::vector<splineTable> spln;
  spln.reserve(allSplines.size());
  for (FmfSpline* f : allSplines)
    spln.push_back(splineTable(f,f->numValues()/2));

  std::sort(spln.begin(), spln.end(),
            [](const splineTable& lhs, const splineTable& rhs)
            { return lhs.size < rhs.size; });

  std::vector<splineTable> smallSplines, largeSplines;

  for (const splineTable& s : spln)
    if (s.size <= 40)
      smallSplines.push_back(s);
    else
      largeSplines.push_back(s);

  for (size_t i = 0; i < largeSplines.size(); i++)
    largeSplines[i].ICODE = i < 6 ? 31+i : 36;

  // fill up the small splines
  for (size_t i = 0; i < smallSplines.size(); i++)
    if (i < 2)
      smallSplines[i].ICODE = i+6;
    else if (largeSplines.size()+(i-2) < 6)
      smallSplines[i].ICODE = (i-2)+31+largeSplines.size();
    else
      smallSplines[i].ICODE = 7;

  // set ICODE back in splines
  for (splineTable& s : smallSplines)
    s.object->setSplineICODE(s.ICODE);
  for (splineTable& s : largeSplines)
    s.object->setSplineICODE(s.ICODE);
}


bool FmfSpline::readAndConnect(std::istream& is, std::ostream&)
{
  FmfSpline* obj = new FmfSpline();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      FmfMultiVarBase::localParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmfSpline::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmfSpline::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmfSpline::getClassTypeID());
}


int FmfSpline::checkSplines()
{
  int errCount = 0;
  std::vector<FmfSpline*> allSplines;
  FmDB::getAllSplines(allSplines);
  for (FmfSpline* f : allSplines)
    if (f->numValues() < 8)
    {
      errCount++;
      ListUI <<"ERROR: Too few control points in "<< f->getIdString() <<".\n";
    }

  return errCount;
}


void FmfSpline::getXAxisDomain(double& start, double& stop) const
{
  if (this->getExtrapolationType() > FmfMultiVarBase::NONE) return;

  const DoubleVec& itsVals = this->myValues.getValue();
  if (itsVals.empty()) return;

  double min = itsVals[0];
  double max = min;
  for (size_t i = 2; i < itsVals.size(); i += 2)
    if (itsVals[i] < min)
      min = itsVals[i];
    else if (itsVals[i] > max)
      max = itsVals[i];

  if (start < min) start = min;
  if (stop  > max) stop  = max;
}
