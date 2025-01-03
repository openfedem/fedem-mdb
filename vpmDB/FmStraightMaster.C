// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmStraightMaster.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <algorithm>


Fmd_DB_SOURCE_INIT(FcMASTER_LINE, FmStraightMaster, Fm1DMaster);


FmStraightMaster::FmStraightMaster()
{
  Fmd_CONSTRUCTOR_INIT(FmStraightMaster);
}


bool FmStraightMaster::insertTriad(FmTriad* triad, size_t pos)
{
  if (!triad) return false;

  FmTriad* first = this->getFirstTriad();
  if (!first)
    return this->Fm1DMaster::insertTriad(triad,pos);

  FmTriad* last = this->getLastTriad();
  if (last && first != last)
  {
    // Check that the new triad is on the line between existing the triads
    FaVec3 triadPos = triad->getGlobalTranslation();
    FaVec3 firstPos = first->getGlobalTranslation();
    FaVec3 lineVec  = last->getGlobalTranslation() - firstPos;
    if (!lineVec.isParallell(triadPos - firstPos, FmDB::getParallelTolerance()))
    {
      ListUI <<"ERROR: Could not add "<< triad->getIdString(true)
             <<" as independent triad.\n      "
             <<" It is not on the straight line through existing triads.\n";
      return false;
    }
  }

  // If the triad has important directions,
  // check that they fit with what we want to use
  if (!triad->importantDirections())
    triad->setOrientation(first->getOrientation());
  else if (!triad->getOrientation().isCoincident(first->getOrientation(),
                                                 FmDB::getPositionTolerance()))
  {
    ListUI <<"ERROR: Could not add independent triad: "<< triad->getIdString(true)
           <<" does not have same orientation as "<< first->getIdString(true)
           <<".\n";
    return false;
  }

  return this->Fm1DMaster::insertTriad(triad,pos);
}


/*!
  Add a triad at the given point \a globPoint.
*/

bool FmStraightMaster::addTriadOnPoint(const FaVec3& globPoint)
{
  // Check that this line is connected to one (and only one) part
  FmTriad* first = this->getFirstTriad();
  FmPart*  part  = first ? first->getOwnerPart() : NULL;
  if (!part)
  {
    ListUI <<"ERROR: Could not add independent triad: ";
    if (first)
      ListUI << first->getIdString(true) <<" is not attached to a Part.\n";
    else
      ListUI <<" (NULL)\n"; // Should not happen (logic error if so)
    return false;
  }

  FmTriad* last = this->getLastTriad();
  if (!last || last->getOwnerPart() != part)
    return false; // Should never happen (topological inconsistency)

  // Check that the point is on the line between existing triads
  FaVec3 point     = part->getGlobalCS().inverse() * globPoint;
  FaVec3 firstPos  = first->getTranslation();
  FaVec3 lineVec   = last->getTranslation() - firstPos;
  double parallTol = FmDB::getParallelTolerance();
  int parallelFlag = lineVec.isParallell(point - firstPos, parallTol);
  if (parallelFlag == 0)
  {
    ListUI <<"ERROR: Could not add independent triad: Point is not on the straight line.\n";
    return false;
  }

  // If attached to an FE part, check that there is a valid FE node on that point
  double posTolerance = FmDB::getPositionTolerance();
  if (part->isFEPart() && !part->getNodeAtPoint(point,posTolerance))
  {
    ListUI <<"ERROR: Could not add independent triad: Point is not on a valid FE-node.\n";
    return false;
  }

  // Check whether there already is a triad at that point, create one if not
  FmTriad* newTriad = part->getTriadAtPoint(point,posTolerance);
  if (!newTriad)
  {
    newTriad = new FmTriad(globPoint);
    newTriad->setParentAssembly(this->getParentAssembly());
    newTriad->connect(part);
    newTriad->setOrientation(first->getOrientation());
  }

  // If the triad found has important directions,
  // check that they fit with what we want to use
  else if (!newTriad->importantDirections())
    newTriad->setOrientation(first->getOrientation());
  else if (!newTriad->getOrientation().isCoincident(first->getOrientation(),parallTol))
  {
    ListUI <<"ERROR: Could not add independent triad: "<< newTriad->getIdString(true)
           <<" does not have same orientation as "<< first->getIdString(true) <<".\n";
    return false;
  }

  // Insert the triad in its proper location
  double myLength = lineVec.sqrLength();
  double distance = (point-firstPos).sqrLength();
  bool ok = false;
  if (parallelFlag < 0)
    ok = this->addTriad(newTriad,true);
  else if (distance > myLength)
    ok = this->addTriad(newTriad,false);
  else for (size_t pos = 1; pos < this->size(); pos++)
    if ((this->getTriad(pos)->getTranslation()-firstPos).sqrLength() > distance)
    {
      ok = this->Fm1DMaster::insertTriad(newTriad,pos);
      break;
    }

  if (ok)
  {
    newTriad->draw();
    newTriad->updateChildrenDisplayTopology();
  }

  return ok;
}


double FmStraightMaster::getSliderPosition(FaMat34& ur, const FaVec3& pos) const
{
  FmTriad* first = this->getFirstTriad();
  if (first)
    ur = FaMat34(first->getGlobalCS().direction(),first->getGlobalCS().inverse()*pos);
  else
    ur = FaMat34(pos);

  return ur[3][2];
}


std::ostream& FmStraightMaster::writeFMF(std::ostream& os)
{
  os <<"MASTER_LINE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmStraightMaster::readAndConnect(std::istream& is, std::ostream&)
{
  FmStraightMaster* obj = new FmStraightMaster();

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


void FmStraightMaster::initAfterResolve()
{
  this->Fm1DMaster::initAfterResolve();

  // Check that the point is on the line between existing triads
  FmTriad* M1 = this->getFirstTriad();
  FmTriad* M2 = this->getLastTriad();
  if (!M1 || !M2) return;

  // Get global positions of the triads
  FaVec3 P1 = M1->getTranslation();
  FaVec3 P2 = M2->getTranslation();

  // Check that the first triad has proper orientation
  if (M1->getLocalCS()[2].isParallell(P2-P1,FmDB::getParallelTolerance()) != 1)
  {
    FaMat33 newOrient;
    newOrient.makeGlobalizedCS(P2-P1).shift(-1);
    ListUI <<" ==> WARNING: Resetting orientation of "<< M1->getIdString(true) <<".\n";
    M1->setOrientation(newOrient);
  }

  if (this->size() > 2)
  {
    // Lambda function defining the triad order based on orientations.
    auto&& isT1beforeT2 = [](const FmTriad* T1, const FmTriad* T2)
    {
      FaMat34 T1mx = T1->getGlobalCS();
      FaMat34 T2mx = T2->getGlobalCS();
      // T1 is in front of T2 if the vector T1-T2
      // is in the same direction as the Z-axis of T1
      return (T2mx[VW] - T1mx[VW]) * T1mx[VZ] > 0.0;
    };

    std::vector<FmTriad*> triads;
    this->getTriads(triads);

    for (size_t i = 1; i+1 < triads.size(); i++)
      if (isT1beforeT2(triads[i+1],triads[i]))
      {
        ListUI <<" ==> WARNING: Resetting the ordering of "<< this->getIdString(true) <<"\n.";
        std::sort(triads.begin(),triads.end(),isT1beforeT2);
        this->setTriads(triads);
	break;
      }
  }
}


bool FmStraightMaster::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmStraightMaster::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmStraightMaster::getClassTypeID());
}


int FmStraightMaster::printSolverEntry(FILE* fp)
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  this->printForSolver(fp,triads);

  double slideVarVal = -1.0;
  FaVec3 thisPos, oldPos;
  FaMat34 ur;

  for (FmTriad* triad : triads)
  {
    ur      = triad->getGlobalCS();
    thisPos = ur.translation();
    if (slideVarVal >= 0.0)
      slideVarVal += (thisPos - oldPos).length();
    else
      slideVarVal = 0.0;
    oldPos  = thisPos;

    fprintf(fp,"&MASTER_POS\n");
    fprintf(fp,"  masterId = %d\n", this->getBaseID());
    fprintf(fp,"  triadId  = %d\n", triad->getBaseID());
    fprintf(fp,"  PosInGlobal =%17.9e %17.9e %17.9e %17.9e\n",
            ur[0][0],ur[1][0],ur[2][0],ur[3][0]);
    fprintf(fp,"               %17.9e %17.9e %17.9e %17.9e\n",
            ur[0][1],ur[1][1],ur[2][1],ur[3][1]);
    fprintf(fp,"               %17.9e %17.9e %17.9e %17.9e\n",
            ur[0][2],ur[1][2],ur[2][2],ur[3][2]);
    fprintf(fp,"  slideVarVal =%17.9e\n", slideVarVal);
    fprintf(fp,"  upVec       =%17.9e %17.9e %17.9e\n/\n",
            ur[0][0],ur[0][1],ur[0][2]);
  }

  fprintf(fp,"\n");
  return 0;
}
