// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmStraightMaster.H"
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


bool FmStraightMaster::insertTriad(FmTriad* triad, size_t pos, bool updateViz)
{
  if (!triad) return false;

  if (!this->reOrientTriad(triad)) return false;

  return this->Fm1DMaster::insertTriad(triad,pos,updateViz);
}


bool FmStraightMaster::reOrientTriad(FmTriad* triad)
{
  FmTriad* first = this->getFirstTriad();
  if (!first) return true; // this is the first triad, do nothing

  FmTriad* last = this->getLastTriad();
  if (last && first != last)
  {
    // Check that the new triad is on the line between the two end triads
    FaVec3 triadPos = triad->getGlobalTranslation();
    FaVec3 firstPos = first->getGlobalTranslation();
    FaVec3 lineVec  = last->getGlobalTranslation() - firstPos;
    if (!lineVec.isParallell(triadPos - firstPos, FmDB::getParallelTolerance()))
    {
      ListUI <<" ==> ERROR: "<< triad->getIdString(true)
             <<" is not on the straight line through the end triads "
             << first->getIdString() <<" and "<< last->getIdString()
             <<".\n";
      return false;
    }
  }

  // If the new triad has important directions,
  // check that they fit with what we want to use
  if (!triad->importantDirections())
    triad->setOrientation(first->getOrientation(),true);
  else if (!triad->getOrientation().isCoincident(first->getOrientation(),
                                                 FmDB::getPositionTolerance()))
  {
    ListUI <<" ==> ERROR: "<< triad->getIdString(true)
           <<" does not have same orientation as "<< first->getIdString(true)
           <<".\n";
    return false;
  }

  return true;
}


/*!
  Add a triad at the given point \a globPoint.
*/

bool FmStraightMaster::addTriadOnPoint(const FaVec3& globPoint)
{
  // Check that this line is connected to one (and only one) part
  FmTriad* first = this->getFirstTriad();
  FmTriad* last  = this->getLastTriad();
  FmPart*  part1 = first ? first->getOwnerPart() : NULL;
  FmPart*  part2 = last  ? last->getOwnerPart() : NULL;
  if (!part1 || !part2)
  {
    ListUI <<" ==> ERROR: Can not add independent triad: ";
    if (first && !part1)
      ListUI << first->getIdString(true) <<" is not attached to a Part.\n";
    else if (last && !part2)
      ListUI << last->getIdString(true) <<" is not attached to a Part.\n";
    else
      ListUI <<"(NULL)\n"; // should not happen (logic error if so)
    return false;
  }

  // Check that the point is on the line between existing triads
  FaVec3 firstPos  = first->getGlobalTranslation();
  FaVec3 lineVec   = last->getGlobalTranslation() - firstPos;
  double parallTol = FmDB::getParallelTolerance();
  int parallelFlag = lineVec.isParallell(globPoint - firstPos, parallTol);
  if (parallelFlag == 0)
  {
    ListUI <<" ==> ERROR: Can not add independent triad:"
           <<" Point is not on the straight line.\n";
    return false;
  }

  // If attached to an FE part,
  // check that there is a valid FE node on that point
  double posTolerance = FmDB::getPositionTolerance();
  FaVec3 point;
  FmPart* part = NULL;
  if (FaVec3 p1 = part1->getGlobalCS().inverse() * globPoint;
      part1->isFEPart() && part1->getNodeAtPoint(p1,posTolerance))
  {
    point = p1;
    part = part1;
  }
  else if (part2 != part1)
  {
    if (FaVec3 p2 = part2->getGlobalCS().inverse() * globPoint;
        part2->isFEPart() && part2->getNodeAtPoint(p2,posTolerance))
    {
      point = p2;
      part = part2;
    }
    else
    {
      std::vector<FmPart*> parts;
      FmDB::getAllParts(parts);
      for (FmPart* pp : parts)
        if (pp != part1 && pp != part2 && pp->isFEPart())
          if (FaVec3 p3 = pp->getGlobalCS().inverse() * globPoint;
              pp->getNodeAtPoint(p3,posTolerance))
          {
            point = p3;
            part = pp;
            break;
          }
    }
  }

  if (!part)
  {
    ListUI <<" ==> ERROR: Can not add independent triad:"
           <<" Point is not on a valid FE-node.\n";
    return false;
  }

  FaMat33 Orient = first->getOrientation();
  if (part != part1)
    Orient = part->getOrientation().transpose()*part1->getOrientation()*Orient;

  // Check whether there already is a triad at that point, create one if not
  FmTriad* newTriad = part->getTriadAtPoint(point,posTolerance);
  if (!newTriad)
  {
    newTriad = new FmTriad(globPoint);
    newTriad->setParentAssembly(this->getParentAssembly());
    newTriad->connect(part);
    newTriad->setOrientation(Orient);
  }

  // If the triad found has important directions,
  // check that they fit with what we want to use
  else if (!newTriad->importantDirections())
    newTriad->setOrientation(Orient,true);
  else if (!newTriad->getOrientation().isCoincident(Orient,parallTol))
  {
    ListUI <<" ==> ERROR: Can not add independent triad: "
           << newTriad->getIdString(true)
           <<" does not have same orientation as "
           << first->getIdString(true) <<".\n";
    return false;
  }

  // Insert the triad in its proper location
  double myLength = lineVec.sqrLength();
  double distance = (globPoint-firstPos).sqrLength();
  bool ok = false;
  if (parallelFlag < 0)
    ok = this->addTriad(newTriad,true);
  else if (distance > myLength)
    ok = this->addTriad(newTriad,false);
  else for (size_t pos = 1; pos < this->size(); pos++)
    if (FaVec3 relPos = this->getTriad(pos)->getGlobalTranslation() - firstPos;
        relPos.sqrLength() > distance)
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
  if (FmTriad* first = this->getFirstTriad(); first)
    ur = FaMat34(first->getGlobalCS().direction(),
                 first->getGlobalCS().inverse()*pos);
  else
    ur = FaMat34(pos);

  return ur.translation().z();
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

  // Get global Z-axis of the master line
  FaVec3 Axis = M2->getGlobalTranslation() - M1->getGlobalTranslation();

  // Check that the first triad has proper orientation
  if (M1->getGlobalCS()[VZ].isParallell(Axis,FmDB::getParallelTolerance()) != 1)
  {
    if (FmPart* masterPart = M1->getOwnerPart(0); masterPart)
      Axis = Axis * masterPart->getGlobalCS().direction();

    FaMat33 newOrient;
    newOrient.makeGlobalizedCS(Axis).shift(-1);
    ListUI <<" ==> WARNING: Resetting orientation of "
           << M1->getIdString(true) <<".\n";
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
        std::sort(triads.begin(),triads.end(),isT1beforeT2);
        ListUI <<" ==> WARNING: Resetting triad ordering of "
               << this->getIdString() <<":";
        for (FmTriad* triad : triads)
          ListUI <<" "<< triad->getIdPath();
        ListUI <<"\n";
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
  FaVec3 oldPos;
  FaMat34 ur;

  for (FmTriad* triad : triads)
  {
    ur = triad->getGlobalCS();
    if (slideVarVal >= 0.0)
      slideVarVal += (ur.translation() - oldPos).length();
    else
      slideVarVal = 0.0;
    oldPos = ur.translation();

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
