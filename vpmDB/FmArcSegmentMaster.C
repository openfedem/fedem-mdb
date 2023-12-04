// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmArcSegmentMaster.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaAlgebra/FFa3PArc.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include <algorithm>


static double acosClip(double cosVal)
{
  if (cosVal >= 1.0)
    return 0.0;
  else if (cosVal <= -1.0)
    return M_PI;
  else
    return acos(cosVal);
}


Fmd_DB_SOURCE_INIT(FcMASTER_ARC_SEGMENT, FmArcSegmentMaster, Fm1DMaster);


FmArcSegmentMaster::FmArcSegmentMaster() : Fm1DMaster()
{
  Fmd_CONSTRUCTOR_INIT(FmArcSegmentMaster);

  FFA_FIELD_INIT(myLoopFlag, false, "LOOPING");
}


/*!
  Add a triad at the given point \a globPoint.
*/

bool FmArcSegmentMaster::addTriadOnPoint(const FaVec3& globPoint)
{
  // Check that this arc segment is connected to one (and only one) part
  FmPart* part = dynamic_cast<FmPart*>(this->getOwnerLink());
  if (!part)
  {
    ListUI <<"ERROR: Could not add triad: Arc segment object is not attached.\n";
    return false;
  }

  // Convert the global point to a local point
  FaVec3 point = part->getLocalCS().inverse() * globPoint;

  // Check that there is a valid FE node on that point
  double posTolerance = FmDB::getPositionTolerance();
  if (!part->getNodeAtPoint(point,posTolerance))
  {
    ListUI <<"ERROR: Could not add triad: Point is not on a valid FE-node.\n";
    return false;
  }

  // Check whether there already is a triad at that point, create one if not
  FmTriad* newTriad = part->getTriadAtPoint(point,posTolerance);
  if (!newTriad)
  {
    newTriad = new FmTriad();
    newTriad->setParentAssembly(this->getParentAssembly());
    newTriad->connect(part);
  }

  if (!this->Fm1DMaster::addTriad(newTriad))
    return false;

  newTriad->setTranslation(point);
  newTriad->draw();
  newTriad->updateChildrenDisplayTopology();
  return true;
}


bool FmArcSegmentMaster::isAddable(FmTriad* triad) const
{
  FmTriad* first = this->getFirstTriad();
  if (!first)
    return true; // This is the first triad to be added
  else if (!first->isAttached() && !triad->isAttached())
    return true; // The triads are not attached yet

  FmPart* ownerPart = triad->getOwnerPart(0);
  if (ownerPart)
  {
    // The first triad is attached to a (FE or generic) part
    if (first->isAttached(ownerPart))
      return true;

    ListUI <<"ERROR: All triads must be on the same part.\n";
    return false;
  }

  FmTriad* second = this->getTriad(1);

  std::vector<FmBeam*> beams;
  first->getBeamBinding(beams);
  for (FmBeam* beam : beams)
  {
    // The first triad is along a beamstring.
    // Check if the triad to be added is on the same one.
    std::vector<FmIsPlottedBase*> bs;
    beam->traverse(first,bs);
    if (std::find(bs.begin(),bs.end(),triad) == bs.end())
    {
      // The triad to be added is not on the same (part of the) beamstring
      if (beams.size() == 1)
      {
        ListUI <<"ERROR: All triads must be on the same beamstring.\n";
        return false;
      }
      if (std::find(bs.begin(),bs.end(),second) == bs.end())
        continue; // Not this part, try the next one
    }
    else if (!second || std::find(bs.begin(),bs.end(),second) != bs.end())
      return true; // The triad to be added is on the right beamstring

    ListUI <<"ERROR: All triads must be on the same part of the beamstring.\n";
    return false;
  }

  ListUI <<"ERROR: The triads can be attached to only parts and beams,\n"
         <<"       and all (or none) must be attached during modeling.\n";
  return false;
}


bool FmArcSegmentMaster::addTriad(FmTriad* triad, bool asFront)
{
  if (!triad) return false;

  if (this->isLooping())
  {
    ListUI <<"ERROR: Arc segment is closed. Can not add more triads.\n";
    return false;
  }

  if (!this->isAddable(triad))
    return false;

  else if (!this->hasTriad(triad))
    return this->Fm1DMaster::addTriad(triad,asFront);

  else if (triad == this->getFirstTriad())
  {
    ListUI <<"NOTE: Closing the arc segment loop.\n";
    myLoopFlag.setValue(true);
    return true;
  }
  else
    ListUI <<"ERROR: "<< triad->getIdString()
           <<" is already on this arc segment.\n";

  return false;
}


int FmArcSegmentMaster::releaseTriad(FmTriad* triad, FmTriad* replacement)
{
  int indexOfRemoved = this->Fm1DMaster::releaseTriad(triad,replacement);
  if (indexOfRemoved < 1)
    return false;
  else if (!this->isLooping() || replacement)
    return true;

  // A triad in a looping arc segment was released ==> it is no longer looping
  myLoopFlag.setValue(false);
  // Unless it was the first or last triad, we need to reshuffle the triad list
  this->reshuffleTriads(indexOfRemoved-1);
  return true;
}


void FmArcSegmentMaster::releaseTriads(bool eraseUnused)
{
  myLoopFlag.setValue(false);
  this->Fm1DMaster::releaseTriads(eraseUnused);
}


std::ostream& FmArcSegmentMaster::writeFMF(std::ostream& os)
{
  os <<"MASTER_ARC_SEGMENT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmArcSegmentMaster::readAndConnect(std::istream& is, std::ostream&)
{
  FmArcSegmentMaster* obj = new FmArcSegmentMaster();

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


bool FmArcSegmentMaster::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmArcSegmentMaster::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmArcSegmentMaster::getClassTypeID());
}


void FmArcSegmentMaster::initAfterResolve()
{
  FmTriad* tr1 = this->getFirstTriad();
  FmTriad* tr2 = this->getLastTriad();
  Fm1DMaster::initAfterResolve();
  FmTriad* tr3 = this->getLastTriad();
  if (tr1 == tr2 && tr3 != tr2)
  {
    this->setLooping();
    ListUI <<"     Setting looping flag ON instead.\n";
  }
}


int FmArcSegmentMaster::printSolverEntry(FILE* fp)
{
  std::vector<FmTriad*>     triads;
  std::vector<CurveSection> curveSections;
  this->getTriads(triads);
  this->getCurveSections(curveSections);
  if (this->isLooping())
    this->printForSolver(fp,triads,curveSections.back().slideValue[1]);
  else
    this->printForSolver(fp,triads);

  size_t nTriads = triads.size();
  if (nTriads < 2 || nTriads%2 == (this->isLooping() ? 1 : 0))
  {
    FmMMJointBase* joint;
    FFaMsg::list("\n---> ERROR: ",true);
    if (this->hasReferringObjs(joint,"myMaster"))
      ListUI << joint->getIdString(true);
    else
      ListUI << this->getIdString(true);
    if (nTriads < 2)
      ListUI <<" has zero length.\n";
    else if (this->isLooping())
      ListUI <<" has an odd number of independent triads.\n";
    else
      ListUI <<" has an even number of independent triads.\n";
    return 1;
  }

  double curv, slideVarVal;
  FaVec3 upVec;
  FaMat34 ur;

  for (size_t i = 0; i < nTriads; i++)
  {
    fprintf(fp,"&MASTER_POS\n");
    fprintf(fp,"  masterId = %d\n", this->getBaseID());
    fprintf(fp,"  triadId  = %d\n", triads[i]->getBaseID());

    if (!this->isLooping() && i == nTriads-1)
    {
      ur          = curveSections[i-1].ur[1];
      curv        = curveSections[i-1].curvature;
      slideVarVal = curveSections[i-1].slideValue[1];
      upVec       = curveSections[i-1].radVec;
    }
    else
    {
      ur          = curveSections[i].ur[0];
      curv        = curveSections[i].curvature;
      slideVarVal = curveSections[i].slideValue[0];
      upVec       = curveSections[i].radVec;
    }

    fprintf(fp,"  PosInGlobal =%17.9e %17.9e %17.9e %17.9e\n",
            ur[0][0],ur[1][0],ur[2][0],ur[3][0]);
    fprintf(fp,"               %17.9e %17.9e %17.9e %17.9e\n",
            ur[0][1],ur[1][1],ur[2][1],ur[3][1]);
    fprintf(fp,"               %17.9e %17.9e %17.9e %17.9e\n",
            ur[0][2],ur[1][2],ur[2][2],ur[3][2]);

    fprintf(fp,"  curvature   =%17.9e\n", curv);
    fprintf(fp,"  slideVarVal =%17.9e\n", slideVarVal);
    fprintf(fp,"  upVec       =%17.9e %17.9e %17.9e\n",
            upVec[0], upVec[1], upVec[2]);

    fprintf(fp,"/\n");
  }

  fprintf(fp,"\n");
  return 0;
}


void FmArcSegmentMaster::getCurveSegments(std::vector<CurveSegment>& seg) const
{
  // Count the triads, add 1 if it is looping
  size_t cpCount = this->size();
  if (this->isLooping()) cpCount++;

  // Calculate the segment count
  size_t segCount = cpCount > 1 ? ldiv(cpCount-1,2).quot : 0;
  seg.resize(segCount);
  if (segCount == 0) return;

  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  // Find the three points defining each arc segment
  size_t i, j, k;
  for (i = k = 0; i < segCount; i++, k += 2)
  {
    for (j = 0; j < 3 && k+j < triads.size(); j++)
      seg[i][j] = triads[k+j]->getLocalTranslation();
    seg[i][3] = triads[k]->getLocalCS()[VX]; // Up-vector for this segment
  }
  if (this->isLooping())
    seg.back()[2] = triads.front()->getLocalTranslation();
}


void FmArcSegmentMaster::getCurveSections(std::vector<CurveSection>& sec) const
{
  // Count the triads, subtract 1 unless it is looping
  size_t secCount = this->size();
  if (secCount > 0 && !this->isLooping()) secCount--;

  sec.resize(secCount);
  if (secCount == 0) return;

  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  FaMat34 ur;
  size_t i, k;
  for (i = k = 0; i < secCount-1; i++)
  {
    ur = triads[k++]->getGlobalCS();
    sec[i].ur[0][3] = ur[3]; // Position
    sec[i].ur[0][1] = ur[1]; // Y-axis
    ur = triads[k]->getGlobalCS();
    sec[i].ur[1][3] = ur[3]; // Position
    sec[i].ur[1][1] = ur[1]; // Y-axis
    // Default radius direction vector, sections with curvature gets new value
    sec[i].radVec = (ur[1]^(sec[i].ur[1][3]-sec[i].ur[0][3])).normalize();
  }

  // Define the last section depending on the loop variable
  ur = triads[k++]->getGlobalCS();
  sec[i].ur[0][3] = ur[3]; // Position
  sec[i].ur[0][1] = ur[1]; // Y-axis

  if (this->isLooping())
    ur = triads.front()->getGlobalCS();
  else
    ur = triads[k]->getGlobalCS();

  sec[i].ur[1][3] = ur[3]; // Position
  sec[i].ur[1][1] = ur[1]; // Y-axis
  sec[i].radVec = (ur[1]^(sec[i].ur[1][3]-sec[i].ur[0][3])).normalize();

  // Compute the curvature, and remaining part of the section type

  double slideValue = 0.0;
  const double tiny = 1.0e-15;
  for (i = 0; i+1 < secCount; i+=2)
  {
    FaVec3 s12 = sec[i  ].ur[1][3] - sec[i  ].ur[0][3];
    FaVec3 s23 = sec[i+1].ur[1][3] - sec[i+1].ur[0][3];
    FaVec3 s13 = sec[i+1].ur[1][3] - sec[i  ].ur[0][3];
    double l12 = s12.length(); s12.normalize();
    double l23 = s23.length(); s23.normalize();
    double l13 = s13.length(); s13.normalize();

    double gamma = acosClip(s12*s13) + acosClip(s23*s13);
    double curvature = l13 > tiny ? 2.0*sin(gamma)/l13 : 0.0;

    sec[i].slideValue[0] = slideValue;

    double relArcHeight = 0.125*curvature*l13;
    if (fabs(relArcHeight) > 0.0001) // 0.1mm on 1.0m
    {
      sec[i].curvature = sec[i+1].curvature = curvature;

      // Slide values for arc
      double R = 1.0/curvature;
      double alpha1 = 2.0*acosClip(s23*s13);
      double alpha2 = 2.0*acosClip(s12*s13);
      slideValue += alpha1/curvature;
      sec[i  ].slideValue[1] = slideValue;
      sec[i+1].slideValue[0] = slideValue;
      slideValue += alpha2/curvature;
      sec[i+1].slideValue[1] = slideValue;

      // Tangent vectors for arc
      FaVec3 nVec  = (s13^s12).normalize();
      FaVec3 rVec  = (s13^nVec).normalize();
      FaVec3 Origo = (sec[i].ur[0][3]+sec[i+1].ur[1][3])*0.5 + rVec*cos(gamma)*R;

      // Vector in positive radius direction
      sec[i  ].radVec = -rVec;
      sec[i+1].radVec = -rVec;

      // Z-dir, also positive tangent direction
      rVec = sec[i].ur[0][3] - Origo;
      sec[i  ].ur[0][2] = (rVec^nVec).normalize(); // Z-dir, tangent dir

      rVec = sec[i].ur[1][3] - Origo;
      sec[i  ].ur[1][2] = (rVec^nVec).normalize(); // Z-dir, tangent dir
      sec[i+1].ur[0][2] = sec[i].ur[1][2];

      rVec = sec[i+1].ur[1][3] - Origo;
      sec[i+1].ur[1][2] = (rVec^nVec).normalize();
    }
    else // curvature is negligible, ie straight line
    {
      // Slide values for straight line
      slideValue += l12;
      sec[i  ].slideValue[1] = slideValue;
      sec[i+1].slideValue[0] = slideValue;
      slideValue += l23;
      sec[i+1].slideValue[1] = slideValue;

      // Z-dir, also positive tangent direction
      sec[i  ].ur[0][2] = s13; // Z-dir, tangent dir
      sec[i  ].ur[1][2] = s13; // Z-dir, tangent dir
      sec[i+1].ur[0][2] = s13; // Z-dir, tangent dir
      sec[i+1].ur[1][2] = s13; // Z-dir, tangent dir
    }

    // X-axis vector
    sec[i  ].ur[0][0] = (sec[i  ].ur[0][1]^sec[i  ].ur[0][2]).normalize();
    sec[i  ].ur[1][0] = (sec[i  ].ur[1][1]^sec[i  ].ur[1][2]).normalize();
    sec[i+1].ur[0][0] = (sec[i+1].ur[0][1]^sec[i+1].ur[0][2]).normalize();
    sec[i+1].ur[1][0] = (sec[i+1].ur[1][1]^sec[i+1].ur[1][2]).normalize();

    // Y-axis vector
    sec[i  ].ur[0][1] = (sec[i  ].ur[0][2]^sec[i  ].ur[0][0]).normalize();
    sec[i  ].ur[1][1] = (sec[i  ].ur[1][2]^sec[i  ].ur[1][0]).normalize();
    sec[i+1].ur[0][1] = (sec[i+1].ur[0][2]^sec[i+1].ur[0][0]).normalize();
    sec[i+1].ur[1][1] = (sec[i+1].ur[1][2]^sec[i+1].ur[1][0]).normalize();
  }
}


double FmArcSegmentMaster::getSliderPosition(FaMat34& ur,
                                             const FaVec3& pos) const
{
  std::vector<CurveSection> cSections;
  this->getCurveSections(cSections);

  // Find which curve section the slider is at
  std::vector<CurveSection>::const_iterator cit;
  cit = std::find_if(cSections.begin(), cSections.end(),
                     [pos](const CurveSection& cs)
                     {
                       FaVec3 r1 = pos - cs.ur[0][3];
                       FaVec3 r2 = pos - cs.ur[1][3];
                       return (cs.ur[0][2]*r1 >= 0.0 &&
                               cs.ur[1][2]*r2 <  0.0);
                     });
  if (cit == cSections.end()) return -999.999;

  // Find slider coordinate value
  const CurveSection& cs = *cit;
  FaVec3 sec  = cs.ur[1][3] - cs.ur[0][3];
  double Lsec = sec.length(); sec.normalize();
  double frac;

  if (cs.curvature > 0.0)
  {
    double R = 1.0/cs.curvature;
    //bh double alpha = asin(0.5*Lsec/R);
    double alpha = acosClip(cs.ur[0][2]*cs.ur[1][2])*0.5;
    FaVec3 rVec = (cs.ur[1][2]-cs.ur[0][2]).normalize();
    FaVec3 Origo= (cs.ur[0][3]+cs.ur[1][3])*0.5 + R*cos(alpha)*rVec;
    FaVec3 r1 = (cs.ur[0][3] - Origo).normalize();
    FaVec3 r2 = (pos - Origo).normalize();
    frac  = acosClip(r1*r2)/(2.0*alpha);
    ur[3] = Origo + r2*R; //TODO: Update to handle offset in Y-direction
    if ( (frac <  0.5 && r2*cs.ur[0][0] < 0.0) ||
         (frac >= 0.5 && r2*cs.ur[1][0] < 0.0) ) r2 = -r2;
    ur[0] = r2;
    ur[1] = (sec^r2).normalize();
    ur[2] = ur[0]^ur[1];
  }
  else
  {
    FaVec3 r1 = pos - cs.ur[0][3];
    frac  = (r1*sec)/Lsec;
    ur[3] = (1.0-frac)*cs.ur[0][3] + frac*cs.ur[1][3];
    ur[2] = sec;
    ur[1] = (sec^cs.radVec).normalize();
    ur[0] = ur[1]^ur[2];
  }

  return (1.0-frac)*cs.slideValue[0] + frac*cs.slideValue[1];
}


/*!
  Returns the average of two matrices assuming they share Ey and Position.
*/

static FaMat34 average(const FaMat34& m1, const FaMat34& m2)
{
  FaVec3 Ez = m1[2] + m2[2];
  FaVec3 Ey = m1[1];

  if (Ez.length() < 1.0e-9)
    Ez = m1[2] ^ Ey;

  Ez.normalize();
  FaVec3 Ex = Ey ^ Ez;

  return FaMat34(Ex, Ey, Ez, m1[3]);
}


void FmArcSegmentMaster::setDefaultOrientation(FmTriad* follower)
{
  std::vector<CurveSegment> curveSegs;
  this->getCurveSegments(curveSegs);
  if (curveSegs.empty()) return;

  std::vector<FFa3PArc> arcs;
  std::vector<FFa3PArc>::const_iterator iarc;
  arcs.reserve(curveSegs.size());
  for (const CurveSegment& seg : curveSegs)
    arcs.push_back(FFa3PArc(seg[0],seg[1],seg[2]));

  // Position of the follower relative to the link this triad is attached to.
  // If not attached yet, it will be equal to the global position.
  FaVec3 followerPos = follower->getLocalTranslation(this->getOwnerLink());

  // Find a valid normal, also when dealing with straight lines
  FaVec3 positiveNormal;

  // First, try to find a good arc
  for (iarc = arcs.begin(); iarc != arcs.end(); ++iarc)
    if (iarc->isArc(1.0e-7))
    {
      positiveNormal = iarc->getNormal();
      break;
    }

  if (iarc == arcs.end())
  {
    // No arcs found, all segments are straight lines.
    // Find first line not parallel to the first one.

    FaVec3 line;
    FaVec3 tan1 = arcs.front().P[2] - arcs.front().P[0];
    if (arcs.size() > 1)
    {
      iarc = arcs.begin();
      for (++iarc; iarc != arcs.end(); ++iarc)
      {
        line = iarc->P[2] - iarc->P[0];
        if (!tan1.isParallell(line))
        {
          positiveNormal = tan1 ^ line;
          break;
        }
      }
    }

    if (iarc == arcs.end())
    {
      // All lines are parallel
      line = followerPos - this->getFirstTriad()->getLocalTranslation();
      if (!line.isParallell(tan1))
        positiveNormal = tan1 ^ line;
      else if (!tan1.isParallell(FaVec3(0.0,0.0,1.0)))
        positiveNormal = FaVec3(tan1[VY],-tan1[VX],0.0); // tan1 x Z-axis
      else
        positiveNormal.x(1.0);
    }
  }

  // Find triad closest to follower

  std::vector<FmTriad*> triads;
  this->getTriads(triads);

  FaVec3 upVec = followerPos - triads.front()->getLocalTranslation();
  double dist = upVec.length();
  size_t i, closestTriadIdx = 0;
  for (i = 1; i < triads.size(); i++)
  {
    FaVec3 up = followerPos - triads[i]->getLocalTranslation();
    if (up.length() < dist)
    {
      upVec = up;
      dist = upVec.length();
      closestTriadIdx = i;
    }
  }

  // Find a Z-direction (along the curve), and
  // adjust the plane normal to make the X-direction point towards the follower

  FaVec3 alongVec = triads[closestTriadIdx]->getLocalTranslation();
  if (closestTriadIdx+1 < triads.size())
    alongVec = triads[closestTriadIdx+1]->getLocalTranslation() - alongVec;
  else if (this->isLooping())
    alongVec = triads.front()->getLocalTranslation() - alongVec;
  else if (closestTriadIdx > 0)
    alongVec = alongVec - triads[closestTriadIdx-1]->getLocalTranslation();

  if (positiveNormal * (upVec^alongVec) <= 0.0)
    positiveNormal *= -1.0;

  // Set triad orientations taking looping into account

  i = 0;
  for (FmTriad* triad : triads)
  {
    if (i++ == 2*arcs.size()) break;

    // Check if the orientation of this triad is allowed to change
    if (triad->getSimpleSensor()) continue;
    if (triad->hasAddMass()) continue;
    if (triad->itsLocalDir.getValue() > FmTriad::GLOBAL)
    {
      if (triad->hasConstraints()) continue;
      if (triad->hasInitVel()) continue;
      if (triad->hasInitAcc()) continue;
    }
    std::vector<FmJointBase*> joints;
    triad->getJointBinding(joints);
    if (joints.size() > 1) continue;

    if (i == 1)
    {
      if (this->isLooping()) // Take looping into account for the first triad
        triad->setLocalCS(average(arcs.back().getCtrlPointMatrix(2,positiveNormal),
                                  arcs.front().getCtrlPointMatrix(0,positiveNormal)));
      else
        triad->setLocalCS(arcs.front().getCtrlPointMatrix(0,positiveNormal));
    }
    else if (i%2 == 0) // Triad at a mid-point of an arc
      triad->setLocalCS(arcs[i/2-1].getCtrlPointMatrix(1,positiveNormal));
    else if (i/2 < arcs.size()) // Third triad of not the last arc
      triad->setLocalCS(average(arcs[i/2-1].getCtrlPointMatrix(2,positiveNormal),
                                arcs[i/2].getCtrlPointMatrix(0,positiveNormal)));
    else // Third triad of the last arc, i.e., the end triad
      triad->setLocalCS(arcs.back().getCtrlPointMatrix(2,positiveNormal));

    triad->draw(); // Update the visualization making the new directions show up
  }
}
