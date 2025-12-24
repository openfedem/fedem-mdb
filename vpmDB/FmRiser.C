// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRiser.H"
#include "vpmDB/FmSoilPile.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include <algorithm>
#include <cstdio>


Fmd_SOURCE_INIT(FcRISER, FmRiser, FmSubAssembly);


FmRiser::FmRiser(bool isDummy) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(internalMud, true, "MUD_FILLED");
  FFA_FIELD_INIT(mudDensity, 0.0, "MUD_DENSITY");
  FFA_FIELD_INIT(mudLevel, 0.0, "MUD_LEVEL");

  FFA_FIELD_INIT(visualize3Dts, 1, "VISUALIZE3D");
  FFA_FIELD_INIT(visualize3DAngles, Ints(0,360), "VISUALIZE3D_ANGLES");
}


const char** FmRiser::getListViewPixmap() const
{
  return beamstring_xpm;
}


std::ostream& FmRiser::writeFMF(std::ostream& os)
{
  os <<"RISER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmRiser::readAndConnect(std::istream& is, std::ostream&)
{
  FmRiser* obj = new FmRiser();

  // Obsolete fields
  FFaObsoleteField<int> startAngle, stopAngle;
  FFA_OBSOLETE_FIELD_INIT(startAngle,0,"VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_INIT(stopAngle,360,"VISUALIZE3D_STOP_ANGLE",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_START_ANGLE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("VISUALIZE3D_STOP_ANGLE",obj);

  // Update from old model file
  if (startAngle.wasOnFile())
    obj->visualize3DAngles.getValue().first = startAngle.getValue();
  if (stopAngle.wasOnFile())
    obj->visualize3DAngles.getValue().second = stopAngle.getValue();

  if (!obj->connect())
    // This riser assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "
                << obj->getIdString() << std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


static bool stitchTriads(FmTriad* t1, FmTriad* t2,
                         const std::string& d1, const std::string& d2,
                         FmMathFuncBase* contSpring, FmBase* parent,
                         bool interconnectXY)
{
  if (!t1 || !t2) return false;

  // Ensure that triad t2 is not already dependent in another joint
  if (t2->isSlaveTriad(true))
  {
    if (t1->isSlaveTriad(true))
    {
      ListUI <<"  -> ERROR: Both "<< t1->getIdString(true)
             <<" and "<< t2->getIdString(true) <<" are dependent.\n";
      return false;
    }
    else
      std::swap(t1,t2);
  }

  // Set the joint coordinate system such that its local Z-axis is parallel
  // to the local X-axis of the (first) beam element connected to it
  std::vector<FmBeam*> beams;
  t1->getBeamBinding(beams);
  if (beams.empty())
  {
    ListUI <<"  -> ERROR: No beam attached to "<< t1->getIdString(true) <<".\n";
    return false;
  }

  // Ensure the dependent triad is free
  int iDof;
  for (iDof = 0; iDof < 6; iDof++)
    if (t2->getStatusOfDOF(iDof) != FmHasDOFsBase::FREE)
    {
      t2->setStatusForDOF(iDof,FmHasDOFsBase::FREE);
      ListUI <<"  -> NOTE: Resetting DOF "<< (iDof < 3 ? 'T' : 'R')
             << char('X'+ iDof%3) <<" in "<< t2->getIdString(true)
             <<" to FREE.\n";
    }

  FaMat33 orient = beams.front()->getGlobalCS().direction();
  FaMat34 cs(orient.shift(-1),t1->getGlobalTranslation());

  // Create a contact joint
  FmSMJointBase* jnt = new FmFreeJoint();
  jnt->setParentAssembly(parent);
  jnt->setUserDescription("Coupling " + d1 + " and " + d2);
  jnt->setMasterMovedAlong(true);
  jnt->setSlaveMovedAlong(true);
  jnt->setAsMasterTriad(t1);
  jnt->setAsSlaveTriad(t2);
  jnt->setGlobalCS(cs);
  jnt->updateLocation();
  jnt->connect();
  jnt->draw();

  // Attach the contact spring function to the local X- and Y-dofs
  for (iDof = 0; iDof < 2; iDof++)
  {
    jnt->setStatusForDOF(iDof,FmHasDOFsBase::SPRING_CONSTRAINED);
    jnt->getSpringAtDOF(iDof,true)->setSpringCharOrStiffFunction(contSpring);
  }

  if (interconnectXY)
    jnt->tranSpringCpl.setValue(FmJointBase::XY);

  ListUI <<"  => Connected "<< t1->getIdString(true)
         <<" to "<< t2->getIdString(true) <<".\n";
  return true;
}


bool FmRiser::stitch(FmModelMemberBase* beam1,
                     FmModelMemberBase* beam2,
                     FmModelMemberBase* contactSpring,
                     bool interconnectXY, double tolDist)
{
  if (!dynamic_cast<FmRiser*>(beam1) && !dynamic_cast<FmSoilPile*>(beam1))
    return false;

  if (!dynamic_cast<FmRiser*>(beam2) || beam2 == beam1)
    return false;

  FmMathFuncBase* contSpring = dynamic_cast<FmMathFuncBase*>(contactSpring);
  if (!contSpring) return false;

  if (contSpring->getFunctionUse() != FmMathFuncBase::SPR_TRA_STIFF &&
      contSpring->getFunctionUse() != FmMathFuncBase::SPR_TRA_FORCE)
  {
    ListUI <<"  -> ERROR: Invalid contact spring function "
           << contSpring->getIdString(true) <<"\n";
    return false;
  }

  // Find the common parent
  FmBase* p2 = NULL;
  FmBase* parent = beam1->getParentAssembly();
  for (; parent != p2; parent = parent->getParentAssembly())
    for (p2 = beam2->getParentAssembly(); p2; p2 = p2->getParentAssembly())
      if (p2 == parent) break;

  // Get all triads in the two beamstring assemblies
  std::vector<FmTriad*> triads1, triads2;
  FmDB::getAllTriads(triads1,static_cast<FmSubAssembly*>(beam1));
  FmDB::getAllTriads(triads2,static_cast<FmSubAssembly*>(beam2));

  // For soil piles, we must skip the grounded triads
  size_t i = 0;
  if (dynamic_cast<FmSoilPile*>(beam1))
  {
    while (i < triads1.size())
      if (triads1[i]->isAttached(FmDB::getEarthLink()))
        triads1.erase(triads1.begin()+i);
      else
        i++;
  }

  std::string desc1 = beam1->getUserDescription();
  std::string desc2 = beam2->getUserDescription();
  if (desc1.empty()) desc1 = beam1->getIdString();
  if (desc2.empty()) desc2 = beam2->getIdString();

  // Traverse all triads in first beam string
  int numJnt = 0;
  bool randomSearch = false;
  std::vector<FmTriad*>::iterator t1, t2, t3, t4;
  for (t1 = triads1.begin(); t1 != triads1.end();)
  {
    // Traverse the triads of the second beam string,
    // restarting from the beginning if no match yet, or random search
    if (numJnt == 0 || randomSearch) t2 = t3 = triads2.begin();
    FaVec3 x1 = (*t1)->getGlobalTranslation();
    for (; t2 != triads2.end(); t3 = t2, ++t2)
      if ((x1-(*t2)->getGlobalTranslation()).length() < tolDist)
      {
        // We found two matching triads
        if (numJnt == 0 && t2 != triads2.begin())
        {
          // Check if the triads2 array needs to be traversed in opposite order
          FmTriad* match = NULL;
          FaVec3 x3 = (*t3)->getGlobalTranslation();
          for (i = 0, t4 = t1+1; i < 3 && t4 != triads1.end() && !match; i++, ++t4)
            if ((x3-(*t4)->getGlobalTranslation()).length() < tolDist)
              match = *t2; // yes, the opposite order is better

          if (match)
          {
            // Reverse the triad ordering of the second beamstring,
            // while keeping track of the matching triad (*t2) there
            std::reverse(triads2.begin(),triads2.end());
            for (t2 = triads2.begin(); t2 != triads2.end(); ++t2)
              if (*t2 == match) break;
          }
        }

        // Create a Free Joint connecting the two triads *t1 and *t2
        if (t2 != triads2.end())
          if (stitchTriads(*t1,*t2,desc1,desc2,contSpring,parent,interconnectXY))
          {
            numJnt++;
            if (randomSearch)
              t2 = triads2.erase(t2); // remove found triad from list to search in
          }
        break;
      }

    if (!randomSearch && numJnt > 0 && t2 == triads2.end())
    {
      // We did not find the triad t1 among the triads2.
      // Start over searching in random order (more costly though).
      randomSearch = true;
      FFaMsg::list(" ==> Warning: The triads in the two beam strings are not"
                   " arranged in consequtive order.\n",true);
      ListUI <<"     Using a more time-consuming search algorithm for the last "
             << int(triads1.end()-t1) <<" triads...\n";
    }
    else
      ++t1;
  }

  if (numJnt < 1) return false;

  ListUI <<" ==> "<< numJnt <<" Free Joints";
  if (parent) ListUI <<" in "<< parent->getIdString(true);
  ListUI <<" created, stitching together "<< desc1 <<" and "<< desc2 <<"\n";
  return true;
}


bool FmRiser::split(FmModelMemberBase* beam1, FmModelMemberBase* beam2)
{
  if (!dynamic_cast<FmRiser*>(beam1) && !dynamic_cast<FmSoilPile*>(beam1))
    return false;

  if (!dynamic_cast<FmRiser*>(beam2) || beam2 == beam1)
    return false;

  // TODO...
  FFaMsg::dialog("This function is not yet implemented. Sorry...",FFaMsg::ERROR);
  return false;
}
