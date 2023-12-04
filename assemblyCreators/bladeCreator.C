// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "turbineConverter.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmBladeProperty.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"


static FmBeam* addBeam (FmModelMemberBase* parent, FmModelMemberBase* prop,
                        FmTriad*& tr2, const FaVec3& localZ, double Len,
                        const FaMat33& endCS)
{
  FmTriad* tr1 = tr2;
  tr2 = new FmTriad(FaVec3(0.0,0.0,Len)); // Placing triads along the Z-axis
  tr2->setParentAssembly(parent);
  tr2->connect();
  tr2->setOrientation(endCS);

  FmBeam* beam = new FmBeam();
  beam->setParentAssembly(parent);
  beam->setProperty(prop);
  beam->alpha2.setValue(0.005); // Default stiffness-proportional damping
  beam->setOrientation(localZ);
  beam->connect(tr1,tr2);

  return beam;
}


bool FWP::createBladeElements (FmRotor* theRotor, FmLink* hub,
			       FmBladeDesign* bladeDef,
			       const std::vector<FmBladeProperty*>& blSeg)
{
  // Get segment lengths
  std::vector<double> DRNode;
  if (bladeDef && !bladeDef->getSegmentLengths(DRNode))
    return false;

  std::vector<FmModelMemberBase*> blades;
  FmDB::getAllOfType(blades,FmSubAssembly::getClassTypeID(),theRotor);
  std::vector<FmRevJoint*> pitch(blades.size());

  // Get some hub properties
  double preCone = RAD(theRotor->PreCone.getValue());
  double hubRad = theRotor->HubDiam.getValue()*0.5;
  double cpc = cos(preCone);
  double spc = sin(preCone);

  // Loop over the blades
  int totElms = 0;
  for (size_t b = 0; b < blades.size(); b++)
  {
    FmBlade* blade = static_cast<FmBlade*>(blades[b]);

    // Define the pitch axis of the b'th blade.
    // Note the negative angle psi here, this is needed because AeroDyn
    // requires the blades to pass through a given azimuth in the 3-2-1 order.
    double psi = RAD(-360.0*(double)b/(double)blades.size());
    FaVec3 pitchAxis(spc,cpc*sin(psi),cpc*cos(psi));

    // Define the local coordinate system for the b'th blade
    FaVec3 yAxis(0.0,pitchAxis[2],-pitchAxis[1]); yAxis.normalize();
    FaMat34 bladeCS(yAxis^pitchAxis,yAxis,pitchAxis,hubRad*pitchAxis);
    blade->setLocalCS(bladeCS);

    // Master triad at the blade root connected to the hub
    FmTriad* cone = new FmTriad();
    cone->setParentAssembly(theRotor);
    cone->setUserDescription(FFaNumStr("Cone %d",b+1));
    cone->setLocalCS(bladeCS);
    cone->connect(hub);

    // Slave triad at the blade root
    FmTriad* triad2 = new FmTriad();
    triad2->setParentAssembly(blade);
    triad2->setUserDescription(FFaNumStr("Blade %d",b+1));
    triad2->connect();

    // Create the pitch joint connecting the blade to the hub.
    // Its DOF status is set as FIXED here. The user has to
    // assign other conditions manually, if desired.
    pitch[b] = new FmRevJoint();
    pitch[b]->setParentAssembly(theRotor);
    pitch[b]->setUserDescription(FFaNumStr("Pitch %d",b+1));
    pitch[b]->setAsMasterTriad(cone);
    pitch[b]->setAsSlaveTriad(triad2);
    pitch[b]->updateLocation();
    pitch[b]->setStatusForDOF(5,FmHasDOFsBase::FIXED);
    pitch[b]->connect();

    // Now create the beam elements of the blade. Note that the triads along
    // the blade are placed on the pitch axis. They are rotated according to the
    // Twist angle of the first connected blade element, which also is rotated.
    double Rnode = 0.0;
    for (size_t i = 0; i < DRNode.size(); i++)
    {
      double Length = DRNode[i]*0.5;
      double Atwist = blSeg[i]->Twist.getValue();
      double Btwist = blSeg[i]->ElAxisRot.getValue() + Atwist;
      FaVec3 localZ(cos(RAD(Btwist)),sin(RAD(-Btwist)),0.0);
      FaMat33 endCS = FaMat33::makeZrotation(RAD(-Atwist));
      for (int e = 1; e <= 2; e++, totElms++, Rnode += Length)
	addBeam(blade, blSeg[i], triad2, localZ, Rnode+Length, endCS);
    }
  }
  ListUI <<"  -> Created "<< totElms <<" blade elements.\n";
  return true;
}


bool FWP::updateBladeElements (FmRotor* theRotor, FmBladeDesign* bladeDef,
			       const std::vector<FmBladeProperty*>& blSeg)
{
  // Get segment lengths
  std::vector<double> DRNode;
  if (bladeDef && !bladeDef->getSegmentLengths(DRNode))
    return false;

  std::vector<FmModelMemberBase*> blades, pitch;
  FmDB::getAllOfType(blades,FmSubAssembly::getClassTypeID(),theRotor);
  FmDB::getAllOfType(pitch,FmRevJoint::getClassTypeID(),theRotor);
  if (blades.size() != pitch.size())
  {
    ListUI <<" *** Number of pitch joints "<< (int)pitch.size() <<" is not"
	   <<" equal to the number of blades "<< (int)blades.size() <<"\n";
    return false;
  }

  // Get some hub properties
  double preCone = RAD(theRotor->PreCone.getValue());
  double hubRad = theRotor->HubDiam.getValue()*0.5;
  double cpc = cos(preCone);
  double spc = sin(preCone);

  // Loop over the blades
  FmTriad* triad = NULL;
  int updElms = 0, newElms = 0;
  for (size_t b = 0; b < blades.size(); b++)
  {
    FmBlade* blade = static_cast<FmBlade*>(blades[b]);

    // Define the pitch axis of the b'th blade.
    // Note the negative angle psi here, this is needed because AeroDyn
    // requires the blades to pass through a given azimuth in the 3-2-1 order.
    double psi = RAD(-360.0*(double)b/(double)blades.size());
    FaVec3 pitchAxis(spc,cpc*sin(psi),cpc*cos(psi));

    // Define the local coordinate system for the b'th blade
    FaVec3 yAxis(0.0,pitchAxis[2],-pitchAxis[1]); yAxis.normalize();
    FaMat34 bladeCS(yAxis^pitchAxis,yAxis,pitchAxis,hubRad*pitchAxis);
    blade->setLocalCS(bladeCS);

    // Fetch the current blade elements, if any
    std::vector<FmBeam*> elms;
    FmDB::getAllBeams(elms,blade);

    // Master triad at the blade root connected to the hub
    FmTriad* cone = static_cast<FmSMJointBase*>(pitch[b])->getItsMasterTriad();
    cone->setLocalCS(bladeCS);
    cone->onChanged();

    // Fetch current tip triad, if any
    if (elms.empty())
      triad = static_cast<FmSMJointBase*>(pitch[b])->getSlaveTriad();
    else
      triad = elms.back()->getSecondTriad();

    // Now update the beam elements of the blade. Note that the triads along
    // the blade are placed on the pitch axis. They are rotated according to the
    // Twist angle of the first connected blade element, which also is rotated.
    double Rnode = 0.0;
    size_t iel   = 0;
    for (size_t i = 0; i < DRNode.size(); i++)
    {
      double Atwist = blSeg[i]->Twist.getValue();
      double Btwist = Atwist + blSeg[i]->ElAxisRot.getValue();
      FaVec3 localZ(cos(RAD(-Btwist)),sin(RAD(Btwist)),0.0);
      FaMat33 endCS = FaMat33::makeZrotation(RAD(-Atwist));
      double Length = DRNode[i]*0.5;

      for (int e = 1; e <= 2; e++, iel++, Rnode += Length)
      {
	if (iel == elms.size())
	{
	  newElms++; // Increasing the number of blade elements
	  elms.push_back(addBeam(blade, blSeg[i], triad, localZ,
				 Rnode+Length, endCS));
	}
	else
	{
	  updElms++; // Updating the existing beam element
	  elms[iel]->setOrientation(localZ);
	  elms[iel]->setProperty(blSeg[i]);

	  // Update position of the second triad of the existing element
	  FmTriad* end = elms[iel]->getSecondTriad();
	  end->setLocalCS(FaMat34(endCS,FaVec3(0.0,0.0,Rnode+Length)));
	}
	elms[iel]->onChanged();
      }
    }

    if (elms.size() > iel)
    {
      // Update the tip triad
      triad->setTranslation(FaVec3(0.0,0.0,DRNode.back()*0.5));
      triad->onChanged();

      // Erase superfluous blade elements in case the number has been reduced
      for (; elms.size() > iel; --newElms)
      {
	elms.back()->getSecondTriad()->erase();
	elms.back()->erase();
	elms.pop_back();
      }
    }
  }

  ListUI <<"  -> Updated "<< updElms <<" blade elements.\n";
  if (newElms > 0)
    ListUI <<"     Created "<< newElms <<" blade elements.\n";
  else if (newElms < 0)
    ListUI <<"     Removed "<< -newElms <<" old blade elements.\n";

  return true;
}
