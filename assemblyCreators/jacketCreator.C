// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "assemblyCreators.H"
#include "vpmDB/FmJacket.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "FFlLib/FFlLinkHandler.H"
#include "FFlLib/FFlElementBase.H"
#include "FFlLib/FFlFEParts/FFlPMAT.H"
#include "FFlLib/FFlFEParts/FFlPMASS.H"
#include "FFlLib/FFlFEParts/FFlPBEAMSECTION.H"
#include "FFlLib/FFlFEParts/FFlPBEAMECCENT.H"
#include "FFlLib/FFlFEParts/FFlPORIENT.H"
#include "FFlLib/FFlFEParts/FFlPSPRING.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef FF_NAMESPACE
using namespace FF_NAMESPACE;
#endif


void FWP::createJacket (const FFlLinkHandler* jl, const std::string& name,
                        const double* Morison, int IDoffset)
{
  double Ca = Morison[0];
  double Cm = Morison[1];
  double Cd = Morison[2];

  FmJacket* jacket = new FmJacket();
  jacket->setUserDescription(FFaFilePath::getBaseName(name,true));
  jacket->connect();

  std::vector<int> assID(1,jacket->getID());

  std::map<FmBeamProperty*,FmBeamProperty*>::const_iterator cit;
  std::map<FmBeamProperty*,FmBeamProperty*> dupCS;

  int newNodeID = jl->getNewNodeID();
  bool addRigid = true;
  int  nRigid = 0, nSpring = 0;
  std::vector<FmFreeJoint*> groundSpr;

  // Create a beam object for each beam element in the provided FE model
  bool haveShells = false;
  bool haveSolids = false;
  int nBeam = 0, nTriad = 0;
  std::map<std::string,int> ignored;
  ListUI <<"\n===> Generating jacket model\n";
  for (ElementsCIter eit = jl->elementsBegin(); eit != jl->elementsEnd(); ++eit)
  {
    const std::string& elmtype = (*eit)->getTypeName();
    if (elmtype == "BEAM2")
    {
      FFlPMAT* pmat = dynamic_cast<FFlPMAT*>((*eit)->getAttribute("PMAT"));
      if (!pmat)
      {
        ListUI <<" ==> Ignoring beam element "<< (*eit)->getID()
               <<" without material properties.\n";
        continue;
      }

      FFlPBEAMSECTION* psec = dynamic_cast<FFlPBEAMSECTION*>((*eit)->getAttribute("PBEAMSECTION"));
      if (!psec)
      {
        ListUI <<" ==> Ignoring beam element "<< (*eit)->getID()
               <<" without cross section properties.\n";
        continue;
      }

      // Check if these properties already have been created
      FmBase* bmat  = FmDB::findID(FmMaterialProperty::getClassTypeID(),
                                   IDoffset+pmat->getID(),assID);
      FmBase* bprop = FmDB::findID(FmBeamProperty::getClassTypeID(),
                                   IDoffset+psec->getID(),assID);

      FmMaterialProperty* elmMat;
      if (bmat)
        elmMat = static_cast<FmMaterialProperty*>(bmat);
      else
      {
        // Create material property object
        elmMat = new FmMaterialProperty();
        elmMat->setParentAssembly(jacket);
        elmMat->setID(IDoffset+pmat->getID());
        elmMat->setUserDescription(pmat->getName());
        elmMat->connect();
        elmMat->updateProperties(pmat->materialDensity.getValue(),
                                 pmat->youngsModule.getValue(),
                                 pmat->shearModule.getValue());
      }

      FmBeamProperty* elmProp;
      if (bprop)
      {
        elmProp = static_cast<FmBeamProperty*>(bprop);
        if (elmProp->material.isNull())
          elmProp->material.setRef(elmMat);
        else if (elmProp->material.getPointer() != elmMat)
        {
          // This cross section property already exists,
          // but it refers to another material.
          for (cit = dupCS.begin(); cit != dupCS.end(); ++cit)
            if (cit->second == elmProp && cit->first->material.getPointer() == elmMat)
              break;

          if (cit != dupCS.end())
            elmProp = cit->first; // Found a duplicate with this material
          else
          {
            // Create a new copy of it for the new material reference
            ListUI <<"  -> Duplicating "<< elmProp->getIdString(true) <<"\n";
            FmBeamProperty* newProp = new FmBeamProperty();
            newProp->clone(elmProp,FmBase::SHALLOW);
            newProp->setParentAssembly(jacket);
            newProp->material.setRef(elmMat);
            // Do not connect these objects yet, to avoid ID conflicts
            newProp->setID(0);
            newProp->setUserDescription("Copy of " + elmProp->getInfoString());
            // Store reference to the 'master' cross section
            dupCS[newProp] = elmProp;
            elmProp = newProp;
          }
        }
      }
      else
      {
        // Create beam cross section object
        elmProp = new FmBeamProperty();
        elmProp->setParentAssembly(jacket);
        elmProp->setID(IDoffset+psec->getID());
        elmProp->setUserDescription(psec->getName());
        elmProp->connect();
        elmProp->material.setRef(elmMat);
        if (psec->Iy.getValue() == psec->Iz.getValue())
        {
          // Extract cross sectional parameters (assuming a circular pipe)
          double A = psec->crossSectionArea.getValue();
          double I2oA = (psec->Iy.getValue() + psec->Iz.getValue())/A;
          double Ao2Pi = 0.5*A/M_PI;
          double D =     2.0*sqrt(I2oA + Ao2Pi); // Outer diameter
          double t = 0.5*D - sqrt(I2oA - Ao2Pi); // Wall thickness

          elmProp->crossSectionType.setValue(FmBeamProperty::PIPE);
          elmProp->Do.setValue(D);
          elmProp->Di.setValue(D-2.0*t);
          elmProp->Dd.setValue(D);
          elmProp->Db.setValue(D);
          elmProp->Ca.setValue(Ca);
          elmProp->Cm.setValue(Cm);
          elmProp->Cd.setValue(Cd);
          elmProp->hydroToggle.setValue(true);
          elmProp->updateDependentValues();
        }
        else
        {
          // Use generic cross section (unknown geometry)
          double A = psec->crossSectionArea.getValue();
          double E = elmMat->E.getValue();
          double G = elmMat->G.getValue();
          double rho = elmMat->Rho.getValue();

          // Note: The Iy and Iz values are swapped here.
          // Probably there is a defintion mismatch somewere....
          // TODO: Check this and find out why and fix properly.
          elmProp->crossSectionType.setValue(FmBeamProperty::GENERIC);
          elmProp->A.setValue(A);
          elmProp->Iy.setValue(psec->Iz.getValue()); // y,z swapped
          elmProp->Iz.setValue(psec->Iy.getValue()); // y,z swapped!
          elmProp->Ip.setValue(psec->Iy.getValue() + psec->Iz.getValue());
          elmProp->EA.setValue(E*A);
          elmProp->EI.getValue().first = E*psec->Iz.getValue(); // y,z swapped!
          elmProp->EI.getValue().second = E*psec->Iy.getValue(); // z,y swapped!
          if (psec->Kxy.getValue() > 0.0)
            elmProp->GAs.getValue().second = G*A/psec->Kxy.getValue(); // y,z swapped!
          if (psec->Kxz.getValue() > 0.0)
            elmProp->GAs.getValue().first = G*A/psec->Kxz.getValue(); // y,z swapped!
          elmProp->GIt.setValue(G*psec->It.getValue());
          elmProp->Mass.setValue(rho*A);
        }
      }

      // Create a triad at each end of the beam element
      FmTriad* t1 = FmTriad::createAtNode((*eit)->getNode(1),jacket,IDoffset,nTriad);
      FmTriad* t2 = FmTriad::createAtNode((*eit)->getNode(2),jacket,IDoffset,nTriad);
      FaVec3   X1 = t1->getGlobalTranslation();
      FaVec3   X2 = t2->getGlobalTranslation();

      // Check for eccentricities
      double massScale = 1.0;
      FFlPBEAMECCENT* pecc = dynamic_cast<FFlPBEAMECCENT*>((*eit)->getAttribute("PBEAMECCENT"));
      if (pecc)
      {
        if (addRigid)
        {
          // Create rigid joints connecting the node (master Triad) and beam end
          double tol = 1.0e-6*(X2-X1).length();
          if (!pecc->node1Offset.getValue().isZero(tol))
          {
            FmTriad* s1 = new FmTriad(X1 + pecc->node1Offset.getValue());
            s1->setParentAssembly(jacket);
            s1->setID(IDoffset+(newNodeID++));
            s1->connect();
            nTriad++;

            FmRigidJoint* joint = new FmRigidJoint();
            joint->setParentAssembly(jacket);
            joint->setAsMasterTriad(t1);
            joint->setAsSlaveTriad(s1);
            joint->updateLocation();
            joint->connect();
            nRigid++;

            t1 = s1;
          }
          if (!pecc->node2Offset.getValue().isZero(tol))
          {
            FmTriad* s2 = new FmTriad(X2 + pecc->node2Offset.getValue());
            s2->setParentAssembly(jacket);
            s2->setID(IDoffset+(newNodeID++));
            s2->connect();
            nTriad++;

            FmRigidJoint* joint = new FmRigidJoint();
            joint->setParentAssembly(jacket);
            joint->setAsMasterTriad(t2);
            joint->setAsSlaveTriad(s2);
            joint->updateLocation();
            joint->connect();
            nRigid++;

            t2 = s2;
          }
        }
        else
        {
          // Account for the eccentricities only be scaling the mass
          // (ignoring the stiffness difference here)
          double actualLength = (X2-X1).length();
          double effectiveLen = ((X2+pecc->node2Offset.getValue()) -
                                 (X1+pecc->node1Offset.getValue())).length();
          massScale = effectiveLen/actualLength;
        }
      }

      // Create a beam element
      FmBeam* beam = new FmBeam();
      beam->setParentAssembly(jacket);
      beam->setID(IDoffset+(*eit)->getID());
      if (!psec->getName().empty())
        beam->setUserDescription(psec->getName());
      beam->massScale.setValue(massScale);
      beam->connect(t1,t2);
      beam->setProperty(elmProp);

      // Define the local element coordinate system
      FFlPORIENT* bo = dynamic_cast<FFlPORIENT*>((*eit)->getAttribute("PORIENT"));
      if (bo && !bo->directionVector.getValue().isZero())
        beam->setOrientation(bo->directionVector.getValue());

      nBeam++;
    }

    else if (elmtype == "CMASS")
    {
      FFlPMASS* pmass = dynamic_cast<FFlPMASS*>((*eit)->getAttribute("PMASS"));
      if (!pmass)
      {
        ListUI <<" ==> Ignoring mass element "<< (*eit)->getID()
               <<" without properties.\n";
        continue;
      }

      // Create a triad for the point mass
      FmTriad* tr = FmTriad::createAtNode((*eit)->getNode(1),jacket,IDoffset,nTriad);

      const std::vector<double>& M = pmass->M.getValue();
      size_t i, j, k;
      for (j = k = 0; j < 6 && k < M.size(); j++, k++)
      {
        for (i = 0; i < j && k < M.size(); i++, k++)
          if (M[k] != 0.0)
            ListUI <<"  ** Warning: Off-diagonal nodal mass M("
                   << i+1 <<","<< j+1 <<")="<< M[k] <<" is ignored.\n";
        tr->setAddMass(j,M[k]);
      }
      tr->onChanged(); // to update Triad icon
    }

    else if (elmtype == "RSPRING")
    {
      FFlPSPRING* spr = dynamic_cast<FFlPSPRING*>((*eit)->getAttribute("PSPRING"));
      if (!spr)
      {
        ListUI <<" ==> Ignoring spring element "<< (*eit)->getID()
               <<" without properties.\n";
        continue;
      }

      // Create a triad for the spring
      FmTriad* t1 = FmTriad::createAtNode((*eit)->getNode(1),jacket,IDoffset,nTriad);
      // Create the second triad, unless it is a ground spring
      FmTriad* t2 = FmTriad::createAtNode((*eit)->getNode(2),jacket,IDoffset,nTriad);

      // Create a free joint representing the spring element
      FmFreeJoint* joint = new FmFreeJoint();
      joint->setParentAssembly(jacket);
      joint->setID(IDoffset+(*eit)->getID());
      joint->setAsSlaveTriad(t1);
      if (t2)
      {
        joint->setAsMasterTriad(t2);
        joint->updateLocation();
        t1->onChanged();
        t2->onChanged();
      }
      else
        groundSpr.push_back(joint); // Add grounded master triad later

      // Find the diagonal stiffness terms
      size_t i, j, k;
      double tol = 0.0;
      for (j = k = 0; j < 6; k += 6-(j++))
      {
        joint->setStatusForDOF(j,FmHasDOFsBase::SPRING_CONSTRAINED);
        joint->getSpringAtDOF(j,true)->setInitStiff(spr->K[k].getValue());
        tol += fabs(spr->K[k].getValue());
      }

      // Find the off-diagonal stiffness terms, if any, upper diagonal only
      std::string description("#GlobalSpring"), K(" #K__ ");
      tol *= 1.666666666666667e-16;
      for (i = 0, k = 1, K[3] = '1'; i < 6; i++, k++, K[3]++)
        for (j = i+1, K[4] = K[3]+1; j < 6; j++, k++, K[4]++)
          if (fabs(spr->K[k].getValue()) > tol)
            description += K + FFaNumStr(spr->K[k].getValue());

      joint->setUserDescription(description);
      joint->connect();
      nSpring++;
    }

    else if (ignored.find(elmtype) == ignored.end())
    {
      ignored[elmtype] = 1;
      if (elmtype == "TRI3" || elmtype == "TRI6" ||
          elmtype == "QUAD4" || elmtype == "QUAD8")
        haveShells = true;
      else if (elmtype == "TET4"  || elmtype == "TET10" ||
               elmtype == "WEDG6" || elmtype == "WEDG15" ||
               elmtype == "HEX8"  || elmtype == "HEX20")
        haveSolids = true;
    }
    else
      ignored[elmtype] ++;
  }

  // Now create master triads for the grounded springs
  for (size_t i = 0; i < groundSpr.size(); i++)
  {
    FmTriad* slave = groundSpr[i]->getSlaveTriad();
    FmTriad* master = new FmTriad(slave->getGlobalTranslation());
    master->setParentAssembly(jacket);
    master->connect(FmDB::getEarthLink());
    groundSpr[i]->setAsMasterTriad(master);
    groundSpr[i]->updateLocation();
    slave->onChanged();
    master->onChanged();
    nTriad++;
  }

  // Now connect all duplicated cross section entries
  for (cit = dupCS.begin(); cit != dupCS.end(); ++cit)
    cit->first->connect();

  ListUI <<"  -> Created "<< nBeam <<" beam elements and "<< nTriad <<" new triads.\n";
  if (nRigid > 0)
    ListUI <<"     Created "<< nRigid <<" rigid joints (for beam eccentricities).\n";
  if (nSpring > 0)
    ListUI <<"     Created "<< nSpring <<" global spring elements.\n";
  for (const std::pair<const std::string,int>& it : ignored)
    ListUI <<"  -> Ignored "<< it.second <<" "<< it.first <<" elements.\n";

  if (haveShells || haveSolids)
  {
    std::string warning("This FE model contains ");
    warning += haveShells ? "shell" : "solid";
    warning += " elements.\nThese elements have been ignored from this import.\n";
    FFaMsg::dialog(warning + "See the Output List view for details.\n"
                   "It is recommended to organize the shell/volume elements of the model\n"
                   "in separate FE data files, and import them as regular FE parts instead.",
                   FFaMsg::WARNING);
  }

  if (FmDB::hasObjectsOfType(FmLink::getClassTypeID(),jacket->getHeadMap()))
    FmDB::displayAll(*jacket->getHeadMap());
  else
  {
    FFaMsg::list(" ==> Empty jacket assembly, deleted.\n",true);
    jacket->erase();
  }
}
