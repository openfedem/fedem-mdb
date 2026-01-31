// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifdef USE_INVENTOR
#include "vpmDisplay/FdTriad.H"
#include "vpmDisplay/FdPart.H"
#include "vpmDisplay/FdExtraGraphics.H"
#endif

#include "vpmDB/Icons/triadSymbols.h"

#include "FFlLib/FFlLinkHandler.H"
#include "FFlLib/FFlFEParts/FFlNode.H"

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmTriad.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmSticker.H"
#include "vpmDB/FmRigidJoint.H"
#include "vpmDB/FmFreeJoint.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmPart.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmLoad.H"
#include "vpmDB/FmTire.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmSeaState.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmGlobalViewSettings.H"

#define SIZEMASS(n) (n == 6 ? 9 : n)


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcTRIAD, FmTriad, FmHasDOFsBase);


FmTriad::FmTriad()
{
  this->init();
}


FmTriad::FmTriad(const FaVec3& globalPos)
{
  this->init();
  this->setTranslation(globalPos);
}


void FmTriad::init()
{
  Fmd_CONSTRUCTOR_INIT(FmTriad);

  FFA_FIELD_DEFAULT_INIT(itsBndC, "DOF_STATUS");
  FFA_FIELD_DEFAULT_INIT(itsMass, "ADD_MASS");

  FFA_FIELD_INIT(itsNDOFs, 6, "NDOFS"); // TT #2812: Must be non-zero
  // to allow un-attached triads with only additional mass
  FFA_FIELD_INIT(FENodeNo, -1, "FE_NODE_NO");
  FFA_FIELD_INIT(itsLocalDir, GLOBAL, "LOCAL_DIRECTIONS");

#ifdef FT_USE_CONNECTORS
  FFA_FIELD_DEFAULT_INIT(itsConnectorGeometry, "CONNECTOR_GEOMETRY");
  FFA_FIELD_DEFAULT_INIT(itsConnectorItems, "CONNECTOR_FE_ITEMS");
  FFA_FIELD_INIT(itsConnectorType, NONE, "CONNECTOR_TYPE");
#endif

  FFA_REFERENCELIST_FIELD_INIT(myAttachedLinksField,
                               myAttachedLinks, "OWNER_LINK");

  FFA_REFERENCE_FIELD_INIT(myMassEngineField, myMassEngine, "MASS_ENGINE");
  myMassEngine.setPrintIfZero(false);

  this->completeInitDOFs();

#ifdef USE_INVENTOR
  itsDisplayPt = new FdTriad(this);
#endif
}


FmTriad::~FmTriad()
{
  this->disconnect();

  // Removal of the connected springs, dampers and loads
  std::vector<FmIsControlledBase*> objs;
  this->getReferringObjs(objs);

  for (FmIsControlledBase* obj : objs)
    obj->erase();
}


bool FmTriad::highlight(bool trueOrFalse)
{
  if (!this->FmHasDOFsBase::highlight(trueOrFalse))
    return false;

#ifdef USE_INVENTOR
  FmPart* owner = this->getOwnerFEPart();
  if (!owner) return true;

  // Highlight the connector geometry, if any
  const FFaCompoundGeometry& connector = itsConnectorGeometry.getValue();
  for (size_t i = 0; i < connector.size(); i++)
    if (connector[i]->getAddExclude())
      FdExtraGraphics::highlight(connector[i],owner->getGlobalCS(),trueOrFalse);
#endif

  return true;
}


#ifdef FT_USE_CONNECTORS
/*!
  Updates a connector using the triad's stored info.
  Unless \a ownerPart is NULL, the connector is recreated even if its type is
  not changed (this is typically used when changing the FE mesh of a part).
  If \a ownerPart is NULL, we do nothing unless the connector type is changed.
*/

bool FmTriad::updateConnector(ConnectorType type, FmPart* ownerPart)
{
  FmPart* owner = ownerPart ? ownerPart : this->getOwnerFEPart();
  if (!owner) return false;

  bool changed = itsConnectorType.setValue(type);
  if (!changed && !ownerPart) return false;

  FFlLinkHandler* FEdata = owner->getLinkHandler();
  if (!FEdata) return changed;

  FFlConnectorItems& items = itsConnectorItems.getValue();
  if (!items.empty())
    changed |= FEdata->deleteConnector(items) > 0;

  if (type > NONE)
    changed |= FEdata->createConnector(itsConnectorGeometry.getValue(),
                                       this->getTranslation(),type,items) > 0;
  else
    items.clear();

  if (changed && !ownerPart)
    owner->delayedCheckSumUpdate();

  return changed;
}
#endif


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

FmPart* FmTriad::getOwnerFEPart() const
{
  FmPart* owner = NULL;
  for (int indx = 0; (owner = this->getOwnerPart(indx)); indx++)
    if (owner->isFEPart())
      return owner; // assume only one FE part connection

  return NULL;
}


FmPart* FmTriad::getOwnerPart(int partIndex) const
{
  if (myAttachedLinks.empty())
    return NULL;
  else if (partIndex >= 0)
    return dynamic_cast<FmPart*>(myAttachedLinks.getPtr(partIndex));
  else if (myAttachedLinks.size() == 1)
    return dynamic_cast<FmPart*>(myAttachedLinks.getFirstPtr());

  return NULL;
}


FmLink* FmTriad::getOwnerLink(int linkIndex) const
{
  if (FmLink* part = this->getOwnerPart(linkIndex); part)
    return part;

  std::vector<FmLink*> elms;
  this->getReferringObjs(elms,"myTriads");
  int elmIndex = linkIndex - myAttachedLinks.size();
  if (elmIndex >= static_cast<int>(elms.size()))
    return NULL;
  else if (elmIndex >= 0)
    return elms[elmIndex];
  else if (elms.size() == 1 && myAttachedLinks.empty())
    return elms.front();

  return NULL;
}


/*!
  This method is an "external" erase. It is called from erase().

  1. Check the joints connected.
  1.1 Point joint:
  1.1.1 If the erased triad and joint belong to the same assembly, erase joint
        and the other triad if that one is not referred by other joints.
  1.1.2 If the erased triad and joint belong to different assemblies,
        create a new triad for the joint replacing the erased one.
  1.2 Line joint:
  1.2.1 If the erased triad is a dependent joint triad and belongs to same
        assembly as the joint, try to erase that triad, then erase the joint,
        and erase all independent triads of the erased joint that are empty.
  1.2.2 If the erased triad is a dependent joint triad and belongs to a
        different assembly than the joint itself, create a new dependent triad
        for the joint replacing the erased one.
  1.2.3 If the erased triad is among the independent triads of a joint,
        try to erase that triad if the number of independent triads of the joint
        are greater than two, otherwise erase the joint
        and erase the other triads connected if they are empty.
  2. Invoke the parent class eraseOptions() method.
*/

bool FmTriad::eraseOptions()
{
  // Check for recursion to avoid deleting more than once
  static std::set<FmTriad*> erasingTriad;
  if (erasingTriad.find(this) == erasingTriad.end())
    erasingTriad.insert(this);
  else
    return false;

  std::vector<FmJointBase*> allJoints;
  this->getJointBinding(allJoints);

  for (FmJointBase* joint : allJoints)
    if (joint->isOfType(FmSMJointBase::getClassTypeID()))
    {
      // The joint is a point joint:
      FmTriad* triad = joint->getSlaveTriad();
      FmTriad* other = triad;
      if (this == triad)
        other = static_cast<FmSMJointBase*>(joint)->getItsMasterTriad();

      if (this->getParentAssembly() == joint->getParentAssembly())
      {
        // The joint is in the same assembly as the erased triad,
        // so erase the joint also
        joint->eraseInternal();
        if (other->hasReferences())
          other->updateTopologyInViewer();
        else
          other->erase();
      }
      else
      {
        // The joint is in a different assembly, do not delete it
        // but create a new triad instead in place of the erased one.
        // The new triad is then put in the same assembly as the joint.
        FmTriad* newTriad = new FmTriad(this->getGlobalTranslation());
        newTriad->setUserDescription(this->getUserDescription());
        newTriad->setParentAssembly(joint->getParentAssembly());
        newTriad->connect();
        newTriad->draw();

        if (this == triad)
          joint->setAsSlaveTriad(newTriad);
        else
          static_cast<FmSMJointBase*>(joint)->setAsMasterTriad(newTriad);
        joint->updateTopologyInViewer();
      }
    }
    else if (joint->isOfType(FmMMJointBase::getClassTypeID()))
    {
      // The joint is a line joint:
      FmTriad* triad = joint->getSlaveTriad();
      Fm1DMaster* line = static_cast<FmMMJointBase*>(joint)->getMaster();

      if (this == triad)
      {
        // This is the dependent triad
        if (this->getParentAssembly() == joint->getParentAssembly())
        {
          // The joint is in the same assembly as the deleted triad,
          // so try to erase the independent triad also,
          // and then erase the joint
          if (line)
          {
            static_cast<FmMMJointBase*>(joint)->setMaster(NULL);
            if (FmMMJointBase* other; line->hasReferringObjs(other))
              line->updateTopologyInViewer();
            else
              line->erase();
          }
          joint->eraseInternal();
        }
        else
        {
          // The joint is in a different assembly, do not delete it
          // but create a new triad instead in place of the erased one.
          // The new triad is then put in the same assembly as the joint.
          FmTriad* newTriad = new FmTriad(this->getGlobalTranslation());
          newTriad->setUserDescription(this->getUserDescription());
          newTriad->setParentAssembly(joint->getParentAssembly());
          newTriad->connect();
          newTriad->draw();

          joint->setAsSlaveTriad(newTriad);
          joint->updateTopologyInViewer();
        }
      }
      else if (!line || line->size() < 3)
      {
        // The joint has two or less triads connected and this is one of them.
        // Try to erase the independent joint triads. Note: For line joints
        // the independent triads are assumed to always be in the same
        // assembly as the joint itself.
        // Therefore, we do not check for the other possibility here.
        if (line)
        {
          static_cast<FmMMJointBase*>(joint)->setMaster(NULL);
          if (FmMMJointBase* other; line->hasReferringObjs(other))
            line->updateTopologyInViewer();
          else
            line->erase();
        }

        // Then erase the joint
        joint->eraseInternal();

        // Erasing the dependent triad, unless it is in another sub-assembly
        if (triad->hasReferences() ||
            triad->getParentAssembly() != this->getParentAssembly())
          triad->updateTopologyInViewer();
        else
          triad->erase();
      }
      else
      {
        // Finding this triad among the independent joint triads
        std::vector<FmTriad*> triads;
        line->getTriads(triads);
        for (FmTriad* triad : triads)
          if (triad == this)
            line->releaseTriad(this);

        joint->updateTopologyInViewer();
      }
    }

  bool status = this->FmHasDOFsBase::eraseOptions();
  erasingTriad.erase(this);
  return status;
}


bool FmTriad::interactiveErase()
{
  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);

  for (FmLink* link : links)
    if (link->isDisabled())
    {
      ListUI <<"ERROR: "<< this->getIdString() <<" is not deleted because\n"
             <<"       it is attached to "<< link->getIdString()
             <<" which currently is disabled.\n";
      return false;
    }

  return this->erase();
}


bool FmTriad::isLegalDOF(int dof) const
{
  return dof >= 0 && dof < itsNDOFs.getValue();
}


FmHasDOFsBase::DOFStatus FmTriad::getStatusOfDOF(int dof) const
{
  if (dof >= 0 && dof < static_cast<int>(itsBndC.getValue().size()))
    return itsBndC.getValue()[dof];
  else
    return FREE;
}

bool FmTriad::setStatusForDOF(int dof, DOFStatus status)
{
  if (!this->isLegalDOF(dof))
    return false;

  if (itsBndC.getValue().empty() && status == FREE)
    return false;

  itsBndC.getValue().resize(itsNDOFs.getValue(),FREE);
  if (status > FREE_DYNAMICS || status == itsBndC.getValue()[dof])
    return false;

  itsBndC.getValue()[dof] = status;
  return true;
}

bool FmTriad::setDOFStatus(int DOF, DOFStatus status)
{
  if (!this->setStatusForDOF(DOF,status)) return false;

  this->updateDisplayDetails();
  return true;
}


bool FmTriad::isSuppressed() const
{
  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);
  if (links.empty()) return false;

  for (FmLink* link : links)
    if (!link->isSuppressed() && !link->isEarthLink())
      return false;

  return true;
}


bool FmTriad::hasConstraints(bool fixedOnly) const
{
  if (itsBndC.getValue().empty())
    return false;

  size_t nfree = 0;
  int constrained = 0;
  for (int bcode : itsBndC.getValue())
    switch (bcode)
      {
      case FIXED:
      case FREE_DYNAMICS:
        constrained++;
        break;
      case PRESCRIBED:
        if (!fixedOnly)
          constrained++;
        break;
      case FREE:
        nfree++;
        break;
      default:
        break;
      }

  if (nfree == itsBndC.getValue().size())
    const_cast<FmTriad*>(this)->itsBndC.getValue().clear();

  return constrained > 0;
}


bool FmTriad::fullyConstrained(bool fixedOnly) const
{
  if (itsBndC.getValue().empty())
    return false;

  size_t nfree = 0;
  int constrained = 0;
  for (int bcode : itsBndC.getValue())
    switch (bcode)
      {
      case FIXED:
        constrained++;
        break;
      case PRESCRIBED:
        if (!fixedOnly)
          constrained++;
        break;
      case FREE:
        nfree++;
        break;
      default:
        break;
      }

  if (nfree == itsBndC.getValue().size())
    const_cast<FmTriad*>(this)->itsBndC.getValue().clear();

  return constrained == this->getNDOFs();
}


bool FmTriad::hasLoad(int DOF) const
{
  if (!this->isLegalDOF(DOF))
    return false;

  if (myLoads[DOF].isNull())
    return false;

  return (myLoads[DOF]->getEngine() || myLoads[DOF]->getInitLoad() != 0.0);
}


bool FmTriad::hasAddMass() const
{
  if (itsMass.getValue().empty())
    return false;

  for (double mass : itsMass.getValue())
    if (mass != 0.0) return true;

  const_cast<FmTriad*>(this)->itsMass.getValue().clear();
  return false;
}

double FmTriad::getAddMass(int DOF) const
{
  const DoubleVec& mass = itsMass.getValue();
  if (DOF < 0 && mass.size() > 2)
    return (mass[0] + mass[1] + mass[2]) / 3.0;
  else if (DOF >= 0 && DOF < static_cast<int>(mass.size()))
    return mass[DOF];
  else
    return 0.0;
}

void FmTriad::setAddMass(int DOF, double mass)
{
  int nMass = SIZEMASS(itsNDOFs.getValue());
  if (DOF >= 0 && DOF < nMass)
  {
    itsMass.getValue().resize(nMass,0.0);
    itsMass.getValue()[DOF] = mass;
  }

  this->updateDisplayDetails();
}


void FmTriad::setAddedMass(double mass)
{
  int nMass = SIZEMASS(itsNDOFs.getValue());
  itsMass.getValue().resize(nMass,0.0);

  for (int dof = 0; dof < 3 && dof < nMass; dof++)
    itsMass.getValue()[dof] = mass;

  this->updateDisplayDetails();
}


bool FmTriad::hasInitVel() const
{
  if (initVel.getValue().empty())
    return false;

  for (double ivel : initVel.getValue())
    if (ivel != 0.0) return true;

  const_cast<FmTriad*>(this)->initVel.getValue().clear();
  return false;
}


void FmTriad::setInitVel(int DOF, double var)
{
  if (DOF >= 0 && DOF < itsNDOFs.getValue())
  {
    initVel.getValue().resize(itsNDOFs.getValue(),0.0);
    initVel.getValue()[DOF] = var;
  }

  this->updateDisplayDetails();
}


bool FmTriad::hasInitAcc() const
{
  if (initAcc.getValue().empty())
    return false;

  for (double iacc : initAcc.getValue())
    if (iacc != 0.0) return true;

  const_cast<FmTriad*>(this)->initAcc.getValue().clear();
  return false;
}


void FmTriad::setInitAcc(int DOF, double var)
{
  if (DOF >= 0 && DOF < itsNDOFs.getValue())
  {
    initAcc.getValue().resize(itsNDOFs.getValue(),0.0);
    initAcc.getValue()[DOF] = var;
  }

  this->updateDisplayDetails();
}


/*!
  Use this method to insert the Triad into the main book-keeping ring,
  and update the FE connection of the Triad.
  If parent is supplied (must be an FmLink) it will be added to myAttachedLinks.
  If parent is an FmPart, the local coordinate system of the triad is updated to
  be relative to this part, if it is the first part to be attached to the triad.
*/

bool FmTriad::connect(FmBase* parent)
{
  bool status = this->mainConnect();

  if (parent)
    if (parent->isOfType(FmLink::getClassTypeID()))
      myAttachedLinks.push_back(static_cast<FmLink*>(parent));

  // Coordinate system conversion - from global to local.
  // Do it only when connecting to the first part.
  FmPart* owner = this->getOwnerPart();
  if (parent && owner)
    this->setLocalCS(owner->getGlobalCS().inverse() *
                     this->FmIsPositionedBase::getGlobalCS());

  this->updateFENodeAndDofs(owner);
  return status;
}


/*!
  Internal method used only from connect and initAfterResolve.
  Sets up the allowed DOFs for the Triad and makes sure the associated
  FE node is labelled as external.
*/

bool FmTriad::updateFENodeAndDofs(FmPart* ownerPart)
{
  if (!ownerPart)
    return false;

  // Set nDOFs to zero for grounded triads
  if (ownerPart->isEarthLink())
    return this->setNDOFs(0);

  // Set nDOFs to 6 for generic part triads
  if (ownerPart->useGenericProperties.getValue())
    return this->setNDOFs(6);

  // For triads on FE parts, find the FE-node number of DOFs
#ifdef FT_USE_CONNECTORS
  FFlConnectorItems* ci = &(itsConnectorItems.getValue());
#else
  FFlConnectorItems* ci = NULL;
#endif
  FFlNode* tmpNode = ownerPart->getNodeAtPoint(this->getLocalCS().translation(),
                                               FmDB::getPositionTolerance(),ci);

  // If no node => the FE data is most likely not loaded, don't touch anything.
  if (!tmpNode) return false;

  if (tmpNode->isSlaveNode())
  {
    // This should normally not happen, only if the part is locked such that
    // an attachable node could not be created over the dependent node
    ListUI <<"ERROR: Cannot connect "<< this->getIdString()
           <<" to "<< ownerPart->getIdString()
           <<" because it matches a dependent node in that FE part.\n";
    this->disconnect();
    return false;
  }

  FENodeNo.setValue(tmpNode->getID());
  if (tmpNode->setExternal(true))
    ownerPart->delayedCheckSumUpdate();

  return this->setNDOFs(tmpNode->getMaxDOFs());
}


void FmTriad::initAfterResolve()
{
  this->FmHasDOFsBase::initAfterResolve();

  // Replace references to beam elements from this triad
  // by references to the triad from the beam elements.
  // This is used when converting R7.0 models to R7.1.
  int nBeams = 0;
  FaMat34 globCS;
  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);

  for (FmLink* link : links)
  {
    FmBeam* beam = dynamic_cast<FmBeam*>(link);
    if (!beam) continue;

    if (!beam->getFirstTriad())
      beam->setTriad(this,0);
    else if (!beam->getSecondTriad())
      beam->setTriad(this,1);
    else
      continue;

    myAttachedLinks.removePtr(link);
    if (++nBeams == 1)
    {
      // If this was the first beam this triad is connected to,
      // its coordinate system was local to the beam/part coordinate system.
      // We must therefore transform it to global system here.
      if (FmAssemblyBase* parent = beam->getPositionedAssembly(); parent)
        globCS = parent->toGlobal(beam->myCS.getValue())*this->getLocalCS();
      else
        globCS = beam->myCS.getValue()*this->getLocalCS();
    }
  }
  if (nBeams > 0)
  {
    if (FmPart* owner = this->getOwnerPart(0); owner)
      // Convert to local CS w.r.t. owner part
      this->setLocalCS(owner->getGlobalCS().inverse()*globCS);
    else
      this->setGlobalCS(globCS);
  }

  // Move additional BCs on dependent triads over to the joint DOFs
  // (this can only happen when reading pre R5.1 model files)
  if (this->hasConstraints())
    if (FmJointBase* joint = this->getJointWhereSlave(); joint)
    {
      for (size_t i = 0; i < itsBndC.getValue().size(); i++)
        switch (itsBndC.getValue()[i]) {
        case FIXED:
          joint->setStatusForDOF(i,FIXED);
          break;
        case FREE_DYNAMICS:
          if (joint->getStatusOfDOF(i) == SPRING_CONSTRAINED)
            joint->setStatusForDOF(i,SPRING_DYNAMICS);
          else if (joint->getStatusOfDOF(i) == FREE)
            joint->setStatusForDOF(i,FREE_DYNAMICS);
        default:
          break;
        }
      itsBndC.getValue().clear();
    }

#ifdef FT_USE_CONNECTORS
  // Clear the connector type field if the connector geometry field
  // also is empty (to minimize the model file size)
  if (itsConnectorGeometry.getValue().empty())
    itsConnectorType.setValue(NONE);
#endif

  this->updateFENodeAndDofs(this->getOwnerPart(0));

  if (FmDB::getModelFileVer() <= FFaVersionNumber(7,3,0,11))
    // The definition of triad DOF loads are changed in R7.3
    // to refer to the System direction of the triad.
    // This is to retain backward compatibility for older models.
    for (int d = 0; d < itsNDOFs.getValue(); d++)
      if (FmDofLoad* load = this->getLoadAtDOF(d); load)
      {
        FFaString lDesc = load->getUserDescription();
        if (lDesc.empty())
          load->setUserDescription("#LocalAxis");
        else if (!lDesc.hasSubString("#LocalAx"))
          load->setUserDescription(lDesc + " #LocalAxis");
      }
}


/*!
  Syncronizes the FE node reference of the triad
*/

int FmTriad::syncOnFEmodel(bool useDialog)
{
  FmPart* owner = this->getOwnerFEPart();
  if (!owner) return -1;

#ifdef FT_USE_CONNECTORS
  bool haveGeometry = !itsConnectorGeometry.getValue().empty();
  FFlConnectorItems* items = &(itsConnectorItems.getValue());
#else
  FFlConnectorItems* items = NULL;
#endif

  // Find an FE node at the triad's location
  FFlNode* node = owner->getNodeAtPoint(this->getLocalTranslation(),
                                        FmDB::getPositionTolerance(),items);

  // If it was a dependent node, consider as no node
  if (node && node->isSlaveNode())
    node = NULL;

#ifdef FT_USE_CONNECTORS
  // Unless the same as the connector used, we need to recreate the connector
  // or remove it if the user prefers that option instead
  // (the latter is now enforced if useDialog is false)
  if (node && haveGeometry && FENodeNo.getValue() != node->getID())
  {
    std::string msg = this->getIdString() + " was connected to FE node " +
      std::to_string(FENodeNo.getValue()) + "\nbut now matches node " +
      std::to_string(node->getID()) + " in the new FE model.";

    if (!useDialog)
    {
      FFaMsg::list("\nWarning: " + msg +
                   "\nThe Triad will be attached to the new node"
                   " while removing the surface connector.\n",true);
      haveGeometry = false;
    }
    else if (FFaMsg::dialog(msg +
                            "\n\nDo you want to connect to this node instead?"
                            "\nThe existing connector will then be removed.",
                            FFaMsg::YES_NO))
      haveGeometry = false;

    if (haveGeometry)
      node = NULL;
    else
    {
      itsConnectorType.setValue(NONE);
      itsConnectorGeometry.reset();
    }
  }

  if (!node && haveGeometry)
  {
    // Recreate connector
    if (this->updateConnector(itsConnectorType.getValue(),owner))
      owner->delayedCheckSumUpdate();

    node = owner->getNodeAtPoint(this->getLocalTranslation(),
                                 FmDB::getPositionTolerance(),items);
  }
#endif

  // Set triads FE node status
  int nodeNo = -1;
  if (node)
  {
    node->setExternal(true);
    nodeNo = node->getID();
    if (!this->setNDOFs(node->getMaxDOFs()) &&
        FENodeNo.getValue() == nodeNo)
      return nodeNo;
  }

  FENodeNo.setValue(nodeNo);
  this->onChanged();

  return nodeNo;
}


bool FmTriad::disconnect()
{
  FmPart* owner = this->getOwnerFEPart();
  if (owner)
  {
#ifdef FT_USE_CONNECTORS
    // Remove the spider connector, if any
    if (this->updateConnector(NONE,owner))
    {
      owner->delayedCheckSumUpdate();
      FFaCompoundGeometry& connector = itsConnectorGeometry.getValue();
#ifdef USE_INVENTOR
      // Remove connector geometry highlighting, if any
      for (size_t i = 0; i < connector.size(); i++)
        if (connector[i]->getAddExclude())
          FdExtraGraphics::highlight(connector[i],owner->getGlobalCS(),false);
#endif
      owner->updateConnectorVisualization();
      connector.deleteGeometry();
    }
#endif

    // Remove the FE node connectivity
    if (FFlNode* tmpNode = owner->getNode(FENodeNo.getValue()); tmpNode)
      if (tmpNode->setExternal(false))
        owner->delayedCheckSumUpdate();

    FENodeNo.setValue(-1);
  }

  this->mainDisconnect();
  myAttachedLinks.clear();

  // Coordinate system conversion - from local to global.
  // This has to be done _after_ disconnecting the owner part (TT #2990).
  if (owner)
  {
    this->setGlobalCS(owner->getGlobalCS() * this->getLocalCS());
    // Redraw generic part spider (if any) after removal of this triad
    owner->updateGPVisualization();
  }

  return true;
}


bool FmTriad::detach(FmLink* fromThisOnly, bool notFromDisabledPart)
{
  if (fromThisOnly && myAttachedLinks.size() > 1)
  {
    // Only detach it from the specified part, don't touch coordinate systems
    myAttachedLinks.removePtr(fromThisOnly);
    return true;
  }
  else if (myAttachedLinks.empty())
  {
    ListUI <<"  -> Error: "<< this->getIdString() <<" is already detached.\n";
    return false;
  }

  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);

  if (notFromDisabledPart)
    for (FmLink* link : links)
      if (link->isDisabled())
      {
        ListUI <<"  -> Error: "<< this->getIdString() <<" is not detached"
               <<" from "<< link->getIdString()
               <<"\n            because that part is currently disabled.\n";
        return false;
      }

  // Lambda function doing the actual detach operation for a Triad.
  auto&& reConnect = [](FmTriad* triad)
  {
    triad->disconnect();
    triad->connect();
    if (triad->setNDOFs(6)) // in case it was grounded (TT #3009)
      triad->onChanged(); // to udate the Triad icon in the Objects list
    triad->updateTopologyInViewer();
  };

  reConnect(this);

  // fedem-gui Issue #104: Update spider when detaching from Generic Part(s)
  for (FmLink* link : links)
    link->updateGPVisualization();

  if (this->isMasterTriad())
  {
    std::vector<FmJointBase*> joints;
    this->getJointBinding(joints);
    for (FmJointBase* joint : joints)
    {
      // Update the other independent joint triads
      std::vector<FmTriad*> triads;
      joint->getMasterTriads(triads);
      for (FmTriad* triad : triads)
        if (triad != this)
          reConnect(triad);

      // Update the dependent joint triad
      joint->getSlaveTriad()->updateTopologyInViewer();
    }
  }

  return true;
}


std::string FmTriad::getLinkIDString(bool objPrefix) const
{
  if (FmLink* owner = this->getOwnerLink(0); owner)
    return owner->getLinkIDString(objPrefix);

  return std::string("n/a");
}


/*!
  Returns true if the triad symbol is to be displayed. This is:

  - the triad is not a joint triad (because in that case
    its visualization is part of the joint visualization itself)
  - it is not a beam triad or beam triad visualization is enabled
*/

bool FmTriad::showSymbol() const
{
  if (this->isMasterTriad() || this->isSlaveTriad())
    return false;

  // Check if the triad is connected to at least one beam element
  // and no links of other type
  if (this->hasBeamBinding() && myAttachedLinks.empty())
    return FmDB::getActiveViewSettings()->visibleBeamTriads();

  return true;
}


/*!
  Returns true if the triad represents something that requires its directions
  to be visualized. This is:

  - the triad is not on a link
  - the triad has important directions
  - the triad description contains "#ShowDir"
*/

bool FmTriad::showDirections()
{
  if (myAttachedLinks.empty() && !this->hasElementBinding())
    return true;

  if (this->importantDirections())
    return true;

  return FFaString(this->getUserDescription()).hasSubString("#ShowDir");
}


/*!
  Returns true if the triad represents something that is referring to
  its directions. This is either of the following:

  - the triad is in a joint
  - the triad has measurements attached (simple sensor only)
  - the triad has boundary conditions or component loads
  - the triad has additional masses/moments of inertia
  - the triad has initial velocity or acceleration
*/

bool FmTriad::importantDirections()
{
  if (hasJointBinding())
  {
    if (isSlaveTriad(true))
      return true;
    else if (isMasterTriad(true))
      return true;
  }
  if (getSimpleSensor()) return true;
  if (hasConstraints())  return true;

  for (int i = 0; i < MAX_DOF; i++)
    if (this->hasLoad(i))
      return true;

  if (hasAddMass()) return true;
  if (hasInitVel()) return true;
  if (hasInitAcc()) return true;

  // Every second blade triad is assumed to receive forces
  // from AeroDyn. Therefore their directions are important.
  if (dynamic_cast<FmBlade*>(this->getParentAssembly()))
    return this->getID()%2 == 0;

  return false;
}


bool FmTriad::hasReferences() const
{
  if (hasElementBinding())return true;
  if (hasSpringBinding()) return true;
  if (hasDamperBinding()) return true;
  if (hasLoadBinding())   return true;
  if (hasJointBinding())  return true;
  if (hasSensors())       return true;
  if (hasCurveSets())     return true;
  if (hasAddMass())       return true;

  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);
  for (FmLink* link : links)
    if (link->isGenericPart() || link->isDisabled())
      return true;

  return false;
}


/*!
  Deletes all the joints that this triad is a member of.
  If this triad is an independent line joint triad, the joint is
  deleted only if it is less than tree such triads left in the joint.
*/

bool FmTriad::removeJointBinding()
{
  std::vector<FmJointBase*> allJoints;
  this->getJointBinding(allJoints);

  for (FmJointBase* joint : allJoints)
    if (joint->isOfType(FmSMJointBase::getClassTypeID()))
      joint->erase();
    else if (joint->isOfType(FmMMJointBase::getClassTypeID()))
    {
      std::vector<FmTriad*> triads;
      joint->getMasterTriads(triads);
      if (triads.size() < 3)
        joint->erase();
      else for (FmTriad* triad : triads)
        triad->updateTopologyInViewer();
    }

  return true;
}


bool FmTriad::setAsSlave(FmJointBase* jnt)
{
  return jnt->setAsSlaveTriad(this);
}


FmJointBase* FmTriad::getJointWhereSlave() const
{
  std::vector<FmJointBase*> joints;
  this->getReferringObjs(joints,"itsSlaveTriad");

  for (FmJointBase* joint : joints)
    if (!joint->isContactElement())
      if (!joint->isGlobalSpringElement())
        return joint;

  return NULL;
}


/*!
  Check if this triad is attached to the specified link,
  when exceptForThis is \e false. Otherwise, check if this triad is attached
  to any link except the specified one.
*/

bool FmTriad::isAttached(const FmLink* link, bool exceptForThis) const
{
  if (const FmPart* part = dynamic_cast<const FmPart*>(link);
      part && myAttachedLinks.hasPtr(part))
    return !exceptForThis || myAttachedLinks.size() > 1;

  std::vector<FmLink*> elms;
  this->getElementBinding(elms);
  if (std::find(elms.begin(),elms.end(),link) != elms.end())
    return !exceptForThis || elms.size() > 1;

  return false;
}


/*!
  Check if this triad is attached to a link. If \a ignoreGPandEarth is \e true,
  only FE parts are considered. If \a allowMultipleLinks is \e false,
  this method returns \e true only when the triad is attached to one single link
  and \e false if it is attached to more than one or not at all.
  If \a allowMultipleLinks is \e true, this method returns \e true no matter
  how many links the triad is attached to.
*/

bool FmTriad::isAttached(bool ignoreGPandEarth, bool allowMultipleLinks) const
{
  if (!ignoreGPandEarth && this->hasElementBinding())
  {
    if (allowMultipleLinks)
      return true;

    std::vector<FmLink*> elms;
    this->getElementBinding(elms);
    if (elms.size() == 1)
      return true;
  }

  std::vector<FmLink*> links;
  myAttachedLinks.getPtrs(links);
  if (links.empty()) return false;

  if (!ignoreGPandEarth)
    return links.size() == 1 || allowMultipleLinks;

  int nLinks = 0;
  for (FmLink* link : links)
    if (!link->isGenericPart() && !link->isEarthLink())
      if (++nLinks > 1 && !allowMultipleLinks)
        return false;

  return nLinks > 0;
}


bool FmTriad::isSlaveTriad(bool realSlavesOnly) const
{
  std::vector<FmJointBase*> joints;
  this->getReferringObjs(joints,"itsSlaveTriad");

  for (FmJointBase* joint : joints)
    if (!realSlavesOnly)
      return true;
    else if (!joint->isContactElement())
      if (!joint->isGlobalSpringElement())
        return true;

  return false;
}


bool FmTriad::isMasterTriad(bool realMastersOnly) const
{
  std::vector<Fm1DMaster*> lines;
  std::vector<FmJointBase*> joints;
  this->getReferringObjs(lines,"myTriads");
  for (Fm1DMaster* line : lines)
    line->getReferringObjs(joints,"myMaster");
  this->getReferringObjs(joints,"itsMasterTriad");

  for (FmJointBase* joint : joints)
    if (!realMastersOnly)
      return true;
    else if (!joint->isContactElement())
      if (!joint->isGlobalSpringElement())
        return true;

  return false;
}


bool FmTriad::hasOnlyFreeJoints() const
{
  std::vector<FmJointBase*> joints;
  this->getReferringObjs(joints,"itsMasterTriad");
  this->getReferringObjs(joints,"itsSlaveTriad");

  if (joints.empty()) return false;

  for (FmJointBase* joint : joints)
    if (!joint->isOfType(FmFreeJoint::getClassTypeID()))
      return false;

  return true;
}


bool FmTriad::isMultiMaster(bool includingCam) const
{
  std::vector<Fm1DMaster*> lines;
  this->getReferringObjs(lines,"myTriads");
  if (includingCam)
  {
    for (Fm1DMaster* line : lines)
      if (FmMMJointBase* joint; line->hasReferringObjs(joint,"myMaster"))
        return true;
  }
  else
  {
    std::vector<FmMMJointBase*> joints;
    for (Fm1DMaster* line : lines)
      line->getReferringObjs(joints,"myMaster");
    for (FmMMJointBase* joint : joints)
      if (!joint->isOfType(FmCamJoint::getClassTypeID()))
        return true;
  }

  return false;
}


bool FmTriad::isInLinJoint() const
{
  if (FmMMJointBase* joint; this->hasReferringObjs(joint,"itsSlaveTriad"))
    return !joint->isOfType(FmCamJoint::getClassTypeID());

  return this->isMultiMaster(false);
}


bool FmTriad::hasJointBinding() const
{
  Fm1DMaster* line = NULL;
  FmJointBase* jnt = NULL;
  if (this->hasReferringObjs(line,"myTriads")) return true;
  if (this->hasReferringObjs(jnt,"itsMasterTriad")) return true;
  if (this->hasReferringObjs(jnt,"itsSlaveTriad")) return true;
  return false;
}

void FmTriad::getJointBinding(std::vector<FmJointBase*>& jnts) const
{
  jnts.clear();
  std::vector<Fm1DMaster*> lines;
  this->getReferringObjs(lines,"myTriads");
  for (Fm1DMaster* line : lines)
    line->getReferringObjs(jnts,"myMaster");
  this->getReferringObjs(jnts,"itsMasterTriad");
  this->getReferringObjs(jnts,"itsSlaveTriad");
}


bool FmTriad::hasBeamBinding() const
{
  FmBeam* beam = NULL;
  return this->hasReferringObjs(beam);
}

void FmTriad::getBeamBinding(std::vector<FmBeam*>& beams) const
{
  beams.clear();
  this->getReferringObjs(beams);
}


bool FmTriad::hasElementBinding() const
{
  FmLink* elm = NULL;
  return this->hasReferringObjs(elm,"myTriads");
}

void FmTriad::getElementBinding(std::vector<FmLink*>& elms) const
{
  elms.clear();
  this->getReferringObjs(elms,"myTriads");
}


bool FmTriad::hasSpringBinding() const
{
  FmAxialSpring* spr = NULL;
  return this->hasReferringObjs(spr);
}

void FmTriad::getSpringBinding(std::vector<FmAxialSpring*>& spr) const
{
  spr.clear();
  this->getReferringObjs(spr);
}


bool FmTriad::hasDamperBinding() const
{
  FmAxialDamper* dmp = NULL;
  return this->hasReferringObjs(dmp);
}

void FmTriad::getDamperBinding(std::vector<FmAxialDamper*>& dmp) const
{
  dmp.clear();
  this->getReferringObjs(dmp);
}


void FmTriad::getMotionBinding(std::vector<FmDofMotion*>& motions) const
{
  motions.clear();
  for (int i = 0; i < this->getNDOFs(); i++)
    if (this->getStatusOfDOF(i) == PRESCRIBED)
      motions.push_back(myMotions[i].getPointer());
}


bool FmTriad::hasLoadBinding() const
{
  FmLoad* load = NULL;
  return this->hasReferringObjs(load);
}

void FmTriad::getLoadBinding(std::vector<FmLoad*>& loads) const
{
  loads.clear();
  this->getReferringObjs(loads);
}


void FmTriad::getLoadBinding(std::vector<FmDofLoad*>& loads) const
{
  loads.clear();
  for (int i = 0; i < this->getNDOFs(); i++)
    if (this->getStatusOfDOF(i) == FREE ||
        this->getStatusOfDOF(i) == FREE_DYNAMICS)
      loads.push_back(myLoads[i].getPointer());
}


bool FmTriad::hasTireBinding() const
{
  if (FmJointBase* joint = this->getJointWhereSlave(); joint)
    if (FmTire* tire; joint->hasReferringObjs(tire,"bearingJoint"))
      return true;

  return false;
}


void FmTriad::getTireBinding(std::vector<FmTire*>& tires) const
{
  tires.clear();
  if (FmJointBase* joint = this->getJointWhereSlave(); joint)
    joint->getReferringObjs(tires,"bearingJoint");
}


void FmTriad::getEntities(std::vector<FmSensorChoice>& choicesToFill, int)
{
  choicesToFill = {
    itsEntityTable[FmIsMeasuredBase::POS],
    itsEntityTable[FmIsMeasuredBase::LOCAL_VEL],
    itsEntityTable[FmIsMeasuredBase::GLOBAL_VEL],
    itsEntityTable[FmIsMeasuredBase::LOCAL_ACC],
    itsEntityTable[FmIsMeasuredBase::GLOBAL_ACC],
    itsEntityTable[FmIsMeasuredBase::LOCAL_FORCE],
    itsEntityTable[FmIsMeasuredBase::GLOBAL_FORCE]
  };

  if (FmTurbine* wt = FmDB::getTurbineObject(); wt && this->isPartOf(wt))
    choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::WIND_SPEED]);

  // Check if we a wave function is used
  FmSeaState* sea = FmDB::getSeaStateObject(false);
  if (!sea || sea->waveFunction.isNull()) return;

  // Check if this triad is below the MSL,
  // only in that case it can meassure fluid particle motions
  FaVec3 pos = sea->getLocalCS().inverse()*this->getGlobalTranslation();
  if (pos[VZ] > 0.0) return;

  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::FLUID_VEL]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::FLUID_ACC]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::DYN_PRESS]);
}


void FmTriad::getDofs(std::vector<FmSensorChoice>& choicesToFill)
{
  choicesToFill = {
    itsDofTable[FmIsMeasuredBase::X_TRANS],
    itsDofTable[FmIsMeasuredBase::Y_TRANS],
    itsDofTable[FmIsMeasuredBase::Z_TRANS],
    itsDofTable[FmIsMeasuredBase::X_ROT],
    itsDofTable[FmIsMeasuredBase::Y_ROT],
    itsDofTable[FmIsMeasuredBase::Z_ROT]
  };
}


FaVec3 FmTriad::getGlobalTranslation() const
{
  return this->getGlobalCS().translation();
}


FaVec3 FmTriad::getLocalTranslation(const FmLink* link) const
{
  return this->getRelativeCS(link).translation();
}


FaMat34 FmTriad::getGlobalCS() const
{
  if (FmPart* owner = this->getOwnerPart(0); owner)
    return owner->getGlobalCS() * this->getLocalCS();

  return this->FmIsPositionedBase::getGlobalCS();
}


/*!
  Get local coordinate system of triad, relative to given link.
*/

FaMat34 FmTriad::getRelativeCS(const FmLink* link) const
{
  const FmPart* part = dynamic_cast<const FmPart*>(link);
  if (!link || (part && part == this->getOwnerPart(0)))
    return this->getLocalCS();

  return link->getGlobalCS().inverse() * this->getGlobalCS();
}


/*!
  Sets the position of this triad to be aligned with the provided matrix,
  taking into account the part it could be attached to.

  If moveRelationsAlong == true, the triads in the joints this triad is a member
  of is moved as well, if the joints constraint system demands it.

  This method assumes that it is not called on a triad (with relations)
  that is not movable.
*/

void FmTriad::setGlobalCS(const FaMat34& globalMat, bool moveRelationsAlong)
{
  FaMat34 oldGlobalMat = this->getGlobalCS();

  if (FmPart* owner = this->getOwnerPart(0); owner)
    this->setLocalCS(owner->getGlobalCS().inverse() * globalMat);
  else
    this->FmIsPositionedBase::setGlobalCS(globalMat);

  if (moveRelationsAlong)
  {
    std::vector<FmSMJointBase*> joints;
    this->getReferringObjs(joints,"itsSlaveTriad");
    this->getReferringObjs(joints,"itsMasterTriad");

    // Correct the joints that are not supposed to move when moving this triad.
    // TODO: Also move the MovedAlong joints and their other triad if
    // we want to open the origin tab UI a bit more.
    for (FmSMJointBase* joint : joints)
      if (joint->isMasterTriad(this) && !joint->isMasterMovedAlong())
      {
        joint->setGlobalCS(oldGlobalMat*joint->getLocalCS());
        joint->updateDisplayTopology();
      }
  }
}


/*!
  Reimplemented to update the position of the connected stickers, if any,
  and to update the location data of the joint(s) using this triad.
*/

void FmTriad::setLocalCS(const FaMat34& localMat)
{
  this->FmIsPositionedBase::setLocalCS(localMat);

  std::vector<FmJointBase*> joints;
  this->getReferringObjs(joints,"itsMasterTriad");
  for (FmJointBase* joint : joints)
    joint->updateLocation();

  std::vector<FmSticker*> stickers;
  this->getLocalStickers(stickers);
  for (FmSticker* sticker : stickers)
  {
    sticker->placeAtPoint(this->getGlobalTranslation());
    sticker->draw();
  }
}


int FmTriad::getNDOFs(bool checkForSuppressedOwner) const
{
  int nDOFs = itsNDOFs.getValue();
  if (nDOFs > 0 && checkForSuppressedOwner)
  {
    std::vector<FmLink*> links;
    myAttachedLinks.getPtrs(links);
    for (FmLink* link : links)
      if (link->isSuppressed())
        return 0;
  }
  return nDOFs;
}


bool FmTriad::setNDOFs(int nDOFs)
{
#ifdef FM_DEBUG
  std::cout <<"FmTriad::setNDOFs: "<< this->getIdString()
            <<" nDOFS = "<< nDOFs << std::endl;
#endif
  if (nDOFs != 0 && nDOFs != 3 && nDOFs != 6)
    return false;
  else if (!itsNDOFs.setValue(nDOFs))
    return false;
  else if (nDOFs == 3)
    ListUI <<"  -> Warning: "<< this->getIdString(true)
           <<" is attached to a FE node that has translational DOFs only.\n"
           <<"     Beware that this triad will have no stiffness against rotation.\n";

  if (!initVel.getValue().empty()) initVel.getValue().resize(nDOFs,0);
  if (!initAcc.getValue().empty()) initAcc.getValue().resize(nDOFs,0);
  if (!itsMass.getValue().empty()) itsMass.getValue().resize(SIZEMASS(nDOFs),0);
  if (!itsBndC.getValue().empty()) itsBndC.getValue().resize(nDOFs,FREE);

  return true;
}


bool FmTriad::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmTriad::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmTriad::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmTriad* copyObj = static_cast<FmTriad*>(obj);

  if (!copyObj->myAttachedLinks.empty()) {
    std::vector<FmLink*> links;
    copyObj->myAttachedLinks.getPtrs(links,true);
    this->mainDisconnect();
    myAttachedLinks.clear();
    this->mainConnect();
    myAttachedLinks.setPtrs(links);
  }

  for (int i = 0; i < MAX_DOF; i++) {
    this->setLoadAtDOF  (i,copyObj->getLoadAtDOF(i)  , true);
    this->setMotionAtDOF(i,copyObj->getMotionAtDOF(i), true);
  }

  if (depth == FmBase::DEEP_REPLACE) {
    copyObj->releaseReferencesToMe("myTriads", this);
    copyObj->releaseReferencesToMe("itsSlaveTriad", this);
    copyObj->releaseReferencesToMe("itsMasterTriad", this);
    copyObj->releaseReferencesToMe("itsTriads", this);
    copyObj->releaseReferencesToMe("itsOwnerTriad", this);
  }

  return true;
}


void FmTriad::updateChildrenDisplayTopology()
{
#ifdef USE_INVENTOR
  std::vector<FmLink*> links;
  this->getElementBinding(links);
  for (FmLink* link : links)
    // Need to check spesifically for beams
    // to ensure visualization is created with updated triads
    if (FmBeam* beam = dynamic_cast<FmBeam*>(link); beam)
    {
      // Update the other end-triad of the connected beam,
      // which is not this triad
      if (FmTriad* other = beam->getOtherTriad(this); other)
        other->updateThisTopologyOnly();
      beam->drawObject();
    }
    else
      link->updateThisTopologyOnly();

  std::vector<FmJointBase*> jnts;
  getJointBinding(jnts);
  for (FmJointBase* joint : jnts)
    joint->updateTopologyInViewer();

  std::vector<FmAxialSpring*> spr;
  getSpringBinding(spr);
  for (FmAxialSpring* spring : spr)
    spring->updateTopologyInViewer();

  std::vector<FmAxialDamper*> dmp;
  getDamperBinding(dmp);
  for (FmAxialDamper* damper : dmp)
    damper->updateTopologyInViewer();

  std::vector<FmTire*> tires;
  getTireBinding(tires);
  for (FmTire* tire : tires)
    tire->updateTopologyInViewer();

  myAttachedLinks.getPtrs(links);
  for (FmLink* link : links)
    link->updateGPVisualization();

  FmHasDOFsBase::updateChildrenDisplayTopology();
#endif
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmTriad::writeFMF(std::ostream& os)
{
  os <<"TRIAD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


static bool localParse(const char* keyWord, std::istream& is, FmTriad* obj)
{
  // To account for spelling error in R4.2 model files
  if (strcmp(keyWord,"CONNETOR_TYPE") == 0)
    return FmTriad::parentParse("CONNECTOR_TYPE",is,obj);

  // Conversion of some pre R5.1 keywords
  if (strcmp(keyWord,"GL_VEL") == 0)
    return FmTriad::parentParse("INIT_VELOCITY",is,obj);
  else if (strcmp(keyWord,"GL_ACC") == 0)
    return FmTriad::parentParse("INIT_ACCELERATION",is,obj);

  return FmTriad::parentParse(keyWord,is,obj);
}


bool FmTriad::readAndConnect(std::istream& is, std::ostream&)
{
  FmTriad* obj = new FmTriad();

  // Obsolete fields
  FFaObsoleteField<double>  geoTol;
  FFaObsoleteField<BoolVec> oldBndC;
  FFA_OBSOLETE_FIELD_INIT(geoTol, 0.0, "CONNECTOR_GEOMETRY_TOLERANCE", obj);
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(oldBndC, "ADD_BND", obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      ::localParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("CONNECTOR_GEOMETRY_TOLERANCE", obj);
  FFA_OBSOLETE_FIELD_REMOVE("ADD_BND", obj);

#ifdef FT_USE_CONNECTORS
  // Update from old model file
  if (geoTol.wasOnFile())
    obj->itsConnectorGeometry.getValue().setTolerance(geoTol.getValue());
#endif

  FFaString tDesc = obj->getUserDescription();
  if (oldBndC.wasOnFile())
  {
    DOFStatus newDS = tDesc.hasSubString("#DynBC") ? FIXED : FREE_DYNAMICS;
    for (size_t dof = 0; dof < oldBndC.getValue().size(); dof++)
      if (oldBndC.getValue()[dof]) obj->setStatusForDOF(dof,newDS);
  }

  if (tDesc.hasSubString("#SysDir"))
    obj->itsLocalDir.setValue((LocalDirection)tDesc.getIntAfter("#SysDir"));

  if (tDesc.hasSubString("#InitTransVel"))
  {
    double vel[3];
    tDesc.getDoublesAfter("#InitTransVel",3,vel);
    for (int i = 0; i < 3; i++) obj->setInitVel(i,vel[i]);
  }
  if (tDesc.hasSubString("#InitRotVel"))
  {
    double vel[3];
    tDesc.getDoublesAfter("#InitRotVel",3,vel);
    for (int i = 0; i < 3; i++) obj->setInitVel(3+i,vel[i]);
  }

  // Owner link with ID = -1 means the earth link.
  // The reference to it has to be resolved manually here,
  // because the earth link is not member of the main link ring.
  for (size_t i = 0; i < obj->myAttachedLinks.size(); i++)
    if (obj->myAttachedLinks[i].getRefID() == -1)
      obj->myAttachedLinks[i] = FmDB::getEarthLink();

  obj->connect();
  return true;
}


int FmTriad::checkTriads()
{
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads);
  int numTriads = triads.size();
  int errCount = 0;

  for (FmTriad* activeTriad : triads)
  {
    // Check that the triad is dependent in one joint only
    int numDep = 0;
    std::vector<FmJointBase*> joints;
    activeTriad->getReferringObjs(joints,"itsSlaveTriad");
    for (FmJointBase* joint : joints)
      if (!joint->isContactElement() &&!joint->isGlobalSpringElement())
        numDep++;

    if (numDep > 1)
    {
      errCount++;
      ListUI <<"ERROR: "<< activeTriad->getIdString()
	     <<" is dependent in more than one joint.\n";
    }

    // Check that dependent triads do not have explicit constraints
    else if (numDep == 1 && activeTriad->hasConstraints())
    {
      errCount++;
      ListUI <<"ERROR: "<< activeTriad->getIdString()
	     <<" is dependent but has explicit constraints.\n";
    }
    else if (errCount == 0)
    {
      // Bugfix #380: Ensure DOF motion objects exist for prescribed DOFs
      int nDOFs = activeTriad->getNDOFs(true);
      for (int dof = 0; dof < nDOFs; dof++)
        if (activeTriad->getStatusOfDOF(dof) == PRESCRIBED)
          activeTriad->getMotionAtDOF(dof,true);
    }

    // Fully constrained triads do not need to be attached
    if (activeTriad->fullyConstrained())
    {
      if (activeTriad->hasAddMass())
	ListUI <<"WARNING: Additional mass on fully constrained "
	       << activeTriad->getIdString() <<" has no effect.\n";
      if (activeTriad->fullyConstrained(true))
        numTriads--; // decrement the number of triads with free dofs
      continue; // this triad is either entirely fixed or prescribed --> OK
    }

    // Check that the triad is attached to something
    FmLink* ownerLink = activeTriad->getOwnerLink(0);
    if (!ownerLink)
    {
      if (activeTriad->isMasterTriad())
	continue; // independent joint triad gets contributions from its joint
      else if (activeTriad->hasTireBinding())
	continue; // this is a spindel triad in a link-less Tire --> OK
      else if (activeTriad->hasAddMass() ||
	       activeTriad->hasElementBinding() ||
	       activeTriad->hasSpringBinding() ||
	       activeTriad->hasDamperBinding())
      {
	// This is not (neccesarily) an error condition, but give warning
	ListUI <<"WARNING: "<< activeTriad->getIdString()
	       <<" is not attached to a part.\n";
	continue;
      }

      errCount++;
      ListUI <<"ERROR: "<< activeTriad->getIdString()
	     <<" is not attached to a part or element, and has neither\n"
	     <<"             additional mass, springs, dampers nor a tire"
	     <<" coupled to it.\n";
    }
    else if (ownerLink->isEarthLink())
      if (activeTriad->hasSensors())
      {
	errCount++;
	ListUI <<"ERROR: "<< activeTriad->getIdString()
	       <<" is attached to ground, but has sensors coupled to it.\n";
      }
  }

  // Check for free dofs in the mechanism.
  // Free dofs exist if #triads > 2 x #rigJnt || not all rigJnts on earth link.

  if (numTriads > 2*FmDB::getObjectCount(FmRigidJoint::getClassTypeID()))
    return errCount;

  // Check if there are generalized DOFs on any of the FE parts
  std::vector<FmPart*> parts;
  FmDB::getFEParts(parts);
  for (FmPart* part : parts)
    if (part->nGenModes.getValue() > 0)
      return errCount;

  // Check if there are rigid joints that are not on earth link
  std::vector<FmRigidJoint*> joints;
  FmDB::getAllRigidJoints(joints);
  for (FmRigidJoint* joint : joints)
    if (!joint->getMasterLink()->isEarthLink() &&
        !joint->getSlaveLink()->isEarthLink())
      return errCount;

  errCount++;
  ListUI <<"MECHANISM TOPOLOGY ERROR: No free DOFs on system level.\n";
  return errCount;
}


int FmTriad::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&TRIAD\n");
  this->printID(fp);
  int dof, nDOFs = this->getNDOFs(true);
  LocalDirection sysDir = this->itsLocalDir.getValue();
  fprintf(fp,"  nDOFs = %d\n",nDOFs);
  if (sysDir > GLOBAL)
    fprintf(fp,"  sysDir = %d\n", sysDir);

  // Position matrix
  FaMat34 ur = this->getGlobalCS();
  fprintf(fp,"  ur  =%18.9e%18.9e%18.9e%18.9e\n",
          ur[0][0],ur[1][0],ur[2][0],ur[3][0]);
  fprintf(fp,"       %18.9e%18.9e%18.9e%18.9e\n",
          ur[0][1],ur[1][1],ur[2][1],ur[3][1]);
  fprintf(fp,"       %18.9e%18.9e%18.9e%18.9e\n",
          ur[0][2],ur[1][2],ur[2][2],ur[3][2]);

  if (nDOFs > 0)
  {
    if (this->hasInitVel())
    {
      // Initial velocity
      fprintf(fp,"  urd =%18.9e",this->getInitVel(0));
      for (dof = 1; dof < nDOFs; dof++)
        fprintf(fp,"%18.9e",this->getInitVel(dof));
      fprintf(fp,"\n");
    }
    else
    {
      const double* v0 = NULL;
      double linkVel[3];
      if (FmLink* triadOwner = this->getOwnerLink(); triadOwner)
      {
        // Beta feature: Initial translational velocity on link level
        FFaString lDesc = triadOwner->getUserDescription();
        if (lDesc.getDoublesAfter("#InitTransVel",3,linkVel) > 0)
          v0 = linkVel;
      }
      if (!v0)
      {
        // Global initial velocity that should apply to all triads
        // that don't have their own initial velocity
        const FaVec3& globVel = FmDB::getMechanismObject()->initVel.getValue();
        if (!globVel.isZero())
          v0 = globVel.getPt();
      }
      if (v0)
      {
        // Initial velocity on link or global level
        fprintf(fp,"  urd =%18.9e%18.9e%18.9e",v0[0],v0[1],v0[2]);
        for (dof = 3; dof < nDOFs; dof++) fprintf(fp,"%18.9e",0.0);
        fprintf(fp,"\n");
      }
    }

    if (this->hasInitAcc())
    {
      // Initial acceleration
      fprintf(fp,"  urdd=");
      for (dof = 0; dof < nDOFs; dof++)
        fprintf(fp,"%18.9e",this->getInitAcc(dof));
      fprintf(fp,"\n");
    }

    // Beta feature: Parameters for distributed drag calculations
    FFaString tDesc = this->getUserDescription();
    double dragParams[9];
    if (tDesc.getDoublesAfter("#DragTX",3,dragParams)   |
        tDesc.getDoublesAfter("#DragTY",3,dragParams+3) |
        tDesc.getDoublesAfter("#DragTZ",3,dragParams+6))
    {
      fprintf(fp,"  dragParams =");
      for (int i = 0; i < 9; i++)
        if (i%3 == 0 && i > 0)
          fprintf(fp,"\n              %17.9e",dragParams[i]);
        else
          fprintf(fp,"%17.9e",dragParams[i]);
      fprintf(fp,"\n");
    }

    if (this->hasConstraints(true))
    {
      // Additional BCs for static equilibrium and eigenvalue analysis
      fprintf(fp,"  BC =");
      for (dof = 0; dof < nDOFs; dof++)
        fprintf(fp," %d", this->getStatusCode(dof));
      fprintf(fp,"\n");
    }

    // Beta feature: Output of position matrices for specified triads
    if (tDesc.hasSubString("#savePos"))
      fprintf(fp,"  savePos = 1\n");

    // Variables to be saved:
    // 1 - Global velocity
    // 2 - Global acceleration
    // 3 - Global forces
    // 4 - Local velocity
    // 5 - Local acceleration
    // 6 - Local forces
    // 7 - Global deformations
    this->writeSaveVar(fp,7);
  }

  fprintf(fp,"/\n\n");
  return 0;
}


void FmTriad::printLocalPos(FILE* fp, const FmLink* link, int triadID,
                            bool endOfRecord) const
{
  // Fetch the position matrix of this triad, relative to the local coordinate
  // system of the specified link. If triadID is non-zero, assume it is the
  // internal center of gravity triad of the specified link (a generic part).
  FaMat34 CS = triadID ? link->getPositionCG(false) : this->getRelativeCS(link);

  fprintf(fp,"&TRIAD_UNDPOS\n");
  fprintf(fp,"  supElId = %d\n", link->getBaseID());
  fprintf(fp,"  triadId = %d\n", triadID ? triadID : this->getBaseID());

  // Notice that it is the transpose of the orientation matrix CS[0:2][0:2]
  // that is written here, since it is only this transpose that is used in
  // the solver when calculating the deformational rotation increment
  // (see eq. (4.17) in the R7.3 theory guide, internal version)
  fprintf(fp,"  undPosInSupElSystem =%17.9e%18.9e%18.9e%18.9e\n",
          CS[0][0],CS[0][1],CS[0][2],CS[3][0]);
  fprintf(fp,"                       %17.9e%18.9e%18.9e%18.9e\n",
          CS[1][0],CS[1][1],CS[1][2],CS[3][1]);
  fprintf(fp,"                       %17.9e%18.9e%18.9e%18.9e\n",
          CS[2][0],CS[2][1],CS[2][2],CS[3][2]);

  if (endOfRecord) fprintf(fp,"/\n");
}


int FmTriad::printAdditionalMass(FILE* fp)
{
  if (this->getNDOFs(true) < 1)
    return 0; // Ignore masses on triads attached to ground

  fprintf(fp,"&MASS\n");
  this->printID(fp,false);
  fprintf(fp,"  triadId = %d",this->getBaseID());

  // Beta feature: Added mass and direction-dependent mass
  FFaString tDesc = this->getUserDescription();
  FaVec3 mass(this->getAddMass(),0.0,0.0);
  int mDof = 0;
  if (tDesc.hasSubString("#AddedMass"))
  {
    FaVec3 dir;
    tDesc.getDoublesAfter("#AddedMass",3,dir.getPt());
    mass = mass[0] * dir;
    mDof = -2;
  }
  else if (tDesc.hasSubString("#MassDir"))
  {
    FaVec3 dir;
    tDesc.getDoublesAfter("#MassDir",3,dir.getPt());
    mass = mass[0] * dir.normalize();
    mDof = -1;
  }
  else if (tDesc.hasSubString("#MassX"))
    mDof = 1;
  else if (tDesc.hasSubString("#MassY"))
    mDof = 2;
  else if (tDesc.hasSubString("#MassZ"))
    mDof = 3;

  if (mDof) fprintf(fp,"\n  dof = %d",mDof);
  if (mDof == -2) fprintf(fp,"\n  addedMass = .true.");

  // Beta feature: Mass scaling engine
  int massEngine = tDesc.getIntAfter("#MassScaleEngine");
  if (massEngine > 0)
  {
    FmModelMemberBase* obj = FmDB::findObject(massEngine);
    if (!obj)
      massEngine = -massEngine;
    else if (!obj->isOfType(FmEngine::getClassTypeID()))
      massEngine = -massEngine;
    if (massEngine < 0)
      ListUI <<"\n---> WARNING: #MassScaleEngine "<< -massEngine
             <<" ignored for "<< this->getIdString()
             <<".\n     No Engine with this base ID.\n";
  }

  if (massEngine <= 0 && !myMassEngine.isNull())
    massEngine = myMassEngine->getBaseID();

  if (massEngine > 0)
  {
    fprintf(fp,"\n  mass1 =%17.9e", mass[0]);
    if (mDof < 0)
      fprintf(fp,"%18.9e%18.9e", mass[1], mass[2]);
    fprintf(fp,"\n  massEngineId = %d", massEngine);
    if (itsNDOFs.getValue() == 6)
    {
      fprintf(fp,"\n  II1   =%17.9e%18.9e%18.9e", this->getAddMass(3), 0.0, 0.0);
      fprintf(fp,"\n         %17.9e%18.9e%18.9e", 0.0, this->getAddMass(4), 0.0);
      fprintf(fp,"\n         %17.9e%18.9e%18.9e", 0.0, 0.0, this->getAddMass(5));
      fprintf(fp,"\n  IIengineId = %d",massEngine);
    }
    FmEngine::betaFeatureEngines.insert(massEngine);
  }
  else
  {
    fprintf(fp,"\n  mass0 =%17.9e", mass[0]);
    if (mDof < 0)
      fprintf(fp,"%18.9e%18.9e\n", mass[1], mass[2]);
    if (itsNDOFs.getValue() == 6)
    {
      fprintf(fp,"\n  II0   =%17.9e%18.9e%18.9e", this->getAddMass(3), 0.0, 0.0);
      fprintf(fp,"\n         %17.9e%18.9e%18.9e", 0.0, this->getAddMass(4), 0.0);
      fprintf(fp,"\n         %17.9e%18.9e%18.9e", 0.0, 0.0, this->getAddMass(5));
    }
  }

  fprintf(fp,"\n/\n\n");
  return 0;
}


/*!
  This method is used in the Origin tab and the Align CS commands
  to find out if it is possible to translate the triad.
  If it is attached to an FE part, in a line joint, or in a point
  joint and its movability is connected to an "owning" joint,
  then it is not allowed to translate it.
*/

bool FmTriad::isTranslatable(const FmJointBase* jointToIgnore) const
{
  if (this->isAttached(true))
    return false;

  if (this->isInLinJoint())
    return false;

  std::vector<FmSMJointBase*> joints;
  this->getReferringObjs(joints,"itsSlaveTriad");
  this->getReferringObjs(joints,"itsMasterTriad");

  // Check if this triad is coupled to move along with any of
  // the point joints it is a member of
  for (FmSMJointBase* joint : joints)
    if (joint != jointToIgnore)
    {
      if (joint->isSlaveTriad(this) && joint->isSlaveMovedAlong())
        return false;
      else if (joint->isMasterTriad(this) && joint->isMasterMovedAlong())
        return false;
    }

  return true;
}


/*!
  This method is used in the Origin tab and the Align CS commands
  to find out if it is possible to rotate the triad.
  If it is either the dependent triad in a point joint,
  or an independent joint triad and its movability is connected to an
  "owning" joint, then it is not allowed to rotate it.
  If it is an independent triad of a prismatic or cylindric joint,
  it is only allowed to rotate about the local Z-axis of the joint.
*/

char FmTriad::isRotatable(const FmJointBase* jointToIgnore) const
{
  // Check if this is the dependent triad in a point joint
  if (FmJointBase* joint = this->getJointWhereSlave(); joint)
    if (joint->isOfType(FmSMJointBase::getClassTypeID()))
      if (joint != jointToIgnore) return false;

  // Check if this is an independent triad of a point joint and
  // is coupled to move along with any of the joints it is a member of
  std::vector<FmSMJointBase*> joints;
  this->getReferringObjs(joints,"itsMasterTriad");
  for (FmSMJointBase* joint : joints)
    if (joint->isMasterMovedAlong())
      if (joint != jointToIgnore) return false;

  // Check if this is an independent triad of a prismatic or cylindric joint
  // with its orientation defined by EulerZYX angles
  if (!this->isMultiMaster(false))
    return true;
  else if (myLocation.getValue().getRotType() != FFa3DLocation::EUL_Z_Y_X)
    return false;

  // Check that the object defining the reference coordinate system of the
  // orientation angles is one of the prismatic/cylindric joints using this
  std::vector<FmMMJointBase*> jnts;
  std::vector<Fm1DMaster*> lines;
  this->getReferringObjs(lines,"myTriads");
  for (Fm1DMaster* line : lines)
    line->getReferringObjs(jnts,"myMaster");
  for (FmMMJointBase* joint : jnts)
    if (myRotRef.getPointer() == joint)
      return 3; // can rotate about the local Z-axis of the owning joint

  return false;
}


/*!
  This method returns the list view icon to be used in the Objects browser.
  The icon is selected based on the most dominant DOF status in the triad.
  Joint triads are not checked further for DOF status.
  Neither are triads attached to FE parts without a matching FE node.
  They get the exclamation mark nevertheless.
*/

const char** FmTriad::getListViewPixmap() const
{
  if (this->isAttached(true) && FENodeNo.getValue() == -1)
    return exclamation_xpm;
  else if (this->getNDOFs(true) == 0)
    return triad_fixed_xpm;
  else if (this->isSlaveTriad())
    return triad_slave_xpm;
  else if (this->isMasterTriad())
    return triad_master_xpm;

  int nFixed = 0;
  int nPresc = 0;
  int nLoads = 0;
  for (int i = 0; i < this->getNDOFs(); i++)
    switch (this->getStatusOfDOF(i)) {
    case FREE:
      if (!myLoads[i].isNull()) nLoads++;
      break;
    case FREE_DYNAMICS:
      if (!myLoads[i].isNull()) nLoads++;
    case FIXED:
      nFixed++;
      break;
    case PRESCRIBED:
      nPresc++;
    default:
      break;
    }

  if (nFixed > nPresc && nFixed > nLoads)
    return triad_fixed_xpm;
  else if (nPresc > nLoads)
    return triad_prescribed_xpm;
  else if (nLoads > 0)
    return this->hasAddMass() ? triad_massLoad_xpm : triad_load_xpm;
  else if (this->hasAddMass())
    return triad_mass_xpm;

  return NULL;
}


/*!
  This method is used to get all triads along a generated beamstring.
  Its main purpose is for easy generation of force- and moment diagrams.
*/

int FmTriad::traverseBeam(FmBase* start, std::vector<FmIsPlottedBase*>& objs)
{
  FmTriad* triad = dynamic_cast<FmTriad*>(start);
  if (!triad) return 0;

  // Check if triad is the end of a beam element
  std::vector<FmBeam*> beams;
  triad->getBeamBinding(beams);

  FmBeam* beam = beams.size() == 1 ? beams.front() : NULL;
  for (size_t i = 0; i < beams.size() && !beam; i++)
    if (beams[i]->getUserDescription().find("#Start") != std::string::npos)
      beam = beams[i];

  if (!beam) return 0;

  int nBeamElm = beam->traverse(triad,objs);
  if (nBeamElm > 0)
  {
    // Check if the beamstring is interrupted by point joints.
    // If so, continue the traversal on "the other side" of it,
    // by invoking this method recursively.
    triad = static_cast<FmTriad*>(objs.back());
    if (FmSMJointBase* jnt; triad->hasReferringObjs(jnt,"itsMasterTriad"))
      return nBeamElm + traverseBeam(jnt->getSlaveTriad(),objs);
    else if (triad->hasReferringObjs(jnt,"itsSlaveTriad"))
      return nBeamElm + traverseBeam(jnt->getItsMasterTriad(),objs);
  }

  return nBeamElm;
}


/*!
  This method is used to create a triad at a specified node,
  when creating a system-level beam model from an FE part.
*/

FmTriad* FmTriad::createAtNode(FFlNode* node, FmBase* parent,
                               int IDoffset, int& nTriad)
{
  if (!node) return NULL;

  FmBase* triad = FmDB::findID(FmTriad::getClassTypeID(),
                               IDoffset+node->getID(), {parent->getID()});
  if (triad) return static_cast<FmTriad*>(triad);

  FmTriad* newTriad = new FmTriad(node->getPos());
  newTriad->setParentAssembly(parent);
  newTriad->setID(IDoffset+node->getID());
  newTriad->connect();

  int digit = 2;
  int status = -node->getStatus(-128);
  for (int dof = 0; dof < 6 && status > 0; dof++, digit *= 2)
    if (status%digit > 0)
    {
      newTriad->setStatusForDOF(dof,FIXED);
      status -= digit/2;
    }

  ++nTriad;
  return newTriad;
}
