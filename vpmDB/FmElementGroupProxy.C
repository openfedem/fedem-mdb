// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmElementGroupProxy.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmDB.H"
#include "FFlLib/FFlNamedPartBase.H"
#include "FFlLib/FFlAttributeBase.H"
#include "FFlLib/FFlTypeInfoSpec.H"
#include "FFaLib/FFaString/FFaStringExt.H"

#include "vpmDB/Icons/FmIconPixmaps.H"
#include "FFaLib/FFaString/FFaParse.H"


Fmd_DB_SOURCE_INIT(FcELEMENT_GROUP, FmElementGroupProxy, FmSimulationModelBase);

FmElementGroupProxy::FmElementGroupProxy()
{
  Fmd_CONSTRUCTOR_INIT(FmElementGroupProxy);

  this->removeField("BASE_ID"); // Ignore the base ID field on read/write

  FFA_REFERENCE_FIELD_INIT(myOwnerField, myOwner, "OWNER_PART");

  FFA_FIELD_DEFAULT_INIT(myTypeName, "TYPE");

  FFA_FIELD_INIT(useFatigueOpt, false, "FATIGUE_CALCULATION");
  FFA_FIELD_INIT(myFatigueSNCurve,  0, "FATIGUE_SN_CURVE");
  FFA_FIELD_INIT(myFatigueSNStd,    0, "FATIGUE_SN_STANDARD");
  FFA_FIELD_INIT(myFatigueSCF,    1.0, "FATIGUE_STRESS_CONCENTRATION_FACTOR");

  myElementGroup = NULL;
  saveGroup = false;
}


FmElementGroupProxy::~FmElementGroupProxy()
{
  this->disconnect();
}


void FmElementGroupProxy::setRealObject(FFlNamedPartBase* group)
{
  myElementGroup = group;
  if (!group) return;

  this->setID(group->getID());
  myTypeName.setValue(group->getTypeInfoSpec()->getTypeName());

  this->FmBase::setUserDescription(this->getUserDescription());
}


/*!
  Reimplemented to only give the local ID relative to the FE part
  this group is defined on. Only used in animation setup.
*/

std::string FmElementGroupProxy::getInfoString() const
{
  return FFaNumStr("[%d] ",this->getID()) + this->getUserDescription();
}


std::string FmElementGroupProxy::getUserDescription(size_t) const
{
  // Append the name on file, if any
  if (myElementGroup)
    if (!myElementGroup->getName().empty())
      return this->getTypeName() + ": " + myElementGroup->getName();

  return this->getTypeName();
}


bool FmElementGroupProxy::setUserDescription(const std::string& descr)
{
  if (!this->FmBase::setUserDescription(descr)) return false;

  if (!myElementGroup) return true;

  // Also update the element group name, but without the initial type name
  const std::string& typeName = this->getTypeName();
  if (typeName == descr)
    myElementGroup->setName("");
  else if (descr.find(typeName+": ") == 0)
    myElementGroup->setName(descr.substr(typeName.size()+2));
  else
    myElementGroup->setName(descr);

  return true;
}


bool FmElementGroupProxy::connect(FmBase* parent)
{
  if (parent)
    if (parent->isOfType(FmPart::getClassTypeID()))
      myOwner = static_cast<FmPart*>(parent);

  // Non-unique user IDs are allowed for element group proxies,
  // since it is inherited from the underlying FE element group
  bool status = this->mainConnect(true);

#ifdef FM_DEBUG
  if (myOwner)
    std::cout <<"FmElementGroupProxy::connect {"<< this->getBaseID()
	      <<"} to "<< myOwner->getIdString() << std::endl;
#endif

  return status;
}


bool FmElementGroupProxy::disconnect()
{
#ifdef FM_DEBUG
  if (myOwner)
    std::cout <<"FmElementGroupProxy::disconnect {"<< this->getBaseID()
	      <<"} from "<< myOwner->getIdString() << std::endl;
#endif

  myOwner = NULL;

  return this->mainDisconnect();
}


const char** FmElementGroupProxy::getListViewPixmap() const
{
  if (!myElementGroup) return NULL;

  int v = myElementGroup->getVisibilityStatus();

  if ( ((v & FFlNamedPartBase::FFL_HAS_VIS_ELM_MASK)    == FFlNamedPartBase::FFL_HAS_VIS_ELM) &&
       ((v & FFlNamedPartBase::FFL_HAS_HIDDEN_ELM_MASK) != FFlNamedPartBase::FFL_HAS_HIDDEN_ELM) )
    return allElemsVisible_xpm;
  else if ( ((v & FFlNamedPartBase::FFL_HAS_VIS_ELM_MASK)    == FFlNamedPartBase::FFL_HAS_VIS_ELM) &&
            ((v & FFlNamedPartBase::FFL_HAS_HIDDEN_ELM_MASK) == FFlNamedPartBase::FFL_HAS_HIDDEN_ELM) )
    return someElemsVisible_xpm;
  else if ( ((v & FFlNamedPartBase::FFL_HAS_VIS_ELM_MASK)    != FFlNamedPartBase::FFL_HAS_VIS_ELM) &&
            ((v & FFlNamedPartBase::FFL_HAS_HIDDEN_ELM_MASK) == FFlNamedPartBase::FFL_HAS_HIDDEN_ELM) )
    return noElemsVisible_xpm;
  else
    return NULL;
}


void FmElementGroupProxy::toggleFatigue(bool onOff)
{
  useFatigueOpt.setValue(onOff);
  saveGroup = true; // Output this group only if it has been touched

  std::vector<int> assID;
  this->getAssemblyID(assID);
  FmBase* gHead = FmDB::getHead(FmElementGroupProxy::getClassTypeID(),assID);
  static_cast<FmRingStart*>(gHead)->printHeader(true);
}


std::ostream& FmElementGroupProxy::writeFMF(std::ostream& os)
{
  if (saveGroup)
  {
    os <<"ELEMENT_GROUP\n{\n";
    this->writeFields(os);
    os <<"}\n\n";
  }
  return os;
}


bool FmElementGroupProxy::readAndConnect(std::istream& is, std::ostream&)
{
  FmElementGroupProxy* obj = new FmElementGroupProxy();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  obj->saveGroup = true;

  std::vector<int> assID;
  obj->getAssemblyID(assID);
  FmBase* gHead = FmDB::getHead(FmElementGroupProxy::getClassTypeID(),assID);
  static_cast<FmRingStart*>(gHead)->printHeader(true);
  return true;
}


std::string FmElementGroupProxy::getIdPath(bool withBrackets) const
{
  if (!myOwner)
    return this->FmSimulationModelBase::getIdPath(withBrackets);

  // Assume here (without checking) that the element group and the owner part
  // belong to the same sub-assembly. Anything else would be inconsistent.
  std::string sId = FFaNumStr("%d,",this->getID()) + myOwner->getIdPath(false);

  if (withBrackets)
    return "[" + sId + "]";
  else
    return sId;
}


std::string FmElementGroupProxy::getGroupId() const
{
  std::string groupId;
  if (myElementGroup)
  {
    // If the group is an FE attribute, add its type name to the ID
    FFlAttributeBase* att = dynamic_cast<FFlAttributeBase*>(myElementGroup);
    if (att) groupId = att->getTypeName() + " ";
    groupId += FFaNumStr(myElementGroup->getID());
  }

  return groupId;
}
