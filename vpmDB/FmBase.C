// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmBase.H"
#include "vpmDB/FmRingStart.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaCmdLineArg/FFaCmdLineArg.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <cstring>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcBASE, FmBase, FmBase);


FmBase::FmBase(bool isDummy)
{
  Fmd_CONSTRUCTOR_INIT(FmBase);

  itsNextRingPt = itsPrevRingPt = this;

  if (isDummy)
  {
    myID.setValue(0);
    FFA_REFERENCE_INIT(myParentAssembly);
    return; // No fields in dummy objects
  }

  FFA_FIELD_INIT(myID,0,"ID");

  FFA_REFERENCE_FIELD_INIT(myParentAssemblyField, myParentAssembly, "PARENT_ASSEMBLY");
  myParentAssembly.setPrintIfZero(false);

  FFA_FIELD_DEFAULT_INIT(myDescription,"DESCR");
}


FmBase::~FmBase()
{
#ifdef FM_DEBUG
  if (!myFields.empty())
    std::cout <<"Destructing "<< this->getTypeIDName()
              <<" ["<< this->getID() <<"]"<< std::endl;
#endif
}


std::string FmBase::getIdPath(bool withBrackets) const
{
  std::string strId = FFaNumStr(this->getID());

  // Since the user IDs are unique only within a sub-assembly,
  // we need to add the sub-assembly path to the object identification
  std::vector<int> assID;
  this->getAssemblyID(assID);
  // Write the path in bottom-up direction
  for (int i = assID.size(); i > 0;)
    strId += FFaNumStr(withBrackets ? ",%d" : "_%d", assID[--i]);

  return withBrackets ? "[" + strId + "]" : strId;
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmBase::setParentAssembly(int Id, int classType)
{
  if (classType < 0)
    myParentAssembly.setRef(Id,FmSubAssembly::getClassTypeID());
  else
    myParentAssembly.setRef(Id,classType);
}


void FmBase::setParentAssembly(FmBase* subAss)
{
  if (subAss)
    if (subAss->isOfType(FmSubAssembly::getClassTypeID()))
      myParentAssembly.setRef(subAss);
}


bool FmBase::isPartOf(const FmSubAssembly* subAss) const
{
  FmSubAssembly* pAss = dynamic_cast<FmSubAssembly*>(this->getParentAssembly());
  for (;  pAss;  pAss = dynamic_cast<FmSubAssembly*>(pAss->getParentAssembly()))
    if (pAss == subAss)
      return true;

  return false;
}


/*!
  Method to retrieve the assembly ID of this object.
  The "live" path is returned if the model is resolved,
  and the read path if not.

  The assembly path vector is ordered from top to bottom. <Root, sub,... , leaf>
*/

void FmBase::getAssemblyID(std::vector<int>& assID) const
{
  assID.clear();
  myParentAssembly.getRefAssemblyID(assID);
  int parentAssId = myParentAssembly.getRefID();
  if (parentAssId) assID.push_back(parentAssId);
}


/*!
  This method returns a vector of the ID's of the hierarchy of sub-assemblies
  containing this object. The input vector must be empty. It gets the resolved
  "live" assembly ID and returns nothing if the model is not resolved.

  The "path" to this object. MainModel.SubAss.SubAss ... SubAss
*/

void FmBase::getResolvedAssemblyID(std::vector<int>& assID) const
{
  if (!myParentAssembly.isNull())
  {
    myParentAssembly->getResolvedAssemblyID(assID);
    assID.push_back(myParentAssembly->getID());
  }
}


FmBase* FmBase::getCommonAncestor(const FmBase* other) const
{
  // Find the common parent
  FmBase* p2 = NULL;
  FmBase* parent = myParentAssembly.getPointer();
  for (; parent != p2; parent = parent->getParentAssembly())
    for (p2 = other->getParentAssembly(); p2; p2 = p2->getParentAssembly())
      if (p2 == parent)
        return p2;

  return NULL;
}


/*!
  This method should be used in place of connect(bool) for all objects
  that should be ignored when importing an existing model as a sub-assembly.
  Returns \e true if the object is successfully connected
  and \e false if it is erased or the connection failed.
*/

bool FmBase::eraseOrConnect()
{
  if (FmSubAssembly::old2newAssID.second == 0)
    return this->mainConnect();

  // We are importing a regular model as a sub-assembly, ignore this object
  ListUI <<"  -> Ignoring "<< this->getIdString(true) <<"\n";
  return !this->erase();
}


/*!
  This method is used from model file reading, but only for objects that
  might have been created before reading the model file. If the object
  to be connected (*this) already exists in the database, all data is copied
  (cloned) from this object into the found object. This object is then deleted.
*/

bool FmBase::cloneOrConnect()
{
#if FM_DEBUG > 1
  std::cout <<"FmBase::cloneOrConnect() "<< this->getIdString();
  if (this->isOfType(FmModelMemberBase::getClassTypeID()))
    std::cout <<" {"<< static_cast<FmModelMemberBase*>(this)->getBaseID() <<"}";
  std::cout << std::endl;
#endif

  // If we are importing a regular model as a sub-assembly,
  // this object should be ignored
  if (FmSubAssembly::old2newAssID.second == 0)
  {
    if (this->mainConnect())
      return true;

    // The object already exists.
    // This should only happen on the top level, so no need to
    // check for sub-assemly ID here when cloning the object read.
    FmBase* cloneToObj = FmDB::findID(this->getTypeID(),this->getID());
    if (cloneToObj)
    {
      // Clone the new information into the existing object
      cloneToObj->clone(this,FmBase::DEEP_REPLACE);
      if (this->isOfType(FmModelMemberBase::getClassTypeID()))
        static_cast<FmModelMemberBase*>(cloneToObj)->sendSignal(FmModelMemberBase::MODEL_MEMBER_CHANGED);
    }
    else // This should normally not happen
      std::cerr <<"ERROR: "<< this->getIdString() <<" already exists,"
                <<" ignoring the last item read from file."<< std::endl;
  }
  else
    ListUI <<"  -> Ignoring "<< this->getIdString(true) <<"\n";

  return this->erase();
}


bool FmBase::mainConnect(bool allowNonUniqueIDs)
{
  if (itsNextRingPt != this && itsPrevRingPt != this)
    return false; // already connected

  std::vector<int> assID;
  this->getAssemblyID(assID);
  FmBase* hPt = FmDB::getHead(this->getTypeID(),assID,FmSubAssembly::tmpHeadMap);
  if (!hPt) return false; // logic error

  FmBase* afterPt = hPt->itsPrevRingPt;
  if (this->getID() > 0)
  {
    while (afterPt->getID() > this->getID())
      afterPt = afterPt->itsPrevRingPt;
    if (afterPt->getID() == this->getID() && !allowNonUniqueIDs)
    {
      std::cerr <<"WARNING: Connected object already exists: "
                << this->getIdString() << std::endl;
      return false;
    }
  }
  else
  {
    bool reuseUserID = false;
    FFaCmdLineArg::instance()->getValue("reUseUserID",reuseUserID);
    if (reuseUserID)
    {
      // Reuse old user IDs of deleted objects.
      // Find the first "hole" in the user ID sequence for this class type, and
      // reuse that ID for the object to be connected (this is pre R5.1 behaviour)
      afterPt = hPt;
      while (afterPt->itsNextRingPt != hPt &&
             afterPt->itsNextRingPt->getID() - afterPt->getID() == 1)
        afterPt = afterPt->itsNextRingPt;
    }
    this->setID(afterPt->getID() + 1);
  }

  this->insertAfter(afterPt);
  this->onMainConnected();

  return true;
}


void FmBase::insertAfter(FmBase* afterPt)
{
  itsNextRingPt = afterPt->itsNextRingPt;
  itsPrevRingPt = afterPt->itsNextRingPt->itsPrevRingPt;
  afterPt->itsNextRingPt->itsPrevRingPt = this;
  afterPt->itsNextRingPt = this;
}


bool FmBase::mainDisconnect()
{
  if (itsPrevRingPt == this && itsNextRingPt == this)
    return false; // not connected

  this->onMainAboutToDisconnect();

  itsPrevRingPt->itsNextRingPt = itsNextRingPt;
  itsNextRingPt->itsPrevRingPt = itsPrevRingPt;
  itsPrevRingPt = itsNextRingPt = this;

  this->onMainDisconnected();

  return true;
}


std::ostream& FmBase::writeFields(std::ostream& os) const
{
  for (const FieldContainerMap::value_type& field : myFields)
    if (field.second->isPrintable())
      os << *field.first <<" = "<< *field.second <<";\n";

  return os;
}


/*!
  Try to copy all fields of the cloned object into this object.
  \return true if all of them have a matching field in this and false otherwise.
*/

bool FmBase::cloneLocal(FmBase* obj, int depth)
{
  // If clone depth is SHALLOW, only copy data field values and leave the
  // references untouched. Then it is also safe to clone an object of a
  // difference class, only copying the fields they have in common (kmo 070613).
  // If clone depth is DEEP_UNRESOLVED, copy data fields and references,
  // but leave the references unresolved for later resolving (kmo 300815).
  return this->FFaFieldContainer::copy(obj, depth <= SHALLOW,
                                       depth == DEEP_UNRESOLVED);
}


bool FmBase::localParse(const char* keyWord, std::istream& activeStatement,
                        FmBase* obj)
{
  if (!obj->readField(keyWord,activeStatement))
  {
    std::string msg(keyWord);
    msg += " is not a defined fmm-file keyword for ";
    msg += std::string(obj->getUITypeName()) + "s";
    std::map<std::string,int>::iterator it = FmDB::unknownKeywords.find(msg);
    if (it == FmDB::unknownKeywords.end())
      FmDB::unknownKeywords[msg] = 1;
    else
      ++it->second;
  }
  else if (strcmp(keyWord,"PARENT_ASSEMBLY") == 0 && FmSubAssembly::old2newAssID.first > 0)
    if (obj->myParentAssembly.getRefID() == FmSubAssembly::old2newAssID.first)
      obj->myParentAssembly.setRef(FmSubAssembly::old2newAssID.second,
                                   obj->myParentAssembly.getRefTypeID());

  return false;
}


bool FmBase::setUserDescription(const std::string& descr)
{
  if (!myDescription.setValue(descr))
    return false;

  // Erase all instances of the "-character in the description (if any).
  // The model file parser does not cope with their presense (TT #2926).
  size_t i = descr.find('"');
  if (i == std::string::npos)
    return true;

  FFaMsg::dialog("\"-characters are not allowed in the Description field"
                 " and will be removed",FFaMsg::WARNING);

  std::string& newDesc = myDescription.getValue();
  for (; i < newDesc.size(); i = newDesc.find('"',i))
    newDesc.erase(i,1);

  return true;
}


/*!
  Returns the user description of this object.
  If \a maxLength is non-zero, only the first \a maxLength characters are
  returned, or only the first line if that is shorter than \a maxLength.
*/

std::string FmBase::getUserDescription(size_t maxLength) const
{
  const std::string& descr = myDescription.getValue();
  if (!maxLength || descr.empty())
    return descr;

  size_t eol = descr.find('\n');
  if (eol > maxLength) eol = maxLength;
  return descr.substr(0,eol);
}


/*!
  Returns a composed string describing this object.
  Mainly used to refer to this object in pulldown menus, etc.
*/

std::string FmBase::getInfoString() const
{
  return this->getIdPath() + " " + this->getUserDescription()
         + " (" + this->getUITypeName() + ")";
}


/*!
  Returns a composed string describing this object.
  Mainly used to refer to this object in Output List error messages, etc.
*/

std::string FmBase::getIdString(bool withDescription) const
{
  std::string strId(this->getUITypeName());
  strId += " " + this->getIdPath();
  if (withDescription && !myDescription.getValue().empty())
    strId += " \"" + myDescription.getValue() + "\"";

  return strId;
}


const char* FmBase::getUITypeName() const
{
  // Should never arrive here, unless...
  std::cerr <<"WARNING: Requesting GUI type name during destruction of an object.\n"
	    <<"         This may indicate some logic programming error, previously\n"
	    <<"         resulting in \"pure virtual function call\" runtime error.\n"
	    <<"         Set a break point in FmBase::getUITypeName() to trace this."
	    << std::endl;
  return "FmBase";
}


bool FmBase::makeCopyDescr()
{
  if (myDescription.getValue().empty())
    return false;

  return this->setUserDescription("Copy of " + myDescription.getValue());
}
