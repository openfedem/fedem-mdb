// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/Fm1DMaster.H"
#include "vpmDB/FmMMJointBase.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(Fc1DMASTER, Fm1DMaster, FmIsRenderedBase);


Fm1DMaster::Fm1DMaster()
{
  Fmd_CONSTRUCTOR_INIT(Fm1DMaster);

  FFA_REFERENCELIST_FIELD_INIT(myTriadsField, myTriads, "TRIADS");
}


Fm1DMaster::~Fm1DMaster()
{
  this->disconnect();
}


bool Fm1DMaster::eraseOptions()
{
  // Release the existing triads and erase those without other references
  this->releaseTriads(true);

  return this->FmIsRenderedBase::eraseOptions();
}


bool Fm1DMaster::detach()
{
  // Detaching the first triad detaches all of them
  if (this->isAttached(true))
    if (this->getFirstTriad()->detach())
      return this->draw();

  return false;
}


bool Fm1DMaster::addTriad(FmTriad* triad, bool asFront)
{
  return this->insertTriad(triad, asFront ? 0 : myTriads.size());
}


bool Fm1DMaster::insertTriad(FmTriad* triad, size_t pos)
{
  if (!triad) return false;

  if (this->hasTriad(triad))
  {
    ListUI <<"ERROR: "<< triad->getIdString(true)
           <<" is is already member of "<< this->getIdString() <<".\n";
    return false;
  }

  std::vector<FmJointBase*> joints;
  triad->getReferringObjs(joints,"itsSlaveTriad");
  for (FmJointBase* joint : joints)
  {
    if (joint->isGlobalSpringElement())
      continue;
    else if (joint->isContactElement())
      if (static_cast<FmMMJointBase*>(joint)->getMaster() != this)
        continue;

    ListUI <<"ERROR: "<< triad->getIdString(true)
           <<" is the dependent triad in "<< joint->getIdString(true)
           <<" and can not be used as independent triad here.\n";
    return false;
  }

  if (pos == 0)
    myTriads.push_front(triad);
  else if (pos < myTriads.size())
    myTriads.insert(triad,pos);
  else
    myTriads.push_back(triad);

  triad->updateTopologyInViewer();
  triad->onChanged(); // to update triad icon
  return true;
}


/*!
  Sets the triads supplied as independent joint triads.
  Previously defined triads are released first.
*/

void Fm1DMaster::setTriads(const std::vector<int>& triadIDs)
{
  // Release the existing triads
  this->releaseTriads();

  // Insert the new triads
  myTriads.setRefs(triadIDs);
}


/*!
  Sets the triads supplied as independent joint triads.
  Previously defined triads are released first.
*/

bool Fm1DMaster::setTriads(const std::vector<FmTriad*>& triads)
{
  // Release the existing triads
  this->releaseTriads();

  // Insert the new triads
  for (FmTriad* triad : triads)
    if (!this->addTriad(triad,false))
      return false;

  return true;
}


int Fm1DMaster::releaseTriad(FmTriad* triad, FmTriad* replacement)
{
  int indx = 0;
  if (!myTriads.hasPtr(triad,&indx))
    return 0;

  if (replacement)
    myTriads[indx] = replacement;
  else
    myTriads.erase(indx);

  // Update the released triad
  triad->updateTopologyInViewer();
  triad->onChanged();

  return 1+indx;
}


void Fm1DMaster::releaseTriads(bool eraseUnused)
{
  std::vector<FmTriad*> triads;
  this->getTriads(triads);
  myTriads.clear();

  // Update (or erase) the triads being released
  for (FmTriad* triad : triads)
    if (eraseUnused && !triad->hasReferences())
      triad->erase();
    else
    {
      triad->updateTopologyInViewer();
      triad->onChanged();
    }
}


FmTriad* Fm1DMaster::getFirstTriad() const
{
  return myTriads.empty() ? NULL : myTriads.getFirstPtr();
}


FmTriad* Fm1DMaster::getLastTriad() const
{
  return myTriads.empty() ? NULL : myTriads.getLastPtr();
}


FmLink* Fm1DMaster::getOwnerLink() const
{
  return myTriads.empty() ? NULL : myTriads.getFirstPtr()->getOwnerLink(0);
}


FmPart* Fm1DMaster::getOwnerPart() const
{
  return myTriads.empty() ? NULL : myTriads.getFirstPtr()->getOwnerPart(0);
}


bool Fm1DMaster::isAttached(bool allowMultipleLinks) const
{
  if (myTriads.empty()) return false;

  for (size_t i = 0; i < myTriads.size(); i++)
    if (!myTriads[i]->isAttached(false,allowMultipleLinks))
      return false;

  return true;
}


bool Fm1DMaster::localParse(const char* keyWord, std::istream& activeStatement,
                            Fm1DMaster* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool Fm1DMaster::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(Fm1DMaster::getClassTypeID());
}


void Fm1DMaster::initAfterResolve()
{
  this->FmIsRenderedBase::initAfterResolve();

  // Ensure that no triads are listed twice (in case of manual editing)
  for (size_t i = 0; i < myTriads.size(); i++)
    for (size_t j = i+1; j < myTriads.size();)
      if (myTriads.getPtr(i) == myTriads.getPtr(j))
      {
        ListUI <<" ==> WARNING: "<< myTriads[i]->getIdString()
               <<" occurs more than once in "<< this->getIdString()
               <<", removing the second occurance.\n";
        myTriads.erase(j);
      }
      else
        j++;
}


void Fm1DMaster::printForSolver(FILE* fp, const std::vector<FmTriad*>& triads,
                                double loopLength)
{
  fprintf(fp,"&MASTER_CURVE\n");
  this->printID(fp);
  fprintf(fp,"  nTriads = %u\n", (unsigned int)triads.size());
  fprintf(fp,"  triadIds =");

  size_t i = 0;
  for (FmTriad* triad : triads)
    if (++i > 0 && i%8 == 1)
      fprintf(fp,"\n             %d", triad->getBaseID());
    else
      fprintf(fp," %d", triad->getBaseID());

  bool isExtended = false;
  std::vector<FmMMJointBase*> users;
  this->getReferringObjs(users);
  for (FmMMJointBase* joint : users)
    if (joint->getUserDescription().find("#Extended") != std::string::npos)
      isExtended = true;
  if (isExtended || this->getUserDescription().find("#Extended") != std::string::npos)
    fprintf(fp,"\n  isExtended = 1\n");

  if (loopLength > 0.0)
    fprintf(fp,"\n  isLooping = 1\n  loopLength =%17.9e", loopLength);

  fprintf(fp,"\n/\n");
}


bool operator==(const Fm1DMaster& lhs, const Fm1DMaster& rhs)
{
  if (lhs.getTypeID() != rhs.getTypeID()) return false;
  if (lhs.myTriads.size() != rhs.myTriads.size()) return false;

  for (size_t i = 0; i < lhs.myTriads.size(); i++)
    if (lhs.myTriads[i].getRefID() != rhs.myTriads[i].getRefID())
      return false;

  return true;
}
