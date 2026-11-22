// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmRingStart.H
  \brief Class representing the start (and end) of a linked list of objects.
*/

#ifndef FM_RING_START_H
#define FM_RING_START_H

#include "vpmDB/FmModelMemberBase.H"
#include <vector>


/*!
  \brief Class representing the start (and end) of a linked list of objects.
  \details This class is used to define the topology of the model database.
  It defines the start node of a linked list of model entities of the same
  type, with some extra attributes that the list members have in common.
  See the FmDB methods for the usage of this class.
*/

class FmRingStart : public FmModelMemberBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_HEADER_INIT();
  //! \endcond

public:
  //! \brief The constructor initializes the class members.
  FmRingStart(const std::string& uistr, const char** pixmap = NULL,
              bool doPrintHeader = false);

  //! \brief Returns the value of the #myRingMemberType member.
  int getRingMemberType() const { return myRingMemberType; }
  //! \brief Assigns the #myRingMemberType member.
  void setRingMemberType(int typeID) { myRingMemberType = typeID; }

  //! \brief Returns the value of the #myUIString member.
  const char* getUITypeName() const { return myUIString.c_str(); }
  //! \brief Assigns the #myUIString and #myPixmap members.
  void setUITypeName(const std::string& name, const char** pixmap = NULL);
  //! \brief Returns the UI-type name of the children of this ring start object.
  const char* getChildrenUITypeName() const;

  //! \brief Returns the value of the #myPrintHeader member.
  bool printHeader() const { return myPrintHeader; }
  //! \brief Assigns the #myPrintHeader member.
  void printHeader(bool doPrint) { myPrintHeader = doPrint; }

  //! \brief Returns the number of members in the list headed by \a *this.
  virtual int countRingMembers() const;
  //! \brief Returns \a true if this the linked list is not empty.
  virtual bool hasRingMembers(bool noChildren = false) const;
  //! \brief Updates the rendering of all objects in this linked list.
  virtual void displayRingMembers() const;
  //! \brief Erases all members of this linked list making it empty.
  virtual bool eraseRingMembers(bool showProgress = false);

  // From FFaListViewItem
  std::string getItemDescr() const final { return myUIString; }
  const char** getListViewPixmap() const final { return myPixmap; }
  //! \brief Assigns the #myPixmap member.
  void setListViewPixmap(const char** pixmap) { myPixmap = pixmap; }

  //! \brief Assigned the parent linked list for \a *this.
  void setParent(FmRingStart* parent);
  //! \brief Returns the the parent of this linked list.
  FmRingStart* getParent() const { return myParent; }
  //! \brief Returns a const reference to the #myChildren member.
  const std::vector<FmRingStart*>& getChildren() const { return myChildren; }

  //! \brief Returns all members of this linked list.
  virtual void getModelMembers(std::vector<FmModelMemberBase*>& list) const
  { const_cast<FmRingStart*>(this)->getMembers(list); }

  //! \brief Returns all members of class \a T of this linked list.
  template<class T> void getMembers(std::vector<T*>& list)
  {
    for (FmBase* pt = this->getNext(); pt != this; pt = pt->getNext())
      if (T* member = dynamic_cast<T*>(pt); member) list.push_back(member);
  }

  //! \brief Returns the child linked list of functions with usage \a funcUse.
  virtual FmRingStart* searchFuncHead(int funcUse) const;

  //! \brief Overridden to do nothing.
  bool connect() final { return false; }
  //! \brief Overridden to do nothing.
  bool disconnect() final { return false; }

  //! \brief Returns the #mySortNumber member.
  int getSortNumber() const { return mySortNumber; }

protected:
  //! \brief Adds a child linked list for \a *this.
  void addChild(FmRingStart* child);

private:
  std::string  myUIString; //!< The UI type name of the members of this list
  const char** myPixmap;   //!< Pixmap to use in the List views for this list
  FmRingStart* myParent;   //!< Pointer to the parent linked list for \a *this.

  //! List of child linked list associated with this list.
  std::vector<FmRingStart*> myChildren;

protected:
  int myRingMemberType; //!< Class type ID of the objects in this linked list

private:
  bool myPrintHeader; //!< Should #myUIString be used as a model file header?

  //! \brief Number used as a stable order of the head map entries when needed.
  //! \details E.g., for write to model file in a consistent way.
  //! The value is incremented for each new FmRingStart instance.
  int mySortNumber;
};

#endif
