// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RING_START_H
#define FM_RING_START_H

#include "vpmDB/FmModelMemberBase.H"
#include <vector>


class FmRingStart : public FmModelMemberBase
{
  Fmd_HEADER_INIT();

public:
  FmRingStart(const std::string& UIString, const char** pixmap = NULL,
              bool doPrintHeader = false);
  virtual ~FmRingStart() {}

  int getRingMemberType() const { return myRingMemberType; }
  void setRingMemberType(const int aTypeID) { myRingMemberType = aTypeID; }

  const char* getUITypeName() const { return myUIString.c_str(); }
  void setUITypeName(const std::string& name, const char** pixmap = NULL);
  const char* getChildrenUITypeName() const;

  void printHeader(bool doPrint) { myPrintHeader = doPrint; }
  bool printHeader() const { return myPrintHeader; }
  virtual int countRingMembers() const;
  virtual bool hasRingMembers() const;
  virtual void displayRingMembers() const;
  virtual bool eraseRingMembers(bool showProgress = false);

  virtual std::string getItemDescr() const { return myUIString; }
  virtual const char** getListViewPixmap() const { return myPixmap; }
  void setListViewPixmap(const char** pixmap) { myPixmap = pixmap; }

  void setParent(FmRingStart* parent);
  FmRingStart* getParent() const { return myParent; }
  const std::vector<FmRingStart*>& getChildren() const { return myChildren; }

  virtual void getModelMembers(std::vector<FmModelMemberBase*>& list)
  { this->getMembers(list); }

  template<class T> void getMembers(std::vector<T*>& list)
  {
    T* member = NULL;
    for (FmBase* pt = this->getNext(); pt != this; pt = pt->getNext())
      if ((member = dynamic_cast<T*>(pt))) list.push_back(member);
  }

  virtual FmRingStart* searchFuncHead(int funcUse) const;

  virtual bool connect() { return false; }
  virtual bool disconnect() { return false; }

  int getSortNumber() const { return mySortNumber; }

protected:
  void addChild(FmRingStart* child);

  int                  myRingMemberType;
  std::string          myUIString;
  const char**         myPixmap;
  FmRingStart*         myParent;
  std::vector<FmRingStart*> myChildren;

private:
  bool myPrintHeader; //<! Should myUIString be used as a model file header?
  int  mySortNumber;  //<! Number used as a stable order of the head map entries
                      //<! when needed, e.g., for write to model file.
                      //<! The number is incremented for each new object.
};

#endif
