// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * CLASS: FmBase
 * AUTHOR: Jens LIEN
 *
 * DESCRIPTION:
 * Base parent class for the data structure entries.
 * Very little functionality is included in this class.
 ******************************************************************************/

#ifndef FM_BASE_H
#define FM_BASE_H

#include "FFaLib/FFaContainers/FFaFieldContainer.H"
#include "FFaLib/FFaContainers/FFaField.H"
#include "FFaLib/FFaContainers/FFaReference.H"

class FmRingStart;
class FmSubAssembly;


class FmBase : public FFaFieldContainer
{
  Fmd_DB_HEADER_INIT();

public:
  FmBase* getNext() const { return itsNextRingPt; }
  FmBase* getPrev() const { return itsPrevRingPt; }

  // Description text

  virtual std::string getUserDescription(size_t maxLength = 0) const;
  virtual bool setUserDescription(const std::string& descr);

  bool makeCopyDescr();

  virtual std::string getInfoString() const;
  virtual std::string getIdPath(bool withBrackets = true) const;
  std::string getIdString(bool withDescription = false) const;

  // User ID

  int  getID() const { return myID.getValue(); }
  void setID(int Id) { myID.setValue(Id); }

  void getAssemblyID(std::vector<int>& assID) const;
  FmBase* getParentAssembly() const { return myParentAssembly.getPointer(); }
  FmBase* getCommonAncestor(const FmBase* other) const;
  void setParentAssembly(FmBase* subAss);
  void setParentAssembly(int Id, int classType = -1);

  // Reimplemented from FFaFieldContainer

  virtual int  getResolvedID() const { return this->getID(); }
  virtual void getResolvedAssemblyID(std::vector<int>& assID) const;

  // Topology

  bool cloneOrConnect();
  virtual bool connect() { return this->mainConnect(); }
  virtual bool disconnect() { return this->mainDisconnect(); }
  virtual void drawObject() {}

  bool isPartOf(const FmSubAssembly* subAss) const;

  // File IO

  std::ostream& writeFields(std::ostream& os) const;

  virtual std::ostream& writeFMF(std::ostream& os) { return os; }
  virtual bool clone(FmBase*, int) { return false; }
  virtual FmBase* clone(int) const { return NULL; }

  virtual const char* getUITypeName() const;

  virtual void onMainConnected() {}
  virtual void onMainDisconnected() {}
  virtual void onMainAboutToDisconnect() {}

  enum copyModeType {
    NOTHING,
    SHALLOW,
    DEEP_UNRESOLVED,
    DEEP_APPEND,
    DEEP_REPLACE
  };

private:
  FFaField<std::string> myDescription; //<! User-defined description text
  FFaNoCopyField<int>   myID;          //<! User-id, unique only within one type

  FFaReference<FmBase>        myParentAssembly;
  FFaField<FFaReferenceBase*> myParentAssemblyField;

  FmBase* itsNextRingPt; //<! Pointer to the next entry in the main ring
  FmBase* itsPrevRingPt; //<! Pointer to the previous entry in the main ring

protected:
  FmBase(bool isDummy = false);
  virtual ~FmBase();

  bool eraseOrConnect();
  bool mainConnect(bool allowNonUniqueIDs = false);
  bool mainDisconnect();

  void insertAfter(FmBase* afterPt);

  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmBase* obj);
  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
