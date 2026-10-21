// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmModelMemberBase.H
  \brief Base class for all model data objects.
*/

#ifndef FM_MODEL_MEMBER_BASE_H
#define FM_MODEL_MEMBER_BASE_H

#include <cstdio>

#include "vpmDB/FmBase.H"
#include "FFaLib/FFaDefinitions/FFaListViewItem.H"

class FFaSwitchBoardConnector;


/*!
  \brief Base class for all model data objects.
  \details This class adds a base ID field to the mechanism objects.
  This is a unique ID over all objects in the model independent of their type.
  The class also adds a tag field which can be used for grouping the objects.
  The class is also derived from FFaListViewItem in addition to FmBase, with
  functionalities for managing objects in GUI tree views.
*/

class FmModelMemberBase : public FmBase, public FFaListViewItem
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

protected:
  //! \brief Hook for class-dependent tasks when the object is connected.
  virtual void mainConnectedEvent() {}
  //! \brief Hook for class-dependent tasks when the object is disconnected.
  virtual void mainDisconnectedEvent() {}
  //! \brief Hook for class-dependent tasks when the object is updated.
  virtual void changedEvent() {}

public:
  //! \brief Enumerator for database signals.
  enum Signal {
    MODEL_MEMBER_CONNECTED,
    MODEL_MEMBER_DISCONNECTED,
    MODEL_MEMBER_FINISHED_DISCONNECTED,
    MODEL_MEMBER_CHANGED
  };

  //! \brief Broadcasts a database signal to trigger call-back invokations.
  void sendSignal(Signal sig);
  //! \brief Returns the model database signal connector.
  static FFaSwitchBoardConnector* getSignalConnector();

  // From FFaListViewItem
  virtual const char* getItemName() const { return this->getUITypeName(); }
  virtual std::string getItemDescr() const { return this->getUserDescription(64); }
  virtual int getItemID() const { return this->getID(); }
  virtual int getItemBaseID() const { return this->getBaseID(); }
  virtual void getItemAssemblyPath(std::vector<int>& assID) const;

protected:
  //! \brief Default constructor.
  explicit FmModelMemberBase(bool isDummy = false);

  // From FmBase
  virtual bool cloneLocal(FmBase* obj, int depth);

  //! \copybrief FFaFieldContainer::eraseOptions()
  virtual bool eraseOptions();

public:
  // From FmBase
  virtual void onMainConnected();
  virtual void onMainDisconnected();
  virtual void onMainAboutToDisconnect();

  //! \brief Invokes call-backs when this object updated.
  void onChanged();

  //! \brief Returns a connected copy of this object.
  virtual FmBase* duplicate() const { return NULL; }

  //! \brief Hook for class-dependent tasks for interactive deleting an object.
  //! \details This method can be overriden in sub-classes when additional
  //! user feedback is needed before the object can be deleted.
  //! It returns \e false if the user cancelled the delete.
  virtual bool interactiveErase() { return this->erase(); }

  //! \brief Hoook for printing the solver input file entry for this object.
  virtual int printSolverEntry(FILE*) { return 0; }
  //! \brief Prints the ID of this object in the solver input file format.
  void printID(FILE* myFile, bool withBaseID = true) const;

  //! \brief Returns the base ID of this object.
  int getBaseID() const { return myBaseID.getValue(); }

  //! \brief Resolves the conflicting base ID issues for #baseIdProblems.
  static void resolveBaseIDProblems();

  //! \brief Assigns a tag to this object.
  bool setTag(const std::string& tag) { return myTag.setValue(tag); }
  //! \brief Returns \e true if this object has a tag assigned to it.
  bool isTagged(const std::string& tag) const;
  //! \brief Returns a const reference to the tag assigned to this object.
  const std::string& getTag() const { return myTag.getValue(); }

  //! \brief Moves this object to another sub-assembly.
  bool moveTo(FmSubAssembly* newAss);

protected:
  FFaNoCopyField<int> myBaseID; //!< Base ID, unique over all object types
  FFaField<std::string> myTag;  //!< Text string used for grouping objects

private:
  //! \brief Assigns the base ID to this object.
  void setBaseID(int id) { myBaseID.setValue(id); }

  //! Objects with conflicting base ID encountered during model file parsing
  static std::vector<FmModelMemberBase*> baseIdProblems;

public:
  static bool ignoreBaseID; //!< Set to \e true when importing sub-assembly
  static bool inInteractiveErase; //!< Set to \e true when interactive delete
};

#endif
