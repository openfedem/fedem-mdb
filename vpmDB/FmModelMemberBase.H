// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODEL_MEMBER_BASE_H
#define FM_MODEL_MEMBER_BASE_H

#include <cstdio>
#include <map>

#include "vpmDB/FmBase.H"
#include "FFaLib/FFaDefinitions/FFaListViewItem.H"

class FFaSwitchBoardConnector;


class FmModelMemberBase : public FmBase, public FFaListViewItem
{
  Fmd_DB_HEADER_INIT();

protected:
  virtual void mainConnectedEvent()    {}
  virtual void mainDisconnectedEvent() {}
  virtual void changedEvent()          {}

public:
  enum Signal {
    MODEL_MEMBER_CONNECTED,
    MODEL_MEMBER_DISCONNECTED,
    MODEL_MEMBER_FINISHED_DISCONNECTED,
    MODEL_MEMBER_CHANGED
  };

  void sendSignal(Signal sig);

  static FFaSwitchBoardConnector* getSignalConnector();

  // From FFaListViewItem
  virtual const char* getItemName() const { return this->getUITypeName(); }
  virtual std::string getItemDescr() const { return this->getUserDescription(64); }
  virtual int getItemID() const { return this->getID(); }
  virtual int getItemBaseID() const { return this->getBaseID(); }
  virtual void getItemAssemblyPath(std::vector<int>& assID) const;

protected:
  // Constructor/Destructor
  FmModelMemberBase(bool isDummy = false);
  virtual ~FmModelMemberBase() {}

  // Copying
  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool eraseOptions();

public:
  // From FmBase
  virtual void onMainConnected();
  virtual void onMainDisconnected();
  virtual void onMainAboutToDisconnect();

  void onChanged();

  virtual FmBase* duplicate() const { return NULL; }

  virtual bool interactiveErase() { return this->erase(); }

  virtual int printSolverEntry(FILE*) { return 0; }
  void printID(FILE* myFile, bool withBaseID = true) const;

  int getBaseID() const { return myBaseID.getValue(); }

  static void resolveBaseIDProblems();

  bool setTag(const std::string& tag) { return myTag.setValue(tag); }
  bool isTagged(const std::string& tag) const;
  const std::string& getTag() const { return myTag.getValue(); }

  bool moveTo(FmSubAssembly* newAss);

protected:
  FFaNoCopyField<int> myBaseID;
  FFaField<std::string> myTag;

private:
  void setBaseID(int id) { myBaseID.setValue(id); }

  static int baseIDProblemCount;
  static std::map<FmModelMemberBase*,int> baseIDProblems;

public:
  static bool ignoreBaseID; // set to true when importing sub-assembly
  static bool inInteractiveErase;
};

#endif
