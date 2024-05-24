// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_FUNC_TREE_H
#define FM_FUNC_TREE_H

#include "vpmDB/FmRingStart.H"


class FmFuncStart : public FmRingStart
{
public:
  FmFuncStart(const std::string& uis, const char** pixmap, int fUse);
  virtual ~FmFuncStart() {}

  virtual bool hasRingMembers(bool) const;
  virtual void getModelMembers(std::vector<FmModelMemberBase*>& list) const;

  virtual void displayRingMembers() const {}
  virtual bool eraseRingMembers(bool) { return true; }

  virtual FmRingStart* searchFuncHead(int fUse) const;

private:
  int myFuncUse;
};


class FmFuncTree
{
public:
  FmFuncTree(FmRingStart* root = NULL);
  ~FmFuncTree();

  void setParentAssembly(FmBase* subAss);

  FmRingStart* myHead;

private:
  std::vector<FmRingStart*> myNodes;
};

#endif
