// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RESULT_BASE_H
#define FM_RESULT_BASE_H

#include "vpmDB/FmModelMemberBase.H"


class FmResultBase : public FmModelMemberBase
{
  Fmd_DB_HEADER_INIT();

protected:
  FmResultBase();
  virtual ~FmResultBase() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmResultBase* obj);

  virtual void mainConnectedEvent();

  FmResultBase* copy(int cloneDepth) const;

public:
  virtual bool connect() { return this->eraseOrConnect(); }
  virtual FmBase* clone(int depth) const { return this->copy(depth); }
  using FmBase::clone;
};

#endif
