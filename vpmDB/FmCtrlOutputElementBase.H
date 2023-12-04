// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CTRL_OUTPUT_ELEMENT_BASE_H
#define FM_CTRL_OUTPUT_ELEMENT_BASE_H

#include <vector>

#include "vpmDB/FmCtrlElementBase.H"


class FmCtrlLine;


class FmCtrlOutputElementBase : public FmCtrlElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  bool hasCtrlLines() const;
  void getLines(std::vector<FmCtrlLine*>& lines) const;
  virtual void updateLines(bool single);
  virtual int getNumOutputPorts() const { return 1; }

  int renumberLocalVariables(int startID);

protected:
  FmCtrlOutputElementBase();
  virtual ~FmCtrlOutputElementBase();

  void setStateVarCount(int nvar) { itsStateVariables.resize(nvar); }
  void setOutputVarNo(int no);
  void printVariables(FILE* fp);

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmCtrlOutputElementBase* obj);

private:
  std::vector<int> itsStateVariables;
};

#endif
