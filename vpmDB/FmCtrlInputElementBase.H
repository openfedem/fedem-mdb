// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CTRL_INPUT_ELEMENT_BASE_H
#define FM_CTRL_INPUT_ELEMENT_BASE_H

#include "vpmDB/FmCtrlOutputElementBase.H"


class FmCtrlInputElementBase : public FmCtrlOutputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual FmCtrlLine* getLine(int portNo) const;
  virtual bool setLine(int PortNo, FmCtrlLine* line);

  virtual bool releaseFromPort(FmCtrlLine* line);
  virtual int atWhatPort(const FmCtrlLine* line) const;
  virtual int getNumInputPorts() const { return itsInputs.size(); }

private:
  FmCtrlLine* releaseFromPort(int PortNo);

protected:
  FmCtrlInputElementBase();
  virtual ~FmCtrlInputElementBase();

  void printSolverTopology(FILE* fp);
  void setPortCount(int count) { itsInputs.resize(count,NULL); }

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmCtrlInputElementBase* obj);

private:
  std::vector<FmCtrlLine*> itsInputs;
};

#endif
