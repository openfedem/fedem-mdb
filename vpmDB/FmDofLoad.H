// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_DOF_LOAD_H
#define FM_DOF_LOAD_H

#include "vpmDB/FmIsControlledBase.H"

class FmHasDOFsBase;


class FmDofLoad : public FmIsControlledBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmDofLoad();

  virtual bool disconnect();

  FmHasDOFsBase* getOwner() const;

  virtual FmModelMemberBase* getActiveOwner() const;

  virtual bool isListable() const { return false; }

  virtual int printSolverEntry(FILE* fp);

  double getInitLoad() const { return myLoadVal.getValue(); }
  bool   setInitLoad(double dm) { myLoadVal.setValue(dm); return true; }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Load"; }

protected:
  virtual ~FmDofLoad();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaField<double> myLoadVal;

public:
  FFaField<bool>   freqDomain;
};

#endif
