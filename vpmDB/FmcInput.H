// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMC_INPUT_H
#define FMC_INPUT_H

#include "vpmDB/FmCtrlOutputElementBase.H"
#include "vpmDB/FmEngine.H"


class FmcInput : public FmCtrlOutputElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcInput();

  virtual int printSolverEntry(FILE* fp);
  virtual FmCtrlElementBase* copy();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "CtrlIn"; }
  virtual const char** getListViewPixmap() const;

  FmEngine* getEngine() const { return myEngine.getPointer(); }
  void setEngine(FmEngine* e) { myEngine.setRef(e); }

protected:
  virtual ~FmcInput();

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaReference<FmEngine>      myEngine;
  FFaField<FFaReferenceBase*> myEngineField;
};

#endif
