// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMC_OUTPUT_H
#define FMC_OUTPUT_H

#include "vpmDB/FmCtrlElementBase.H"

class FmCtrlLine;
class FmEngine;


class FmcOutput : public FmCtrlElementBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmcOutput();

  virtual bool interactiveErase();

  virtual FmCtrlElementBase* copy();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "CtrlOut"; }
  virtual const char** getListViewPixmap() const;

  virtual int getNumInputPorts() const { return 1; }

  virtual FmCtrlLine* getLine(int portNo = 1) const;
  virtual bool setLine(int portNo, FmCtrlLine* line);

  virtual bool releaseFromPort(FmCtrlLine* line);
  virtual int atWhatPort(const FmCtrlLine* line) const;

  FmEngine* getEngine() const;

protected:
  virtual ~FmcOutput() {}

  virtual void initAfterResolve();
  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FmCtrlLine* itsInput;
};

#endif
