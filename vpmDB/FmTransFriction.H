// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_TRANS_FRICTION_H
#define FM_TRANS_FRICTION_H

#include "vpmDB/FmFrictionBase.H"


class FmTransFriction : public FmFrictionBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmTransFriction();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Translational friction"; }
  virtual void getParameters(std::vector<FmParameter>& retArray) const;

protected:
  virtual ~FmTransFriction() {}

  virtual const char* getFrictionFsiName() const { return "TRANS_FRICTION"; }

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
