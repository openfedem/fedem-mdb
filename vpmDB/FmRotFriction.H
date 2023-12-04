// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ROT_FRICTION_H
#define FM_ROT_FRICTION_H

#include "vpmDB/FmFrictionBase.H"


class FmRotFriction : public FmFrictionBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRotFriction();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Rotational friction"; }
  virtual void getParameters(std::vector<FmParameter>& retArray) const;

protected:
  virtual ~FmRotFriction() {}

  virtual const char* getFrictionFsiName() const;
  virtual void getTypeDepVars(std::vector<double>& vars) const;

  virtual bool cloneLocal(FmBase* obj, int depth);

  M_MOD_PARAMS(Radius,FmRotFriction);
};

#endif
