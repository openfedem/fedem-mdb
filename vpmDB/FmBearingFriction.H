// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_BEARING_FRICTION_H
#define FM_BEARING_FRICTION_H

#include "vpmDB/FmRotFriction.H"


class FmBearingFriction : public FmRotFriction
{
  Fmd_DB_HEADER_INIT();

public:
  FmBearingFriction();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Bearing friction"; }
  virtual const char** getPixmap() const;
  virtual void getParameters(std::vector<FmParameter>& retArray) const;

protected:
  virtual ~FmBearingFriction() {}

  virtual void getTypeDepVars(std::vector<double>& vars) const;

  M_MOD_PARAMS(Capacity,FmBearingFriction);
  M_MOD_PARAMS(Constant,FmBearingFriction);
};

#endif
