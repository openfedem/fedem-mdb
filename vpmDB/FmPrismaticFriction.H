// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PRISMATIC_FRICTION_H
#define FM_PRISMATIC_FRICTION_H

#include "vpmDB/FmTransFriction.H"


class FmPrismaticFriction : public FmTransFriction
{
  Fmd_DB_HEADER_INIT();

public:
  FmPrismaticFriction();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Prismatic friction"; }
  virtual const char** getPixmap() const;
  virtual void getParameters(std::vector<FmParameter>& retArray) const;

protected:
  virtual ~FmPrismaticFriction() {}

  virtual void getTypeDepVars(std::vector<double>& vars) const;

  M_MOD_PARAMS(Radius  ,FmPrismaticFriction);
  M_MOD_PARAMS(Constant,FmPrismaticFriction);
};

#endif
