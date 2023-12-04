// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_STRAIGHT_MASTER_H
#define FM_STRAIGHT_MASTER_H

#include "vpmDB/Fm1DMaster.H"


class FmStraightMaster : public Fm1DMaster
{
  Fmd_DB_HEADER_INIT();

public:
  FmStraightMaster();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fd);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();
  virtual const char* getUITypeName() const { return "Straight line"; }

  virtual bool addTriadOnPoint(const FaVec3& globalPoint);
  virtual bool insertTriad(FmTriad* triad, size_t pos);

  virtual double getSliderPosition(FaMat34& ur, const FaVec3& pos) const;

protected:
  virtual ~FmStraightMaster() {}

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
