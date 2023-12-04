// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_AXIAL_DAMPER_H
#define FM_AXIAL_DAMPER_H

#include "vpmDB/FmDamperBase.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmAxialDamper : public FmDamperBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmAxialDamper();

  bool connect(FmTriad* tr1, FmTriad* tr2);
  using FmDamperBase::connect;

  void setOwnerTriads(FmTriad* tr1, FmTriad* tr2);
  void removeOwnerTriads();

  FmTriad* getFirstTriad() const;
  FmTriad* getSecondTriad() const;

  static int checkAxialDampers();

  virtual bool detach();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fp);

  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual void getEntities(std::vector<FmSensorChoice>& choicesToFill, int dof = -1);

  virtual const char* getUITypeName() const { return "Axial damper"; }

protected:
  virtual ~FmAxialDamper();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReferenceList<FmTriad>       itsTriads;
  FFaField<FFaReferenceListBase*> itsTriadsField;
};

#endif
