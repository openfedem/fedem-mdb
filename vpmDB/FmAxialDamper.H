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

  using FmDamperBase::connect;
  bool connect(FmTriad* tr1, FmTriad* tr2);

  void setOwnerTriads(FmTriad* tr1, FmTriad* tr2);
  void removeOwnerTriads();

  FmTriad* getFirstTriad() const;
  FmTriad* getSecondTriad() const;
  void getTriads(std::vector<FmTriad*>& triads) const;

  static int checkAxialDampers();

  virtual bool detach();

  virtual int printSolverEntry(FILE* fp);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual void getEntities(FmSensorChoices& choices, int);

  virtual const char* getUITypeName() const { return "Axial damper"; }

protected:
  virtual ~FmAxialDamper();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReferenceList<FmTriad>       itsTriads;
  FFaField<FFaReferenceListBase*> itsTriadsField;
};

#endif
