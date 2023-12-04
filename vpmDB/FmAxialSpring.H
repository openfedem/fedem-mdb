// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_AXIAL_SPRING_H
#define FM_AXIAL_SPRING_H

#include "vpmDB/FmSpringBase.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmAxialSpring : public FmSpringBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmAxialSpring();

  bool connect(FmTriad* tr1, FmTriad* tr2);
  using FmSpringBase::connect;

  void setOwnerTriads(FmTriad* tr1, FmTriad* tr2);
  void removeOwnerTriads();

  FmTriad* getFirstTriad() const;
  FmTriad* getSecondTriad() const;
  void getTriads(std::vector<FmTriad*>& toFill) const;

  static int checkAxialSprings();

  virtual double getModelSpringLength() const;

  virtual bool detach();

  virtual int printSolverEntry(FILE* fp);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual void getEntities(std::vector<FmSensorChoice>& choicesToFill, int dof = -1);

  virtual const char* getUITypeName() const { return "Axial spring"; }

protected:
  virtual ~FmAxialSpring();

  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReferenceList<FmTriad>       itsTriads;
  FFaField<FFaReferenceListBase*> itsTriadsField;
};

#endif
