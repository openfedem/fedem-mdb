// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_LOAD_H
#define FM_LOAD_H

#include "vpmDB/FmIsControlledBase.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaString/FFaEnum.H"


class FmLoad : public FmIsControlledBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmLoad();

  bool connect(FmTriad* tr,
	       FmIsPositionedBase* from, const FaVec3& fromPo,
	       FmIsPositionedBase* to,   const FaVec3& toPo);
  bool connect(FmBase* parent);
  using FmIsControlledBase::connect;
  virtual bool disconnect();

  double getInitLoad() const { return itsLoad.getValue(); }
  bool setInitLoad(double load) { return itsLoad.setValue(load); }

  enum LoadType { FORCE = 0, TORQUE = 1 };

  LoadType getLoadType() const { return itsLoadType.getValue(); }
  bool setLoadType(LoadType type) { return itsLoadType.setValue(type); }

  FmTriad* getOwnerTriad() const { return itsOwnerTriad.getPointer(); }
  void setOwnerTriad(FmTriad* tr) { itsOwnerTriad.setRef(tr); }

  FmIsPositionedBase* getFromRef() const { return itsPos[0].getPointer(); }
  FmIsPositionedBase* getToRef() const   { return itsPos[1].getPointer(); }
  void setFromRef(FmIsPositionedBase* p) { this->setRefPoint(p,0); }
  void setToRef(FmIsPositionedBase* p)   { this->setRefPoint(p,1); }

  void moveAttackPointGlobal(const FaVec3& pos, FmLink* link = NULL);
  void moveAttackPointLocal(const FaVec3& pos, FmLink* link = NULL);

  const FaVec3& getLocalFromPoint() const  { return itsPoint[0].getValue(); }
  const FaVec3& getLocalToPoint() const    { return itsPoint[1].getValue(); }
  void setLocalFromPoint(const FaVec3& pt) { this->setLocalPoint(pt,0); }
  void setLocalToPoint(const FaVec3& pt)   { this->setLocalPoint(pt,1); }

  FaVec3 getGlobalFromPoint() const { return std::move(this->getGlbPoint(0)); }
  FaVec3 getGlobalToPoint()   const { return std::move(this->getGlbPoint(1)); }
  void setGlobalFromPoint(const FaVec3& pt) { this->setGlbPoint(pt,0); }
  void setGlobalToPoint(const FaVec3& pt)   { this->setGlbPoint(pt,1); }

  static void changeAttackPt(FmLink* link, const FaVec3& pt);
  static void changeFromPt(FmIsPositionedBase* p, const FaVec3& pt);
  static void changeToPt(FmIsPositionedBase* p, const FaVec3& pt);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fd);

  virtual bool clone(FmBase* obj, int depth);
  virtual bool detach();

  virtual const char* getUITypeName() const;

private:
  void setRefPoint(FmIsPositionedBase* p, int idx);
  void setLocalPoint(const FaVec3& pt, int idx);
  FaVec3 getGlbPoint(int idx) const;
  void   setGlbPoint(const FaVec3& pt, int idx);

  void moveAttackPoint(const FaVec3& localPos, FmLink* link);

protected:
  virtual ~FmLoad();

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaEnumMapping(LoadType) {
    FFaEnumEntry(FORCE, "FORCE");
    FFaEnumEntry(TORQUE, "TORQUE");
    FFaEnumEntryEnd;
  };

  FFaField<double>                 itsLoad;
  FFaField<LoadTypeEnum>           itsLoadType;
  FFaField<FaVec3>                 itsPoint[2];

  FFaReference<FmTriad>            itsOwnerTriad;
  FFaField<FFaReferenceBase*>      itsOwnerField;

  FFaReference<FmIsPositionedBase> itsPos[2];
  FFaField<FFaReferenceBase*>      itsPosField[2];

public:
  static FmLoad* editedLoad; // Used by load point editing commands
};

#endif
