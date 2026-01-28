// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SM_JOINT_BASE_H
#define FM_SM_JOINT_BASE_H

#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmSMJointBase : public FmJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual bool detach();

  virtual void initAfterResolve();

  virtual bool isMasterAttachedToLink(bool allowMultipleLinks = false) const;
  virtual FmLink* getMasterLink() const;
  virtual FmPart* getMasterPart(bool noEarth = false) const;

  FmTriad* getItsMasterTriad() const { return itsMasterTriad.getPointer(); }
  void setAsMasterTriad(FmTriad* triad) { itsMasterTriad.setRef(triad); }
  void removeItsMasterTriad() { itsMasterTriad.setRef(NULL); }
  bool swapMasterAndSlave();

  virtual void getMasterTriads(std::vector<FmTriad*>& triadsToFill) const;
  virtual bool isMasterTriad(const FmTriad* triad) const;

  virtual FaMat34 getGlobalCS() const;
  virtual void    setGlobalCS(const FaMat34& globalMat, bool moveAlong = false);

  virtual bool isTranslatable() const;
  virtual char isRotatable() const;

  bool isMasterMovedAlong() const { return IAmMovingMasterTriadAlong.getValue(); }
  bool isSlaveMovedAlong()  const { return IAmMovingSlaveTriadAlong.getValue(); }
  bool setMasterMovedAlong(bool onOff) { return IAmMovingMasterTriadAlong.setValue(onOff); }
  bool setSlaveMovedAlong(bool onOff)  { return IAmMovingSlaveTriadAlong.setValue(onOff); }

  virtual FaVec3 getTransJointVariables() const;
  virtual FaVec3 getRotJointVariables() const;
  virtual void   setRotJointVariables(const FaVec3& rotations);

protected:
  FmSMJointBase();
  virtual ~FmSMJointBase();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmSMJointBase* obj);

  virtual bool eraseOptions();

  FFaReference<FmTriad>       itsMasterTriad;
  FFaField<FFaReferenceBase*> itsMasterTriadField;

  FFaField<bool>              IAmMovingMasterTriadAlong;
  FFaField<bool>              IAmMovingSlaveTriadAlong;
};

#endif
