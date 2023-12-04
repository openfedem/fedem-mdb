// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_DOF_MOTION_H
#define FM_DOF_MOTION_H

#include "vpmDB/FmIsControlledBase.H"
#include "FFaLib/FFaString/FFaEnum.H"

class FmHasDOFsBase;


class FmDofMotion : public FmIsControlledBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmDofMotion();

  virtual bool disconnect();
  virtual void initAfterResolve();

  FmHasDOFsBase* getOwner() const;

  virtual FmModelMemberBase* getActiveOwner() const;

  virtual bool isListable() const { return false; }

  virtual int printSolverEntry(FILE* fp);

  double getInitMotion() const { return myMotionVal.getValue(); }
  bool   setInitMotion(double pm) { return myMotionVal.setValue(pm); }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Motion"; }

  static int checkMotions();

  enum MotionType { DEFLECTION, VELOCITY, ACCELERATION };

protected:
  FFaEnumMapping(MotionType)
  {
    FFaEnumEntry(DEFLECTION  ,"DEFLECTION");
    FFaEnumEntry(VELOCITY    ,"VELOCITY");
    FFaEnumEntry(ACCELERATION,"ACCELERATION");
    FFaEnumEntryEnd;
  };

  FFaField<MotionTypeEnum> myType;

public:
  MotionType getMotionType() const { return myType.getValue(); }
  bool setMotionType(MotionType type) { return myType.setValue(type); }
  bool setMotionType(int type);

protected:
  virtual ~FmDofMotion();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmDofMotion* obj);

  FFaField<double> myMotionVal;

public:
  FFaField<bool>   freqDomain;
};

#endif
