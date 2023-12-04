// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_RACK_PINION_H
#define FM_RACK_PINION_H

#include "vpmDB/FmHPBase.H"

class FmRevJoint;
class FmPrismJoint;


class FmRackPinion : public FmHPBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRackPinion();

  bool connect(FmRevJoint* inputJnt, FmPrismJoint* outputJnt);
  using FmHPBase::connect;

  virtual FmJointBase* getOutputJoint() const;
  virtual bool setOutputJoint(FmJointBase* jnt);
  virtual void removeItsOutputJoint();

  virtual int getInputJointDof() const { return 1; }  // RevJoint rotation
  virtual int getOutputJointDof() const { return 3; } // PrismJoint slider

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Rack and pinion"; }

protected:
  virtual ~FmRackPinion();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmRackPinion* obj);

private:
  FFaReference<FmJointBase>   itsOutputJoint;
  FFaField<FFaReferenceBase*> itsOutputJointField;
};

#endif
