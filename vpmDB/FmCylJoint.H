// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CYL_JOINT_H
#define FM_CYL_JOINT_H

#include "vpmDB/FmMMJointBase.H"


class FmCylJoint : public FmMMJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmCylJoint();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual const char* getUITypeName() const { return "Cylindric joint"; }

  double getScrewRatio() const;
  bool setScrewRatio(double ratio);

  void setAsScrewTransmission() { myScrewTransFlag.setValue(true); }
  void unsetAsScrewTransmission() { myScrewTransFlag.setValue(false); }
  bool isScrewTransmission() const { return myScrewTransFlag.getValue(); }

protected:
  virtual ~FmCylJoint() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<bool>   myScrewTransFlag;
  FFaField<double> myScrewTransRatio;
};

#endif
