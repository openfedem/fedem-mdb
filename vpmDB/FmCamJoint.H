// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CAM_JOINT_H
#define FM_CAM_JOINT_H

#include "vpmDB/FmMMJointBase.H"


class FmCamJoint : public FmMMJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmCamJoint();

  virtual bool addMasterOnPoint(const FaVec3& globalPoint);
  virtual bool addAsMasterTriad(FmTriad* triad);

  void setDefaultRotationOnMasters();

  double getThickness() const { return myCamThickness.getValue(); }
  void setThickness(double var) { myCamThickness.setValue(var); }

  double getWidth() const { return myCamWidth.getValue(); }
  void setWidth(double var) { myCamWidth.setValue(var); }

  bool isUsingRadialContact() const { return IAmUsingCylCoords.getValue(); }
  void setUsingRadialContact(bool isRadial) { IAmUsingCylCoords.setValue(isRadial); }

  virtual FaVec3 getTransJointVariables() const;
  virtual FaVec3 getRotJointVariables() const;

  virtual int getValidFrictionType() const;
  virtual int getFrictionDof() const { return 6; }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual const char* getUITypeName() const { return "Cam joint"; }

protected:
  virtual ~FmCamJoint() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool getSaveVar(unsigned int& nVar, IntVec& toggles) const;

private:
  FFaField<double> myCamWidth;
  FFaField<double> myCamThickness;
  FFaField<bool>   IAmUsingCylCoords;
};

#endif
