// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ASSEMBLY_BASE_H
#define FM_ASSEMBLY_BASE_H

#include "vpmDB/FmSubAssembly.H"
#include "FFaLib/FFaAlgebra/FFaMat34.H"
#include "FFaLib/FFaAlgebra/FFa3DLocation.H"


class FmAssemblyBase : public FmSubAssembly
{
protected:
  FmAssemblyBase(bool isDummy = false);
  virtual ~FmAssemblyBase() {}

public:
  FmAssemblyBase* getPositionedParent() const;

  void setLocalCS(const FaMat34& cs, bool updateLoc = true);
  void setGlobalCS(const FaMat34& cs, bool updateLoc = true);

  FaMat34 toGlobal(const FaMat34& m) const;
  FaMat34 toLocal(const FaMat34& m) const;
  FaMat33 toGlobal(const FaMat33& m) const;
  FaMat33 toLocal(const FaMat33& m) const;
  FaVec3 toGlobal(const FaVec3& v, bool directionOnly = false) const;
  FaVec3 toLocal(const FaVec3& v, bool directionOnly = false) const;

  void setLocation(const FFa3DLocation& loc);
  const FFa3DLocation& getLocation() const { return myLocation.getValue(); }

  virtual void updateLocation(char updateWhat = 'A');
  virtual double getMass(FaVec3& CoG, bool includeSubAss = true) const;
  virtual FmBase* duplicate() const;
  using FmSubAssembly::duplicate;

  bool isMovable() const;

  double getTotalLength() const;
  double getTotalMass(double* pLength = NULL) const;
  FaVec3 getGlobalCoG(bool includeSubAss = true) const;
  FaVec3 getExtents() const;

protected:
  FFaField<FaMat34>       myCS;
  FFaField<FFa3DLocation> myLocation;
};

#endif
