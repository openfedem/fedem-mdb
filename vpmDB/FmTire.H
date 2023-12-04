// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_TIRE_H
#define FM_TIRE_H

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmRoad.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmFileReference.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmTire : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmTire();

  FFaField<FFaReferenceBase*>   roadField;
  FFaReference<FmRoad>          road;

  FFaField<FFaReferenceBase*>   bearingJointField;
  FFaReference<FmJointBase>     bearingJoint;

  FFaField<FFaReferenceBase*>   spindelTriadField;
  FFaReference<FmTriad>         spindelTriad;

  FFaReference<FmFileReference> tireDataFileRef;
  FFaField<FFaReferenceBase*>   tireDataFileRefField;
  FFaField<std::string>         tireDataFileName;

  FFaField<std::string>         tireType;
  FFaField<std::string>         tireAPI;

  FFaField<double>              spindelTriadOffset;

  FFaNoPrintField<double>       tireVerticalStiffness;
  FFaNoPrintField<double>       tireVerticalDamping;

  const std::string& getActualTireDataFileName() const;

  virtual int printSolverEntry(FILE* fp);

  // Visualization parameters access :

  void   updateFromFile();
  double getUnloadedTireRadius() const { return myUnloadedTireRadius; }
  double getRimRadius() const          { return myRimRadius; }
  double getTireWidth() const          { return myTireWidth; }
  double getRimWidth() const           { return myRimWidth; }
  bool   hasValidVisData() const       { return IHaveValidVisData; }

  // Normal stuff:

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Tire"; }

protected:
  virtual ~FmTire();

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  // Visualization parameters cached from file:
  bool   IHaveValidVisData;
  double myUnloadedTireRadius;
  double myRimRadius;
  double myTireWidth;
  double myRimWidth;

  std::vector<Doubles> tireSurfaceShape;
};

#endif
