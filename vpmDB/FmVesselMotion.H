// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_VESSEL_MOTION_H
#define FM_VESSEL_MOTION_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmFileReference.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaAlgebra/FFaVec3.H"

class FmHasDOFsBase;
class FaMat34;


class FmVesselMotion : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmVesselMotion();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  virtual FmBase* duplicate() const;

  virtual const char* getUITypeName() const { return "Vessel motion"; }

  const std::string& getActualRAOFileName() const;

  void createMotions();
  bool scaleMotions(FmEngine* scaling);

  void onWaveChanged(bool updateSeaViz = true);
  void onRAOChanged(bool updateSeaViz = false);

  virtual void onEventSwitched(const FmSimulationEvent* event);

  bool getWaveAngles(std::vector<int>& angles) const;
  FmHasDOFsBase* getVesselTriad() const;
  FaMat34 getWaveCS(const FaVec3& g, const FaVec3& x, double z0) const;

  bool setWaveDir(int newDir);

protected:
  virtual ~FmVesselMotion();

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<FFaReferenceBase*>     waveFunctionField;
  FFaField<FFaReferenceListBase*> motionEngineField;
  FFaField<FFaReferenceBase*>     motionScaleField;
  FFaField<FFaReferenceBase*>     raoFileRefField;

public:
  FFaReference<FmMathFuncBase>  waveFunction;
  FFaReferenceList<FmEngine>    motionEngine;
  FFaReference<FmEngine>        motionScale;
  FFaReference<FmFileReference> raoFileRef;

  FFaField<std::string> raoFile;
  FFaField<FaVec3>      offSet;
  FFaField<int>         waveDir;
};

#endif
