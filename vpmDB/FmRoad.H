// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ROAD_H
#define FM_ROAD_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmFileReference.H"
#include "FFaLib/FFaContainers/FFaReference.H"


class FmRoad : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmRoad();

  FFaField<FFaReferenceBase*>  roadFunctionField;
  FFaReference<FmMathFuncBase> roadFunction;

  FFaField<double>             roadZShift;
  FFaField<double>             roadXOffset;
  FFaField<double>             roadZRotation;

  FFaReference<FmFileReference> roadDataFileRef;
  FFaField<FFaReferenceBase*>   roadDataFileRefField;
  FFaField<std::string>         roadDataFileName;
  FFaField<bool>                useExtRoadDataFile;

  FFaField<FFaReferenceBase*>  roadXMotionField;
  FFaReference<FmEngine>       roadXMotion;
  FFaField<FFaReferenceBase*>  roadYMotionField;
  FFaReference<FmEngine>       roadYMotion;
  FFaField<FFaReferenceBase*>  roadZMotionField;
  FFaReference<FmEngine>       roadZMotion;

  const std::string& getActualRoadDataFileName() const;

  virtual int printSolverEntry(FILE* fp);

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Road"; }

  virtual bool isListable() const { return true; }

protected:
  virtual ~FmRoad();

  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
