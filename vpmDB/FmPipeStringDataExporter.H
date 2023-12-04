// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_PIPE_STRING_DATA_EXPORTER_H
#define FM_PIPE_STRING_DATA_EXPORTER_H

#include "vpmDB/FmResultBase.H"
#include "vpmDB/FmCamJoint.H"
#include "vpmDB/FmEngine.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmPipeStringDataExporter : public FmResultBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmPipeStringDataExporter();

  // String position and times for the position
  FFaField< std::vector<double> > times;
  FFaField< std::vector<double> > stringFrontDepths;

  // Contact points and their MD position
  FFaField< std::vector<double> > jointMDPositions;
  FFaReferenceList<FmCamJoint>    contactPoints;
  FFaField<FFaReferenceListBase*> contactPointsField;

  // Reference to the engine doing the vertical hiv motion
  FFaReference<FmEngine>      hivEngine;
  FFaField<FFaReferenceBase*> hivEngineField;

protected:
  virtual ~FmPipeStringDataExporter();

public:
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual const char* getUITypeName() const { return "Pipe String Data"; }
};

#endif
