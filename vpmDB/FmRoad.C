// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmRoad.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"


Fmd_DB_SOURCE_INIT(FcROAD, FmRoad, FmSimulationModelBase);


FmRoad::FmRoad()
{
  Fmd_CONSTRUCTOR_INIT(FmRoad);

  FFA_REFERENCE_FIELD_INIT(roadFunctionField, roadFunction, "ROAD_SHAPE_FUNCTION");

  FFA_FIELD_INIT(roadZShift,    0.0, "ROAD_Z_SHIFT");
  FFA_FIELD_INIT(roadXOffset,   0.0, "ROAD_X_OFFSET");
  FFA_FIELD_INIT(roadZRotation, 0.0, "ROAD_Z_ROTATION");

  FFA_REFERENCE_FIELD_INIT(roadXMotionField, roadXMotion, "ROAD_X_MOTION_ENGINE");
  FFA_REFERENCE_FIELD_INIT(roadYMotionField, roadYMotion, "ROAD_Y_MOTION_ENGINE");
  FFA_REFERENCE_FIELD_INIT(roadZMotionField, roadZMotion, "ROAD_Z_MOTION_ENGINE");
  roadXMotion.setPrintIfZero(false);
  roadYMotion.setPrintIfZero(false);
  roadZMotion.setPrintIfZero(false);

  FFA_REFERENCE_FIELD_INIT(roadDataFileRefField, roadDataFileRef, "ROAD_DATA_FILE_REF");
  roadDataFileRef.setPrintIfZero(false);

  FFA_FIELD_DEFAULT_INIT(roadDataFileName, "ROAD_DATA_FILE");
  FFA_FIELD_INIT(useExtRoadDataFile, false, "USE_EXT_ROAD_DATA_FILE");
}


FmRoad::~FmRoad()
{
  this->disconnect();
}


std::ostream& FmRoad::writeFMF(std::ostream& os)
{
  os <<"ROAD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmRoad::readAndConnect(std::istream& is, std::ostream&)
{
  FmRoad* obj = new FmRoad();

  // obsolete fields
  FFaObsoleteField<std::string> roadFunctionInterpretationObs;
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(roadFunctionInterpretationObs, "ROAD_FUNCTION_INTERPRETATION", obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  // remove obsolete fields
  FFA_OBSOLETE_FIELD_REMOVE("ROAD_FUNCTION_INTERPRETATION", obj);

  if (roadFunctionInterpretationObs.wasOnFile())
  {
    // overrides new data
    if (roadFunctionInterpretationObs.getValue() == "GLOB_X")
      obj->roadZRotation = 0;
    else if (roadFunctionInterpretationObs.getValue() == "GLOB_Y")
      obj->roadZRotation = 90;
  }

  obj->connect();
  return true;
}


bool FmRoad::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmRoad::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmRoad::getClassTypeID());
}


const std::string& FmRoad::getActualRoadDataFileName() const
{
  if (roadDataFileRef)
    return roadDataFileRef->fileName.getValue();
  else
    return roadDataFileName.getValue();
}


int FmRoad::printSolverEntry(FILE* fp)
{
  fprintf(fp, "&ROAD\n");
  this->printID(fp);
  if (useExtRoadDataFile.getValue())
  {
    std::string roadFile = this->getActualRoadDataFileName();
    FFaFilePath::makeItAbsolute(roadFile,relPathCorrection);
    fprintf(fp, "  roadDataFileName = '%s'\n", roadFile.c_str());
  }
  else if (!roadFunction.isNull())
  {
    fprintf(fp, "  roadFuncId = %d\n", this->roadFunction->getBaseID());
    fprintf(fp, "  Xoffset    = %g\n", this->roadXOffset.getValue());
    fprintf(fp, "  Zshift     = %g\n", this->roadZShift.getValue());
    fprintf(fp, "  ThetaInRad = .false.\n");
    fprintf(fp, "  Theta      = %g\n", this->roadZRotation.getValue());
  }

  // Beta feature: Engines moving the road surface around
  FFaString rDesc = this->getUserDescription();
  if (rDesc.hasSubString("#RoadXengId"))
    fprintf(fp, "  roadXengId = %d\n", rDesc.getIntAfter("#RoadXengId"));
  if (rDesc.hasSubString("#RoadYengId"))
    fprintf(fp, "  roadYengId = %d\n", rDesc.getIntAfter("#RoadYengId"));
  if (rDesc.hasSubString("#RoadZengId"))
    fprintf(fp, "  roadZengId = %d\n", rDesc.getIntAfter("#RoadZengId"));

  fprintf(fp, "/\n\n");
  return 0;
}
