// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "vpmDB/FmModelExpOptions.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcMODELEXPORTOPTIONS, FmModelExpOptions, FmModelMemberBase);

FmModelExpOptions::FmModelExpOptions()
{
  Fmd_CONSTRUCTOR_INIT(FmModelExpOptions);

  //Stream app
  FFA_FIELD_DEFAULT_INIT(streamFilename, "STREAM_APP_FILENAME");
  FFA_FIELD_DEFAULT_INIT(streamInputIndGroup, "STREAM_INPUT_INDICATOR_GROUP");
  FFA_FIELD_DEFAULT_INIT(streamOutputIndGroup, "STREAM_OUTPUT_INDICATOR_GROUP");
  FFA_FIELD_INIT(streamWindowSize, 10, "STREAM_WINDOW_SIZE");
  FFA_FIELD_INIT(streamTransferState, false, "STREAM_TRANSFER_STATE");
  FFA_FIELD_INIT(streamAppExport, false, "STREAM_APP_EXPORT");
  
  //Batch app
  FFA_FIELD_DEFAULT_INIT(batchFilename, "BATCH_APP_FILENAME");
  FFA_FIELD_DEFAULT_INIT(batchInputIndGroup, "BATCH_INPUT_INDICATOR_GROUP");
  FFA_FIELD_INIT(batchSurfaceOnly, true, "BATCH_SURFACE_ONLY");
  FFA_FIELD_INIT(batchStressRecovery, true, "BATCH_STRESS_RECOVERY");
  FFA_FIELD_INIT(batchAllFEParts, false, "BATCH_ALL_FE_PARTS");
  FFA_FIELD_INIT(batchAppExport, false, "BATCH_APP_EXPORT");

  //FMU
  FFA_FIELD_DEFAULT_INIT(fmuFilename, "FMU_FILENAME");
  FFA_FIELD_INIT(inclExtFuncFile, false, "INCLUDE_EXT_FUNC_FILE");
  FFA_FIELD_INIT(fmuExport, false, "FMU_APP_EXPORT");
}


FmModelExpOptions::~FmModelExpOptions()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmModelExpOptions::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmModelExpOptions::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmModelExpOptions::getClassTypeID());
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmModelExpOptions::writeFMF(std::ostream& os)
{
  os <<"MODEL_EXPORT_OPTIONS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmModelExpOptions::readAndConnect(std::istream& is, std::ostream&)
{
  FmModelExpOptions* obj = new FmModelExpOptions();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  return obj->cloneOrConnect();
}
