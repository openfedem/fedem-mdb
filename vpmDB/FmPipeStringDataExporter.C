// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmPipeStringDataExporter.H"
#include "FFaLib/FFaString/FFaParse.H"


Fmd_DB_SOURCE_INIT(FcPIPE_STRING_DATA_EXPORTER, FmPipeStringDataExporter, FmResultBase);


FmPipeStringDataExporter::FmPipeStringDataExporter()
{
  Fmd_CONSTRUCTOR_INIT(FmPipeStringDataExporter);

  FFA_REFERENCELIST_FIELD_INIT(contactPointsField, contactPoints, "CONTACT_POINTS");
  FFA_FIELD_DEFAULT_INIT(times, "TIMES");
  FFA_FIELD_DEFAULT_INIT(stringFrontDepths, "STRING_FRONT_DEPTHS");
  FFA_FIELD_DEFAULT_INIT(jointMDPositions, "MD_POSITIONS_CONTACT_POINTS");
  FFA_REFERENCE_FIELD_INIT(hivEngineField, hivEngine, "HIV_ENGINE");
}


FmPipeStringDataExporter::~FmPipeStringDataExporter()
{
  this->disconnect();
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmPipeStringDataExporter::writeFMF(std::ostream& os)
{
  os <<"PIPE_STRING_DATA_EXPORTER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmPipeStringDataExporter::readAndConnect(std::istream& is, std::ostream&)
{
  FmPipeStringDataExporter* obj = new FmPipeStringDataExporter();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	parentParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}
