// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmStrainRosette.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmGageOptions.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcGAGEOPTIONS, FmGageOptions, FmSimulationModelBase);

FmGageOptions::FmGageOptions()
{
  Fmd_CONSTRUCTOR_INIT(FmGageOptions);

  FFA_FIELD_INIT(startTime,0.0,"START_TIME");
  FFA_FIELD_INIT(stopTime,1.0,"STOP_TIME");
  FFA_FIELD_INIT(timeIncr,0.01,"TIME_INCR");
  FFA_FIELD_INIT(allTimeSteps,true,"USE_ALL_TIME_STEPS");
  FFA_FIELD_INIT(autoDacExport,false,"AUTO_DAC_EXPORT");
  FFA_FIELD_INIT(dacSampleRate,0.001,"DAC_SAMPLE_RATE");
  FFA_FIELD_INIT(fatigue,false,"RAINFLOW_ANALYSIS");

  double defaultSize = 1.0e7; // that is 10.0 MPa
  FFaUnitCalculator& toSI = FmDB::getMechanismObject()->modelDatabaseUnits.getValue();
  // Apply the inverse scaling factor to SI units
  if (toSI.isValid()) toSI.inverse(defaultSize,"FORCE/AREA");
  FFA_FIELD_INIT(binSize,defaultSize,"STRESS_BINSIZE");

  FFA_FIELD_DEFAULT_INIT(addOptions,"ADD_OPTIONS");
}


FmGageOptions::~FmGageOptions()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmGageOptions::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmGageOptions::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmGageOptions::getClassTypeID());
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmGageOptions::writeFMF(std::ostream& os)
{
  os <<"GAGEOPTIONS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmGageOptions::readAndConnect(std::istream& is, std::ostream&)
{
  FmGageOptions* obj = new FmGageOptions();

  // Obsolete fields
  FFaObsoleteField<std::string> rosDefFile;
  FFaObsoleteField<bool> nullifyStrains;
  FFA_OBSOLETE_FIELD_DEFAULT_INIT(rosDefFile,"ROSETTE_DEFINE_FILE",obj);
  FFA_OBSOLETE_FIELD_INIT(nullifyStrains,false,"NULLIFY_ROSETTE_STRAINS",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFA_OBSOLETE_FIELD_REMOVE("ROSETTE_DEFINE_FILE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("NULLIFY_ROSETTE_STRAINS",obj);

  // Convert old gage definition file to strain rosette objects
  if (!rosDefFile.getValue().empty())
    FmStrainRosette::createRosettesFromOldFile(rosDefFile.getValue(),
					       nullifyStrains.getValue());
  else if (nullifyStrains.getValue())
    FFaMsg::list(" ==> This model has the global \"Set all start strains to zero\"\n"
		 "     option in the Strain Rosette Recovery Setup toggled on.\n"
		 "     This option has been removed. Use the equivalent option for\n"
		 "     each strain rosette in the property editor panel instead.\n",true);

  return obj->cloneOrConnect();
}
