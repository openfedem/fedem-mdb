// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#include "vpmDB/FmDB.H"
#include "vpmDB/FmFppOptions.H"


Fmd_DB_SOURCE_INIT(FcFPPOPTIONS, FmFppOptions, FmSimulationModelBase);


FmFppOptions::FmFppOptions()
{
  Fmd_CONSTRUCTOR_INIT(FmFppOptions);

  FFA_FIELD_INIT(startTime,0.0,"START_TIME");
  FFA_FIELD_INIT(stopTime,1.0,"STOP_TIME");
  FFA_FIELD_INIT(timeIncr,0.01,"TIME_INCR");
  FFA_FIELD_INIT(allTimeSteps,true,"USE_ALL_TIME_STEPS");
  FFA_FIELD_INIT(nElemsTogether,2000,"N_ELEMS_PROCESSED_TOGETHER");
  FFA_FIELD_INIT(performRainflow,true,"PERFORM_RAINFLOW");
  FFA_FIELD_INIT(histType,S_N,"HIST_ANALYSIS_TYPE");
  FFA_FIELD_INIT(pvxGate,10.0,"PVX_GATE");
  FFA_FIELD_INIT(biaxGate,10.0,"BIAX_GATE");
  FFA_FIELD_INIT(histRange,std::make_pair(-100.0,100.0),"HIST_RANGE");
  FFA_FIELD_INIT(histNBins,64,"HIST_N_BINS");
  FFA_FIELD_DEFAULT_INIT(addOptions,"ADD_OPTIONS");
}


FmFppOptions::~FmFppOptions()
{
  this->disconnect();
}


bool FmFppOptions::useNCode() const
{
  return addOptions.getValue().find("#useNCode") < std::string::npos;
}


bool FmFppOptions::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmFppOptions::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmFppOptions::getClassTypeID());
}


std::ostream& FmFppOptions::writeFMF(std::ostream& os)
{
  os <<"FPPOPTIONS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmFppOptions::readAndConnect(std::istream& is, std::ostream&)
{
  FmFppOptions* obj = new FmFppOptions();

  // Obsolete fields from R4.0 and earlier
  FFaObsoleteField<double> stressScaleFactor, histMaxX, histMinX;
  FFA_OBSOLETE_FIELD_INIT(stressScaleFactor,1.0e-6,"STRESS_SCALE_FACTOR",obj);
  FFA_OBSOLETE_FIELD_INIT(histMaxX, 100.0,"HIST_MAX_X",obj);
  FFA_OBSOLETE_FIELD_INIT(histMinX,-100.0,"HIST_MIN_X",obj);

  FFaObsoleteField<bool> histStressType, histStrainType;
  FFA_OBSOLETE_FIELD_INIT(histStressType,false,"HIST_ABS_MAX_STRESS_TYPE",obj);
  FFA_OBSOLETE_FIELD_INIT(histStrainType,true, "HIST_ABS_MAX_STRAIN_TYPE",obj);

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  // Remove the obsolete fields
  FFA_OBSOLETE_FIELD_REMOVE("STRESS_SCALE_FACTOR",obj);
  FFA_OBSOLETE_FIELD_REMOVE("HIST_MAX_X",obj);
  FFA_OBSOLETE_FIELD_REMOVE("HIST_MIN_X",obj);
  FFA_OBSOLETE_FIELD_REMOVE("HIST_ABS_MAX_STRESS_TYPE",obj);
  FFA_OBSOLETE_FIELD_REMOVE("HIST_ABS_MAX_STRAIN_TYPE",obj);

  // If the obsolete fields were present on the model file it was most likely
  // a model file R4.0 or earlier. Let them override the new data in that case.

  if (stressScaleFactor.wasOnFile() && obj->performRainflow.getValue())
    ListUI <<"===> WARNING: A stress scale factor to MPa ("
	   << stressScaleFactor.getValue() <<") was stored in the\n"
	   <<"     Strain Coat Recovery Setup section of this model file.\n"
	   <<"     This field has been removed in this version, and the scale\n"
	   <<"     factor is now derived from the selected Model database units"
	 <<"\n     in the \"Model Preferences\" dialog.\n";

  if (histMaxX.wasOnFile() && histMinX.wasOnFile())
    obj->histRange.setValue(std::make_pair(histMinX.getValue(),histMaxX.getValue()));

  if (histStressType.wasOnFile() && histStrainType.wasOnFile())
  {
    if (histStressType.getValue())
      obj->histType.setValue(S_N);
    else if (histStrainType.getValue())
      obj->histType.setValue(E_N);
  }

  return obj->cloneOrConnect();
}
