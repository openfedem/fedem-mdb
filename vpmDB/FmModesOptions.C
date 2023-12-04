// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmModesOptions.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include <algorithm>


Fmd_DB_SOURCE_INIT(FcMODESOPTIONS, FmModesOptions, FmSimulationModelBase);


FmModesOptions::FmModesOptions()
{
  Fmd_CONSTRUCTOR_INIT(FmModesOptions);

  convertedFromOldModelFile = false;

  FFA_FIELD_DEFAULT_INIT(modesArray,"EXPAND_MODES");
  FFA_FIELD_INIT(autoVTFExport,false,"AUTO_VTF_EXPORT");
  FFA_FIELD_INIT(vtfFileName,"modes.vtf","AUTO_VTF_FILE");
  FFA_FIELD_INIT(vtfFileType,VTF_EXPRESS,"AUTO_VTF_TYPE");
  FFA_FIELD_DEFAULT_INIT(addOptions,"ADD_OPTIONS");
}


FmModesOptions::~FmModesOptions()
{
  this->disconnect();
}


bool FmModesOptions::setMaxEigenmode(int modeno)
{
  if (modeno < 0) return false;

  // remove mode data when the size moves down...
  FmModeVec& modes = modesArray.getValue();
  FmModeVec::iterator it = modes.begin();
  while (it != modes.end())
    if (it->first > modeno)
      it = modes.erase(it);
    else
      ++it;

  return true;
}


bool FmModesOptions::setMinTime(double minTime)
{
  // remove mode data beyond time constraints
  FmModeVec& modes = modesArray.getValue();
  FmModeVec::iterator it = modes.begin();
  while (it != modes.end())
    if (it->second < minTime)
      it = modes.erase(it);
    else
      ++it;

  return true;
}


bool FmModesOptions::setMaxTime(double maxTime)
{
  // remove mode data beyond time constraints
  FmModeVec& modes = modesArray.getValue();
  FmModeVec::iterator it = modes.begin();
  while (it != modes.end())
    if (it->second > maxTime)
      it = modes.erase(it);
    else
      ++it;

  return true;
}


int FmModesOptions::addEigenmodeData(int modeNo, double time)
{
  FmModeVec& modes = modesArray.getValue();
  FmModeVec::iterator it = std::find(modes.begin(),modes.end(),
                                     FmModeType(modeNo,time));

  if (it != modes.end())
    return it - modes.begin();
  else if (modeNo < 1)
    return -1;
  else if (modeNo > FmDB::getActiveAnalysis()->numEigenmodes.getValue())
    return -1;

  modes.push_back(FmModeType(modeNo,time));
  return modes.size() - 1;
}


std::ostream& FmModesOptions::writeFMF(std::ostream& os)
{
  os <<"MODESOPTIONS\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmModesOptions::readAndConnect(std::istream& is, std::ostream&)
{
  FmModesOptions* obj = new FmModesOptions();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      localParse(keyWord, activeStatement, obj);
  }

  if (obj->convertedFromOldModelFile && obj->modesArray.getValue().empty())
    return obj->erase();

  return obj->cloneOrConnect();
}


bool FmModesOptions::localParse(const char* keyWord, std::istream& activeStatement,
				FmModesOptions* obj)
{
  enum {ENDMARK = -1,
	EXPAND_EIGENMODES = 1,
	OWNER_ANALYSIS = 2,
	EIGENMODE_EXPAND_DATA = 3,
	MODE_EXPAND_DATA = 4,
	MODE_EXPAND_DATA_NEW = 5,
	UNKNOWN = 6};

  const char* key_words[] = {"EXPAND_EIGENMODES",
			     "OWNER_ANALYSIS",
			     "EIGENMODE_EXPAND_DATA",
			     "MODE_EXPAND_DATA",
			     "MODE_EXPAND_DATA_NEW",
			     NULL};

  int i, eigenmodeCount = 0;
  int key = FaParse::findIndex(key_words,keyWord);

  switch (key)
    {
    case ENDMARK:
      return true;

    case OWNER_ANALYSIS:
      obj->convertedFromOldModelFile = true;
      break;

    case EXPAND_EIGENMODES:
    case EIGENMODE_EXPAND_DATA:
    case MODE_EXPAND_DATA_NEW:
      // Conversion of old model file data
      obj->convertedFromOldModelFile = true;
      activeStatement >> eigenmodeCount;
      FmDB::getActiveAnalysis()->numEigenmodes = eigenmodeCount;
      for (i = 0; i < eigenmodeCount; i++)
      {
	int tmpInt, modeNo = 0, timeCount = 0;
	activeStatement >> modeNo;
	if (key == EXPAND_EIGENMODES)
	  activeStatement >> tmpInt;
	activeStatement >> timeCount;
	for (int k = 0; k < timeCount; k++)
	{
	  double tmpTime = 0.0;
	  activeStatement >> tmpTime;
	  if (key == EIGENMODE_EXPAND_DATA || key == MODE_EXPAND_DATA_NEW)
	    activeStatement >> tmpInt;
	  if (key == MODE_EXPAND_DATA_NEW)
	    activeStatement >> tmpInt;
	  if (modeNo > 0 && modeNo <= eigenmodeCount)
	    obj->addEigenmodeData(modeNo,tmpTime);
	}
      }
      break;

    case MODE_EXPAND_DATA:
      obj->convertedFromOldModelFile = true;
      activeStatement >> eigenmodeCount;
      FmDB::getActiveAnalysis()->numEigenmodes = eigenmodeCount;
      break;

    case UNKNOWN:
      break;

    case false:
      return parentParse(keyWord, activeStatement, obj);
    }

  return false;
}


bool FmModesOptions::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmModesOptions::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmModesOptions::getClassTypeID());
}
