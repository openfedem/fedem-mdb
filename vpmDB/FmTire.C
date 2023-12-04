// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmTire.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaAlgebra/FFaUnitCalculator.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#ifdef USE_INVENTOR
#include "vpmDisplay/FdTire.H"
#endif
#include <algorithm>
#include <fstream>


Fmd_DB_SOURCE_INIT(FcTIRE, FmTire, FmIsPlottedBase);


FmTire::FmTire()
{
  Fmd_CONSTRUCTOR_INIT(FmTire);

  FFA_REFERENCE_FIELD_INIT(roadField,            road,            "ROAD"               );
  FFA_REFERENCE_FIELD_INIT(bearingJointField,    bearingJoint,    "BEARING_JOINT"      );
  FFA_REFERENCE_FIELD_INIT(spindelTriadField,    spindelTriad,    "SPINDEL_TRIAD"      );
  FFA_REFERENCE_FIELD_INIT(tireDataFileRefField, tireDataFileRef, "TIRE_DATA_FILE_REF" );
  tireDataFileRef.setPrintIfZero(false);

  FFA_FIELD_DEFAULT_INIT(tireDataFileName, "TIRE_DATA_FILE");
  FFA_FIELD_INIT(tireType,      "MF-TYRE", "TIRE_TYPE");
  FFA_FIELD_INIT(tireAPI,           "STI", "TIRE_API");
  FFA_FIELD_INIT(spindelTriadOffset,  0.0, "SPINDEL_TRIAD_OFFSET");

  FFA_FIELD_INIT(tireVerticalStiffness, 0.0, "VERTICAL_STIFFNESS");
  FFA_FIELD_INIT(tireVerticalDamping,   0.0, "VERTICAL_DAMPING");

  myUnloadedTireRadius = 0.308;
  myRimRadius          = 0.191;
  myTireWidth          = 0.195;
  myRimWidth           = 0.1524;
  IHaveValidVisData    = false;

#ifdef USE_INVENTOR
  itsDisplayPt = new FdTire(this);
#endif
}


FmTire::~FmTire()
{
  this->disconnect();
}


std::ostream& FmTire::writeFMF(std::ostream& os)
{
  os <<"TIRE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


void FmTire::updateFromFile()
{
  IHaveValidVisData = false;
  tireSurfaceShape.clear();

  std::string fileName = this->getActualTireDataFileName();
  if (fileName.empty()) return;

  std::string absModelPath = FmDB::getMechanismObject()->getAbsModelFilePath();
  FFaFilePath::makeItAbsolute(fileName,absModelPath);

  std::ifstream stream(fileName.c_str(),std::ios::in);
  if (!stream) return;

  double tmpVertStiff = 0.0;
  double tmpVertDamp  = 0.0;

  std::string lengthUnit;
  std::string timeUnit;
  std::string propFileFormat;

  int (*pf)(int)=toupper;
  std::string keyword;
  char c;
  while (stream.get(c) && !stream.eof()) {
    stream.putback(c);
    FaParse::skipToWord(stream, '!');
    FaParse::skipToWord(stream, '$');
    stream >> keyword;

    std::transform(keyword.begin(), keyword.end(), keyword.begin(), pf);
    while (stream.get(c) && (isspace(c) || c == '='));
    stream.putback(c);
    if (keyword == "UNLOADED_RADIUS")  {
      stream >> myUnloadedTireRadius;
      myRimRadius = myUnloadedTireRadius*0.7;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "WIDTH") {
      stream >> myTireWidth;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "RIM_RADIUS") {
      stream >> myRimRadius;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "RIM_DIAMETER") {
      double tmpVar;
      stream >> tmpVar;
      myRimRadius = tmpVar/2;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "LENGTH") {
      stream >> lengthUnit;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "TIME") {
      stream >> timeUnit;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "RIM_WIDTH") {
      stream >> myRimWidth;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "VERTICAL_STIFFNESS") {
      stream >> tmpVertStiff;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "VERTICAL_DAMPING") {
      stream >> tmpVertDamp;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "PROPERTY_FILE_FORMAT") {
      stream >> propFileFormat;
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
    else if (keyword == "SHAPE]") {
      stream >> keyword; // {radial
      stream >> keyword; // width}
      while(!stream.eof()) {
        FaParse::skipToWordOrNum(stream, '$');
        FaParse::skipToWordOrNum(stream, '!');
        if (!stream.eof())
          {
            stream.get(c);
            if (isdigit(c) || c == '-')
              {
                double a, b;
                stream >> a >> b;
                tireSurfaceShape.push_back(std::make_pair(a,b));
              }
            else
              break;
          }
      }
      if (!stream.eof())
	FaParse::nextLine(stream, '$');
    }
  }
  stream.close();

  // scale first to M if the model is in MM
  std::transform(lengthUnit.begin(), lengthUnit.end(), lengthUnit.begin(), pf);
  std::transform(timeUnit.begin(), timeUnit.end(), timeUnit.begin(), pf);

  if (lengthUnit == "\'MM\'")
    {
      tmpVertDamp           *= 1000;
      tmpVertStiff          *= 1000;
      myUnloadedTireRadius  /= 1000;
      myRimRadius           /= 1000;
      myTireWidth           /= 1000;
      myRimWidth            /= 1000;
    }

  if (timeUnit == "\'MILLISECOND\'")
    {
      tmpVertDamp           /= 1000;
    }

  // then scale to the appropriate size.
  FFaUnitCalculator& modelDBUnits = FmDB::getMechanismObject()->modelDatabaseUnits.getValue();
  if (modelDBUnits.isValid())
  {
    modelDBUnits.inverse(myUnloadedTireRadius, "LENGTH");
    modelDBUnits.inverse(myRimRadius, "LENGTH");
    modelDBUnits.inverse(myTireWidth, "LENGTH");
    modelDBUnits.inverse(myRimWidth,  "LENGTH");
    modelDBUnits.inverse(tmpVertStiff,"FORCE/LENGTH");
    modelDBUnits.inverse(tmpVertDamp, "FORCE/LENGTH");
    modelDBUnits.inverse(tmpVertDamp, "TIME");
  }

  tireVerticalStiffness.setValue(tmpVertStiff);
  tireVerticalDamping.setValue(tmpVertDamp);

  std::transform(propFileFormat.begin(), propFileFormat.end(), propFileFormat.begin(), pf);

  if (this->tireType.getValue().empty())
  {
    if (propFileFormat == "\'FTIRE\'")
      this->tireType = "FTIRE";
    else if (propFileFormat == "\'SWIFT-TYRE\'")
      this->tireType = "SWIFT";
    else if (propFileFormat == "\'MF-TYRE\'")
      this->tireType = "MF-TYRE";
  }

  if (this->tireAPI.getValue().empty())
  {
    if (this->tireType.getValue() == "FTIRE")
      this->tireAPI = "CTI";
    else
      this->tireAPI = "STI";
  }

  IHaveValidVisData = true;
}


bool FmTire::readAndConnect(std::istream& is, std::ostream&)
{
  FmTire* obj = new FmTire();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFaFilePath::checkName(obj->tireDataFileName.getValue());

  // Override possibly erroneously set tireAPI in old model files
  if (obj->tireType.getValue() == "FTIRE")
    obj->tireAPI = "CTI";
  else if (obj->tireAPI.getValue() == "CTI")
    obj->tireAPI = "STI";

  obj->connect();
  return true;
}


bool FmTire::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmTire::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmTire::getClassTypeID());
}


const std::string& FmTire::getActualTireDataFileName() const
{
  if (tireDataFileRef)
    return tireDataFileRef->fileName.getValue();
  else
    return tireDataFileName.getValue();
}


int FmTire::printSolverEntry(FILE* fp)
{
  this->updateFromFile(); // To ensure that we have legal tire stiffness data

  fprintf(fp, "&TIRE\n");
  this->printID(fp);
  fprintf(fp, "  type         = '%s'\n", tireType.getValue().c_str());
  fprintf(fp, "  api          = '%s'\n", tireAPI.getValue().c_str());

  std::string tireFile = this->getActualTireDataFileName();
  FFaFilePath::makeItAbsolute(tireFile,relPathCorrection);
  fprintf(fp, "  tireDataFile = '%s'\n", tireFile.c_str());

  if (road.isNull() || bearingJoint.isNull())
  {
    ListUI <<"\n---> INTERNAL ERROR: "<< this->getIdString(true)
	   <<" is inconsistent, no road or bearing joint connected.\n";
    fprintf(fp, "/\n\n");
    return 1;
  }

  fprintf(fp, "  roadId       = %d\n", road->getBaseID());
  fprintf(fp, "  jointId      = %d\n", bearingJoint->getBaseID());

  int tireChar = 14; // Dynamic combined forces/torques in XYZ is the default
  FFaString tDesc = this->getUserDescription();

  // Beta feature: Tire model characteristic parameter
  if (tDesc.hasSubString("#SteadyState")) tireChar = 4;
  if (tDesc.hasSubString("#MirrorTChar")) tireChar = -tireChar;
  if (tDesc.hasSubString("#ISWTCH"))      tireChar = tDesc.getIntAfter("#ISWTCH");
  fprintf(fp, "  tireChar     = %d\n", tireChar);

  fprintf(fp, "  Zoffset      = %17.9e\n", spindelTriadOffset.getValue());

  // Beta feature: Flip the wheel carrier axis
  if (tDesc.hasSubString("#FlipWCaxis"))
    fprintf(fp, "  WCYalongZ    = 0\n");

  // Beta feature: Override radial stiffness in tire property file
  if (tDesc.hasSubString("#radialStiff"))
    fprintf(fp, "  radialStiff  = %17.9e\n", tDesc.getDoubleAfter("#radialStiff"));
  else
    fprintf(fp, "  radialStiff  = %17.9e\n", tireVerticalStiffness.getValue());

  // Beta feature: Override radial damping in tire property file
  if (tDesc.hasSubString("#radialDamp"))
    fprintf(fp, "  radialDamp   = %17.9e\n", tDesc.getDoubleAfter("#radialDamp"));
  else
    fprintf(fp, "  radialDamp   = %17.9e\n", tireVerticalDamping.getValue());

  // Variables to be saved:
  // 1 - Tire angles and slip
  // 2 - Tire rolling radius
  // 3 - Contact force
  // 4 - Contact position and road normal
  // 5 - Tire deflection
  // 6 - Deflection velocity
  // 7 - Tire characteristics
  // 8 - Wheel carrier force
  // 9 - Tire energy
  this->writeSaveVar(fp,9);

  fprintf(fp, "/\n\n");
  return 0;
}
