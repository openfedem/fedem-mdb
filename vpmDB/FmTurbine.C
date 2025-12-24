// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmGenericDBObject.H"
#include "vpmDB/FmRevJoint.H"
#include "vpmDB/FmAirState.H"
#include "vpmDB/FmAnalysis.H"
#include "vpmDB/FmLink.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/Icons/FmIconPixmaps.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "vpmDB/FmFileSys.H"


Fmd_SOURCE_INIT(FcTURBINE, FmTurbine, FmSubAssembly);


FmTurbine::FmTurbine(char createSubAssemblies, bool isDummy) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(rhoIce,916.7,"ICE_MASS_DENSITY");
  FFA_FIELD_INIT(ptfmRef,0.0,"PLATFORM_REFERENCE_HEIGHT");
  FFA_FIELD_INIT(nBlade,3,"NUMBER_OF_BLADES");
  FFA_FIELD_INIT(ctrlSys,false,"REGULATION_SYSTEM");

  FFA_REFERENCE_FIELD_INIT(bladeDefField, bladeDef, "BLADE_DESIGN");
  FFA_REFERENCELIST_FIELD_INIT(topologyField, topology, "TOPOLOGY");
  topology.setAutoSizing(false);

  FFA_FIELD_DEFAULT_INIT(towerFile,"TOWER_FILE");

  // Add the base ID to the list of fields to be saved in the model file,
  // because some result items are associated with the turbine object.
  // The base ID should therefore preserve its value from session to session.
  FFA_FIELD_INIT(myBaseID, -1, "BASE_ID");

  if (createSubAssemblies)
  {
    this->connect();

    FmSubAssembly* obj;
    if (createSubAssemblies == 'T')
    {
      obj = new FmTower();
      obj->setParentAssembly(this);
      obj->connect();
    }

    FmSubAssembly* nac = new FmNacelle();
    nac->setParentAssembly(this);
    nac->connect();

    obj = new FmShaft(false,1.8,0.5);
    obj->setParentAssembly(nac);
    obj->connect();

    obj = new FmGearBox();
    obj->setParentAssembly(nac);
    obj->connect();

    obj = new FmShaft(false,0.5,0.3);
    obj->setParentAssembly(nac);
    obj->connect();

    obj = new FmGenerator();
    obj->setParentAssembly(nac);
    obj->connect();

    obj = new FmRotor();
    obj->setParentAssembly(this);
    obj->connect();
  }
}


bool FmTurbine::eraseOptions()
{
  // Bugfix #45: Deleting the functions of the control system first (if any).
  // Workaround to avoid the crash when erasing the whole model (or turbine).
  std::vector<FmModelMemberBase*> engs;
  FmDB::getAllOfType(engs,FmEngine::getClassTypeID(),this);
  for (FmModelMemberBase* obj : engs)
    obj->erase();

  return this->FmSubAssembly::eraseOptions();
}


void FmTurbine::getParts(FmTower*& tower,
			 FmNacelle*& nacelle,
			 FmGenerator*& generator,
			 FmGearBox*& gearbox,
			 FmShaft*& lsShaft,
			 FmShaft*& hsShaft,
			 FmRotor*& rotor) const
{
  std::vector<FmModelMemberBase*> allAss;
  std::vector<FmModelMemberBase*>::const_iterator it;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID(),this);

  tower = NULL;
  for (it = allAss.begin(); it != allAss.end() && !tower; it++)
    tower = dynamic_cast<FmTower*>(*it);

  nacelle = NULL;
  for (it = allAss.begin(); it != allAss.end() && !nacelle; it++)
    nacelle = dynamic_cast<FmNacelle*>(*it);

  rotor = NULL;
  for (it = allAss.begin(); it != allAss.end() && !rotor; it++)
    rotor = dynamic_cast<FmRotor*>(*it);

  generator = NULL;
  for (it = allAss.begin(); it != allAss.end() && !generator; it++)
    generator = dynamic_cast<FmGenerator*>(*it);

  gearbox = NULL;
  for (it = allAss.begin(); it != allAss.end() && !gearbox; it++)
    gearbox = dynamic_cast<FmGearBox*>(*it);

  // Here we make the assumption that, if the turbine has (at least) two
  // shaft assemblies, the low-speed shaft has the lowest user ID of the two
  lsShaft = hsShaft = NULL;
  for (it = allAss.begin(); it != allAss.end() && !lsShaft;)
    if ((lsShaft = dynamic_cast<FmShaft*>(*it)))
      while (++it != allAss.end() && !hsShaft)
	hsShaft = dynamic_cast<FmShaft*>(*it);
    else
      ++it;
}


FmTower* FmTurbine::getTower() const
{
  FmTower* tower = NULL;
  std::vector<FmModelMemberBase*> allAss;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID(),this);
  for (FmModelMemberBase* ass : allAss)
    if ((tower = dynamic_cast<FmTower*>(ass)))
      break;

  return tower;
}


FmBladeDesign* FmTurbine::getBladeProperties(std::vector<FmBladeProperty*>& bprop) const
{
  FmBladeDesign* bdef = dynamic_cast<FmBladeDesign*>(bladeDef.getPointer());
  if (bdef == NULL)
    bprop.clear();
  else
    bdef->getBladeSegments(bprop);

  return bdef;
}


double FmTurbine::getRadius(const FaVec3& X) const
{
  FmIsPositionedBase* apex = dynamic_cast<FmIsPositionedBase*>(topology[3].getPointer());
  if (!apex) return 0.0;

  return (X - apex->getGlobalCS().translation()).length() - this->getHubRadius();
}


double FmTurbine::getHubRadius() const
{
  FmRotor* rotor = NULL;
  std::vector<FmModelMemberBase*> allAss;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID(),this);
  for (FmModelMemberBase* ass : allAss)
    if ((rotor = dynamic_cast<FmRotor*>(ass)))
      return rotor->HubDiam.getValue()*0.5;

  return 0.0;
}


double FmTurbine::getHubHeight() const
{
  FmIsPositionedBase* ref = dynamic_cast<FmIsPositionedBase*>(topology[0].getPointer());
  FmIsPositionedBase* hub = dynamic_cast<FmIsPositionedBase*>(topology[3].getPointer());
  if (!hub)
    hub = dynamic_cast<FmIsPositionedBase*>(topology[2].getPointer());

  double HH = this->ptfmRef.getValue();
  if (ref && hub)
    HH += (this->toLocal(hub->getGlobalCS()[3]) -
           this->toLocal(ref->getGlobalCS()[3])).z();
  else if (hub)
    HH += this->toLocal(hub->getGlobalCS()[3]).z();

  return HH;
}


double FmTurbine::getRotorSize() const
{
  double hDiam = 0.0;
  double bLength = 0.0;
  FmBlade* blade = NULL;
  FmRotor* rotor = NULL;
  std::vector<FmModelMemberBase*> allAss;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID(),this);
  for (FmModelMemberBase* ass : allAss)
    if ((blade = dynamic_cast<FmBlade*>(ass)))
    {
      double L = blade->getTotalLength();
      if (L > bLength) bLength = L;
    }
    else if ((rotor = dynamic_cast<FmRotor*>(ass)))
      hDiam = rotor->HubDiam.getValue();

  return hDiam + 2.0*bLength;
}


void FmTurbine::draw() const
{
  FmDB::displayAll(this->getHeadMap());
}


const char** FmTurbine::getListViewPixmap() const
{
  return windTurbine_xpm;
}


std::ostream& FmTurbine::writeFMF(std::ostream& os)
{
  os <<"TURBINE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmTurbine::readAndConnect(std::istream& is, std::ostream&)
{
  FmTurbine* obj = new FmTurbine();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This turbine assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


int FmTurbine::printSolverEntry(FILE* fp)
{
  if (topology.empty()) return 0; // Probably an old model file (pre R6.0)

  std::vector<FmModelMemberBase*> subAss;
  FmDB::getAllOfType(subAss,FmSubAssembly::getClassTypeID(),this);

  int err = 0;
  FmRotor* rotor = NULL;
  for (FmModelMemberBase* obj : subAss)
    if ((rotor = dynamic_cast<FmRotor*>(obj)))
      break;

  if (!rotor)
  {
    ListUI <<"\n---> ERROR: No rotor assembly in "<< this->getIdString(true)
	   <<".\n            Wind loads will not be included in the dynamic analysis.\n";
    return 0;
  }

  size_t b, nBlades = nBlade.getValue();
  std::vector<FmModelMemberBase*> pitch;
  FmDB::getAllOfType(pitch,FmRevJoint::getClassTypeID(),rotor);
  if (pitch.size() < nBlades) // Should normally not happen
  {
    ListUI <<"\n---> ERROR: "<< this->getIdString(true) <<" has "
	   << (pitch.empty() ? "no" : "too few") <<" pitch joints.\n";
    nBlades = pitch.size();
    err++;
  }
  if (err) return err;

  int topID[5];
  for (size_t i = 0; i < 5; i++)
    if (i < topology.size() && !topology[i].isNull())
      topID[i] = topology[i]->getBaseID();
    else
    {
      ListUI <<"\n---> ERROR: "<< this->getIdString(true)
	     <<" has insufficient topology definition.\n"
	     <<"            Wind loads will not be included in the dynamic analysis.\n"
	     <<"            Check the \"Advanced topology settings\" fields in the Turbine property view.\n";
      return 0;
    }

  fprintf(fp,"&TURBINE_CONFIG\n");
  this->printID(fp);
  fprintf(fp,"  ADFile = \'fedem_aerodyn.ipt\'\n");
  fprintf(fp,"  PtfmRef = %f HubRad = %f\n", this->ptfmRef.getValue(),this->getHubRadius());
  fprintf(fp,"  towerTriad = %d nacelleTriad = %d hubTriad = %d\n", topID[0],topID[1],topID[2]);
  fprintf(fp,"  hubId = %d generatorJoint = %d", topID[3],topID[4]);

  std::vector<FmBladeProperty*> blades;
  this->getBladeProperties(blades);

  if (nBlades > 0)
    fprintf(fp," pitchJoint =");
  for (b = 0; b < nBlades; b++)
    fprintf(fp," %d",pitch[b]->getBaseID());

  size_t nTB = blades.empty() ? 0 : 2*blades.size(); // Assuming two beam elements per segment
  fprintf(fp,"\n  nBlade = %u nTB = %u firstTriadID =", (unsigned int)nBlades,(unsigned int)nTB);
  for (b = 0; b < nBlades; b++)
    fprintf(fp," %d",static_cast<FmRevJoint*>(pitch[b])->getSlaveTriad()->getBaseID());

  if (!blades.empty())
  {
    Doubles AC = blades.front()->getAeroCentre();
    fprintf(fp,"\n  ADcentre = %17.9e %17.9e", AC.first, AC.second);
    for (b = 1; b < blades.size(); b++)
    {
      AC = blades[b]->getAeroCentre();
      fprintf(fp,"\n             %17.9e %17.9e", AC.first, AC.second);
    }
    // Assume that every second triad along the blade is an AeroDyn node
    fprintf(fp,"\n  ADnodes =");
    for (b = 0; b < blades.size(); b++)
      fprintf(fp," 1 0");
  }

  bool haveWind = false;
  FmAirState* air = FmDB::getAirStateObject();
  if (air->useWindFile.getValue())
    haveWind = !air->windFile.getValue().empty();
  else
    haveWind = air->windSpeed.getValue() != 0.0;

  fprintf(fp,"\n  CompAero = .%s.", haveWind ? "true" : "false");
  fprintf(fp," UserID = .false.\n/\n\n");

  return 0;
}


bool FmTurbine::writeBladeElement(FILE* fp, FmLink* beam, int& propId)
{
  // Check if this beam is a turbine blade element
  FmBlade* blade = dynamic_cast<FmBlade*>(beam->getParentAssembly());
  if (!blade) return false;

  FmModelMemberBase* prop = beam->getProperty();
  if (!prop) return false;

  FmBladeProperty* bprop = dynamic_cast<FmBladeProperty*>(prop);
  FmBladeDesign* blDef = NULL;
  if (!prop->hasReferringObjs(blDef,"segment"))
    blDef = dynamic_cast<FmBladeDesign*>(prop);

  FmTurbine* turbine = NULL;
  if (blDef)
    blDef->hasReferringObjs(turbine,"bladeDef");
  else if (prop->getUserDescription() != "Blade property")
    return false;
  else
    for (FmBase* o = blade->getParentAssembly(); o; o = o->getParentAssembly())
      if ((turbine = dynamic_cast<FmTurbine*>(o)))
	break;

  if (!turbine) return false;

  std::vector<FmTriad*> triads;
  beam->getTriads(triads);
  if (triads.size() < 2) return false;

  if (beam->printSolverEntry(fp, blDef ? propId : prop->getBaseID()))
    return false;

  Doubles R(0.0,0.0);
  double bLength = (triads.back()->getLocalTranslation(beam) -
                    triads.front()->getLocalTranslation(beam)).length();

  // Initial triad positions in element coordinate system
  for (FmTriad* triad : triads)
  {
    triad->printLocalPos(fp,beam,0,false);

    // Check for beam end eccentricities
    Doubles ElC, CoG;
    if (bprop)
    {
      ElC = bprop->getElasticCentre();
      CoG = bprop->getMassCentre();
    }
    else if (blDef)
    {
      R.second = turbine->getRadius(triad->getGlobalTranslation());
      if (R.first == 0.0) R.first = R.second;
      blDef->getEccen(R.second,ElC,CoG);
    }
    else
    {
      ElC = std::make_pair(0.0,0.0);
      CoG = std::make_pair(0.0,0.0);
    }

    FaVec3 lPos = beam->getGlobalCS().inverse() * triad->getGlobalTranslation();
    ElC.first  += lPos.y();
    ElC.second += lPos.z();
    if (hypot(ElC.first,ElC.second) > bLength*0.001)
      fprintf(fp,"  eccVec  =%17.9e %17.9e %17.9e\n",-ElC.first,-ElC.second,0.0);
    if (hypot(CoG.first,CoG.second) > bLength*0.001)
      fprintf(fp,"  eccMass =%17.9e %17.9e %17.9e\n",-CoG.first,-CoG.second,0.0);
    fprintf(fp,"/\n");
  }
  fprintf(fp,"\n");

  if (!blDef) return true;

  // Check for ice
  double iceMass = 0.0;
  if (blade->IceLayer.getValue())
    iceMass = blade->IceThickness.getValue()*turbine->rhoIce.getValue();

  double data[10];
  if (bprop)
    bprop->getStructData(data,iceMass);
  else
    blDef->getStructData(0.5*(R.first+R.second),data,iceMass);

  fprintf(fp,"'Turbine blade properties\n");
  fprintf(fp,"&ELEMENT_PROPERTY\n");
  fprintf(fp,"  id = %d\n", propId++);
  fprintf(fp,"  geometry = %17.9e %17.9e %17.9e %17.9e %17.9e %17.9e %f %f\n",
	  data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  // Notice: A negative Youngs modulus, E (the second material parameter below),
  // is used to flag that the property parameters EA, EIy, EIz, GAs_y, GAs_z and
  // GIt are specified on the geometry entry, rather than A, Iy, Iz, It, etc.
  // The stiffness moduli E and G are then not used.
  fprintf(fp,"  material = %17.9e  -1.0  1.0        %17.9e\n", data[8],data[9]);
  fprintf(fp,"/\n\n");
  return true;
}


int FmTurbine::writeAeroDynFile(const std::string& fileName) const
{
  // Check if it is an old model with external AeroDyn file reference
  FmGenericDBObject* config = NULL;
  std::vector<FmModelMemberBase*> objs;
  FmDB::getAllOfType(objs,FmGenericDBObject::getClassTypeID(),this);

  for (FmModelMemberBase* obj : objs)
    if ((config = dynamic_cast<FmGenericDBObject*>(obj)))
      if (config->objectType.getValue() == "TURBINE_CONFIG" &&
	  config->objectDefinition.getValue().find("ADFile") != std::string::npos)
	return 0;

  FILE* fp = fopen(fileName.c_str(),"w");
  if (!fp)
  {
    ListUI <<"\n---> ERROR: Unable to write AeroDyn file "<< fileName <<"\n";
    return 2;
  }

  // Write out air state data to AeroDyn
  FmAirState* air = FmDB::getAirStateObject();
  fprintf(fp,"Aerodynamic properties generated by Fedem; Compatible with AeroDyn v12.58.\n");
  fprintf(fp,"SI          SysUnits    - System of units used for input and output\n");
  fprintf(fp,"%-12sStallMod    - Dynamic stall included\n",
	  air->stallMod.getValue().getText());
  fprintf(fp,"%-12sUseCm       - Use aerodynamic pitching moment model?\n",
	  air->useCM.getValue() ? "USE_CM" : "NO_CM");
  fprintf(fp,"%-12sInfModel    - Inflow model\n",
	  air->infMod.getValue().getText());
  fprintf(fp,"%-12sIndModel    - Induction-factor model\n",
	  air->indMod.getValue().getText());
  fprintf(fp,"%10g  AToler      - Induction-factor tolerance (convergence criteria)\n",
	  air->aToler.getValue());
  fprintf(fp,"%-12sTLModel     - Tip-loss model (EQUIL only)\n",
	  air->tlMod.getValue().getText());
  fprintf(fp,"%-12sHLModel     - Hub-loss model (EQUIL only)\n",
	  air->hlMod.getValue().getText());

  int err = 0;
  std::string windFile;
  if (air->useWindFile.getValue() && !air->windFile.getValue().empty())
  {
    windFile = air->windFile.getValue();
    FFaFilePath::makeItAbsolute(windFile,FmSimulationModelBase::relPathCorrection);
  }
  else
  {
    // Write an internal wind file assuming a constant wind speed.
    // Note that the solver requires a wind file even in the case of no wind.
    windFile = fileName;
    windFile.replace(windFile.size()-3,windFile.size(),"wnd");
    FILE* fwp = fopen(windFile.c_str(),"w");
    if (fwp)
    {
      fprintf(fwp,"! Wind file generated by Fedem.\n"
	      "! Time  Wind     Wind    Vert.    Horiz.   Vert.    LinV    Gust\n"
	      "!       Speed    Dir     Speed    Shear    Shear    Shear   Speed\n"
	      "  0.0%9.3f%7.2f    0.0      0.0      0.0      0.0     0.0\n",
	      air->windSpeed.getValue(), air->windDirection.getValue());
      // Due to a bug in AeroDyn, the wnd-file always needs to have two lines
      fprintf(fwp,"  0.0%9.3f%7.2f    0.0      0.0      0.0      0.0     0.0\n",
	      air->windSpeed.getValue(), air->windDirection.getValue());
      fclose(fwp);
    }
    else
    {
      ListUI <<"\n---> ERROR: Unable to write wind file "<< windFile <<"\n";
      err = 3;
    }

    size_t slashPos = windFile.find_last_of("/\\");
    if (slashPos < windFile.size())
      windFile = windFile.substr(slashPos+1);
  }
  fprintf(fp,"\"%s\" WindFile - Name of file containing wind data\n",
	  windFile.c_str());

  fprintf(fp,"%10g  HH          - Wind reference (hub) height [m]\n",
          this->getHubHeight());
  fprintf(fp,"NEWTOWER    TwrShad     - New tower influence model\n");
  fprintf(fp,"%-12sTwrPotent   - Calculate tower potential flow?\n",
	  air->twrPot.getValue() ? "True" : "False");
  fprintf(fp,"%-12sTwrShadow   - Calculate tower shadow?\n",
	  air->twrShad.getValue() ? "True" : "False");
  if (!towerFile.getValue().empty()) {
    std::string twrFile = towerFile.getValue();
    FFaFilePath::makeItAbsolute(twrFile,FmSimulationModelBase::relPathCorrection);
    fprintf(fp,"\"%12s\" TwrFile - Tower drag file name\n", twrFile.c_str());
  }
  else if (air->twrPot.getValue() || air->twrShad.getValue())
  {
    ListUI <<"\n---> ERROR: No tower drag file has been specified.\n";
    err += 4;
  }
  else
    fprintf(fp,"%12sTwrFile     - Tower drag file name\n"," ");
  fprintf(fp,"%10g  AirDens     - Air density [kg/m^3]\n",
	  air->airDens.getValue());
  fprintf(fp,"%10g  KinVisc     - Kinematic air viscosity [m^2/sec]\n",
	  air->kinVisc.getValue());

  double dt = air->dtAero.getValue();
  if (air->useDSdt.getValue())
    dt = FmDB::getActiveAnalysis()->timeIncr.getValue();
  fprintf(fp,"%10g  DTAero      - Time interval for aerodynamic calculations [sec]\n",dt);

  // Count the airfoil files currently in use
  std::vector<FmBladeProperty*> blades;
  FmBladeDesign* blDef = this->getBladeProperties(blades);
  std::map<std::string,int> fileSet;
  for (FmBladeProperty* blade : blades)
    fileSet[blade->AirFoil.getValue()] = 0;
  size_t nFoils = 0;
  for (std::pair<const std::string,int>& file : fileSet)
    file.second = ++nFoils;
  fprintf(fp,"%10u  NumFoil     - Number of airfoil files", (unsigned int)nFoils);

  // Define path to the active airfoil folder
  std::string aPath = FFaFilePath::getBaseName(blDef->getModelFileName()) + "_airfoils";

  // Write out the airfoil files with absolute path
  std::vector<std::string> airfoilsNotFound;
  for (const std::pair<const std::string,int>& file : fileSet)
  {
    std::string afFile = FFaFilePath::appendFileNameToPath(aPath,file.first);
    // Check if file exists
    if (!FmFileSys::isFile(afFile))
      airfoilsNotFound.push_back(afFile);
    fprintf(fp,"\n\"%s\"",afFile.c_str());
    if (file.first == fileSet.begin()->first)
      fprintf(fp," FoilNm - Names of the airfoil files [NumFoil lines]");
  }
  fprintf(fp,"\n");

  if (!airfoilsNotFound.empty())
  {
    ListUI <<"\n---> ERROR: The following airfoil files can not not be found:";
    for (const std::string& airfoil : airfoilsNotFound)
      ListUI <<"\n            "<< airfoil;
    ListUI <<"\n     Make sure all airfoils are located in the folder \""
           << aPath <<"\".\n";
    err += 8;
  }

  // Find the assembly of the first rotor blade
  double hubDiam = 0.0;
  FmBlade* blade = NULL;
  FmRotor* rotor = NULL;
  std::vector<FmModelMemberBase*> allAss;
  FmDB::getAllOfType(allAss,FmSubAssembly::getClassTypeID(),this);
  for (FmModelMemberBase* ass : allAss)
    if ((blade = dynamic_cast<FmBlade*>(ass)))
      break;
    else if ((rotor = dynamic_cast<FmRotor*>(ass)))
      hubDiam = rotor->HubDiam.getValue();

  // Now write out the blade node data. Note that we her use the triad positions
  // (of the first blade) to derive the blade section length instead of using
  // the data in the blade design object, to account for possible prebending.
  std::vector<FmTriad*> triads;
  if (blade)
    FmDB::getAllTriads(triads,blade,true);
  else
    err += 16;

  unsigned int nNode = triads.size()/2; // Assuming every second triad is an AeroDyn node
  fprintf(fp,"%10u  BldNodes    - Number of blade nodes used for analysis\n", nNode);
  fprintf(fp,"  RNodes AeroTwst  DRNodes   Chord  NFoil  PrnElm\n");

  FaVec3 v0;
  if (!triads.empty()) v0 = triads.front()->getLocalTranslation();
  double RNode = 0.5*hubDiam;
  double DRprv = 0.0;
  for (size_t i = 1; i < triads.size(); i += 2) // Loop over the AeroDyn nodes
  {
    size_t j = i / 2;
    FaVec3 v1 = triads[i]->getLocalTranslation();
    double DR = (v1-v0).length();
    RNode += DR;
    DR *= 2.0;
    if (i > 1) DR -= DRprv;
    fprintf(fp,"%8.4f%9.3f%9.4f%8.4f%7d  PRINT\n", RNode,
	    blades[j]->Twist.getValue(), DR,
	    blades[j]->Chord.getValue(), fileSet[blades[j]->AirFoil.getValue()]);
    DRprv = DR;
    v0 = v1;
  }

  fclose(fp);
  return err;
}


void FmTurbinePart::initFields(double mass)
{
  static Doubles stif(1.0e14,1.0e14); // Default generic part stiffness

  FFA_FIELD_INIT(Stiff,stif,"STIFFNESS");
  FFA_FIELD_INIT(Mass ,mass,"MASS");

  FFA_FIELD_DEFAULT_INIT(CoG    ,"CENTRE_OF_GRAVITY");
  FFA_FIELD_DEFAULT_INIT(Inertia,"INERTIA");
  FFA_FIELD_DEFAULT_INIT(Iaxes  ,"INERTIA_AXES");
}


Fmd_SOURCE_INIT(FcTOWER, FmTower, FmSubAssembly);


FmTower::FmTower(bool isDummy) : FmTurbinePart(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT        (Height,60.0,"HEIGHT");
  FFA_FIELD_INIT        (Thick , 0.1,"WALL_THICKNESS");
  FFA_FIELD_DEFAULT_INIT(segments   ,"BEAM_SEGMENTS");

  FFA_REFERENCE_FIELD_INIT(materialField, material, "MATERIAL");

  this->initFields(5.0e4);

  FFA_FIELD_INIT(visualize3Dts,1,"VISUALIZE3D");
}


const char** FmTower::getListViewPixmap() const
{
  return windTower_xpm;
}


std::ostream& FmTower::writeFMF(std::ostream& os)
{
  os <<"TOWER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmTower::readAndConnect(std::istream& is, std::ostream&)
{
  FmTower* obj = new FmTower();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This tower assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcNACELLE, FmNacelle, FmSubAssembly);


FmNacelle::FmNacelle(bool isDummy) : FmTurbinePart(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  // Define some suitable default values
  FFA_FIELD_INIT(B1, 0.8,"B1"); // Main bearing location
  FFA_FIELD_INIT(B2, 4.0,"B2"); // Second bearing location
  FFA_FIELD_INIT(C1, 0.8,"C1"); // Brake location
  FFA_FIELD_INIT(M2, 2.5,"M2"); // yaw-axis shaft intersection
  FFA_FIELD_INIT(M3, 5.0,"M3"); // Shaft length from yaw axis

  this->initFields(3.0e4);
}


const char** FmNacelle::getListViewPixmap() const
{
  return windNacelle_xpm;
}


std::ostream& FmNacelle::writeFMF(std::ostream& os)
{
  os <<"NACELLE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmNacelle::readAndConnect(std::istream& is, std::ostream&)
{
  FmNacelle* obj = new FmNacelle();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This nacelle assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcGENERATOR, FmGenerator, FmSubAssembly);


FmGenerator::FmGenerator(bool isDummy) : FmTurbinePart(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(Length,1.0,"LENGTH");

  this->initFields(2.0e3);
}


const char** FmGenerator::getListViewPixmap() const
{
  return windGenerator_xpm;
}


std::ostream& FmGenerator::writeFMF(std::ostream& os)
{
  os <<"GENERATOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmGenerator::readAndConnect(std::istream& is, std::ostream&)
{
  FmGenerator* obj = new FmGenerator();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This generator assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcGEARBOX, FmGearBox, FmSubAssembly);


FmGearBox::FmGearBox(bool isDummy) : FmTurbinePart(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(Ratio,97.0,"TRANSMISSION_RATIO");
  FFA_FIELD_INIT(Length,1.0,"LENGTH");
  FFA_FIELD_INIT(O1,0.0,"O1");
  FFA_FIELD_INIT(O2,0.2,"O2");

  this->initFields(5.0e2);
}


const char** FmGearBox::getListViewPixmap() const
{
  return windGears_xpm;
}


std::ostream& FmGearBox::writeFMF(std::ostream& os)
{
  os <<"GEARBOX\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmGearBox::readAndConnect(std::istream& is, std::ostream&)
{
  FmGearBox* obj = new FmGearBox();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This gearbox assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcSHAFT, FmShaft, FmSubAssembly);


FmShaft::FmShaft(bool isDummy, double OD, double ID, double L) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(Tilt,5.0,"ANGLE");
  FFA_FIELD_INIT(Length,L,"LENGTH");

  FFA_FIELD_INIT(visualize3Dts,1,"VISUALIZE3D");

  FFA_FIELD_INIT(Do,     OD,"OUTER_DIAMETER");
  FFA_FIELD_INIT(Di,     ID,"INNER_DIAMETER");
  FFA_FIELD_INIT(Rho,7850.0,"MASS_DENSITY");
  FFA_FIELD_INIT(E,  2.1e11,"YOUNGS_MODULUS");
  FFA_FIELD_INIT(G,  8.1e10,"SHEAR_MODULUS");
}


const char** FmShaft::getListViewPixmap() const
{
  return this->getID() <= 2 ? windShaftBlue_xpm : windShaftOrange_xpm;
}


std::ostream& FmShaft::writeFMF(std::ostream& os)
{
  os <<"SHAFT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmShaft::readAndConnect(std::istream& is, std::ostream&)
{
  FmShaft* obj = new FmShaft();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This shaft assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcROTOR, FmRotor, FmSubAssembly);


FmRotor::FmRotor(bool isDummy) : FmTurbinePart(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(PreCone,-2.5,"A1"); // Precone angle [deg]
  FFA_FIELD_INIT(HubDiam, 3.0,"D1"); // Hub diameter
  FFA_FIELD_INIT(HubApex, 0.2,"N1"); // Distance from shaft end to hub apex

  this->initFields(5.0e3);
}


const char** FmRotor::getListViewPixmap() const
{
  return windRotor_xpm;
}


std::ostream& FmRotor::writeFMF(std::ostream& os)
{
  os <<"ROTOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmRotor::readAndConnect(std::istream& is, std::ostream&)
{
  FmRotor* obj = new FmRotor();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This rotor assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


Fmd_SOURCE_INIT(FcBLADE, FmBlade, FmSubAssembly);


FmBlade::FmBlade(bool isDummy) : FmAssemblyBase(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_FIELD_INIT(IceLayer,  false,"ICE_LAYER");
  FFA_FIELD_INIT(IceThickness,0.0,"ICE_THICKNESS");

  FFA_FIELD_INIT(visualize3Dts,1,"VISUALIZE3D");
}


const char** FmBlade::getListViewPixmap() const
{
  return windBladeProp_xpm;
}


std::ostream& FmBlade::writeFMF(std::ostream& os)
{
  os <<"BLADE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  if (myModelFile.getValue().empty())
    FmDB::reportMembers(os,this->getHeadMap());
  else
    this->FmSubAssembly::writeFMF(myModelFile.getValue());

  return os;
}


bool FmBlade::readAndConnect(std::istream& is, std::ostream&)
{
  FmBlade* obj = new FmBlade();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!obj->connect())
    // This blade assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  return FmSubAssembly::readFMF(obj->myModelFile.getValue());
}


FmJointBase* FmBlade::getPitchJoint() const
{
  FmJointBase* pitch = NULL;
  std::vector<FmTriad*> triads;
  FmDB::getAllTriads(triads,this,true);
  for (FmTriad* triad : triads)
    if ((pitch = triad->getJointWhereSlave()))
      break;

  return pitch;
}
