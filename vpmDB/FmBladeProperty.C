// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmBladeProperty.H"
#include "vpmDB/FmTurbine.H"
#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmDB.H"

#include "vpmDB/FmFileSys.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "Admin/FedemAdmin.H"
#include <fstream>


/*!
  \brief Returns the circumference of an ellipse.

  The formula is taken from http://en.wikipedia.org/wiki/Ellipse.
*/

static double ellipsis(double a, double BoverA)
{
  if (BoverA < 0.0001) return a+a;

  double b = a*BoverA;
  double c = 0.5*(a-b)/(a+b);
  return M_PI*(a+b)*(0.5 + 1.5*c*c/(10.0+sqrt(4.0-3*c*c)));
}


Doubles operator* (const Doubles& a, double b)
{
  return std::make_pair(a.first*b,a.second*b);
}

Doubles operator+ (const Doubles& a, const Doubles& b)
{
  return std::make_pair(a.first+b.first,a.second+b.second);
}

Doubles operator- (const Doubles& a, const Doubles& b)
{
  return std::make_pair(a.first-b.first,a.second-b.second);
}


Fmd_DB_SOURCE_INIT(FcTURBINE_BLADE_PROPERTY, FmBladeProperty, FmStructPropertyBase);


FmBladeProperty::FmBladeProperty()
{
  Fmd_CONSTRUCTOR_INIT(FmBladeProperty);

  const Doubles Zero(0.0,0.0);

  FFA_FIELD_INIT(ElCentre ,Zero,"ELASTIC_CENTRE");
  FFA_FIELD_INIT(ElAxisRot, 0.0,"ELASTIC_AXIS_ANGLE");
  FFA_FIELD_INIT(ShrCentre,Zero,"SHEAR_CENTRE");
  FFA_FIELD_INIT(EA       , 0.0,"AXIAL_STIFFNESS");
  FFA_FIELD_INIT(EI       ,Zero,"BENDING_STIFFNESS");
  FFA_FIELD_INIT(GAs      ,Zero,"SHEAR_STIFFNESS");
  FFA_FIELD_INIT(GIt      , 0.0,"TORSIONAL_STIFFNESS");

  FFA_FIELD_INIT(MassCentre,Zero,"MASS_CENTRE");
  FFA_FIELD_INIT(Mass      , 0.0,"MASS_DENSITY");
  FFA_FIELD_INIT(Tinertia  , 0.0,"TORSIONAL_INTERTIA");

  FFA_FIELD_INIT(PitchCentre,Zero,"PITCH_CENTRE");
  FFA_FIELD_INIT(AeroCentre , 0.0,"AERODYNAMIC_CENTRE");
  FFA_FIELD_INIT(Twist      , 0.0,"AERODYNAMIC_TWIST");
  FFA_FIELD_INIT(Chord      , 0.0,"CHORD_LENGTH");
  FFA_FIELD_INIT(Thick      , 0.0,"THICKNESS_RATIO");
  FFA_FIELD_INIT(Length     , 0.0,"LENGTH");
  FFA_FIELD_DEFAULT_INIT(AirFoil ,"AIRFOIL_FILE");
}


FmBladeProperty::~FmBladeProperty()
{
  this->disconnect();
}


std::ostream& FmBladeProperty::writeFMF(std::ostream& os)
{
  os <<"TURBINE_BLADE_PROPERTY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  return os;
}


bool FmBladeProperty::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmBladeProperty::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmBladeProperty::getClassTypeID());
}


static std::map<int,int> oldToNewBP;

FmBladeProperty* FmBladeProperty::readAndConnect(std::istream& is, std::ostream&)
{
  FmBladeProperty* obj = new FmBladeProperty();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  if (!oldToNewBP.empty())
  {
    // Check if the ID of its parent assembly has been changed
    std::vector<int> assID;
    obj->getAssemblyID(assID);
    if (!assID.empty())
    {
      std::map<int,int>::const_iterator it = oldToNewBP.find(assID.back());
      if (it != oldToNewBP.end())
	obj->setParentAssembly(it->second,FmSubAssembly::getClassTypeID());
    }
  }

  obj->connect();
  return obj;
}


Doubles FmBladeProperty::getAeroCentre() const
{
  Doubles AC = Doubles(AeroCentre.getValue(),0.0) - PitchCentre.getValue();

  return AC * Chord.getValue();
}


Doubles FmBladeProperty::getElasticCentre() const
{
  Doubles EC = ElCentre.getValue() - PitchCentre.getValue();

  // Note the minus sign here because we need the pitch centre location
  // relative to the elastic centre (in the elastic axes orientation)
  // and not the vice versa. TODO: Verify the correctness of this.
  double ca = cos(RAD(-ElAxisRot.getValue()));
  double sa = sin(RAD(-ElAxisRot.getValue()));

  return Doubles(EC.first*ca - EC.second*sa,
                 EC.first*sa + EC.second*ca) * Chord.getValue();
}


Doubles FmBladeProperty::getShearCentre() const
{
  return ShrCentre.getValue() * Chord.getValue();
}


Doubles FmBladeProperty::getMassCentre() const
{
  return MassCentre.getValue() * Chord.getValue();
}


void FmBladeProperty::getStructData(double* data, double iceMass) const
{
  Doubles ShearC = this->getShearCentre();

  data[0] = EA.getValue();
  data[1] = EI.getValue().first;
  data[2] = EI.getValue().second;
  data[3] = GIt.getValue();
  data[4] = GAs.getValue().first;
  data[5] = GAs.getValue().second;
  data[6] = ShearC.first;
  data[7] = ShearC.second;
  data[8] = this->getStructMass(iceMass);
  data[9] = Tinertia.getValue();

  // Compute stiffness parameters from a massive cylinder if undefined
  FmBladeDesign* blDef = NULL;
  if (this->hasReferringObjs(blDef,"segment"))
  {
    double R = 0.5*Chord.getValue();
    double S = blDef->autoEmod.getValue()*R*R*M_PI;
    if (!blDef->withAstiff.getValue()) data[0] = S;
    if (!blDef->withBstiff.getValue()) data[1] = data[2] = 0.25*S*R*R;
    if (!blDef->withTstiff.getValue()) data[3] = 0.5*S*R*R;
    if (!blDef->withSstiff.getValue()) data[4] = data[5] = data[6] = data[7] = 0.0;
  }
}


double FmBladeProperty::getStructMass(double iceMass) const
{
  double mass = Mass.getValue();
  if (iceMass > 0.0) // Add mass due to ice layer
    mass += iceMass*ellipsis(Chord.getValue(),Thick.getValue());

  return mass;
}


Fmd_SOURCE_INIT(FcTURBINE_BLADE_DESIGN, FmBladeDesign, FmSubAssembly);


FmBladeDesign::FmBladeDesign(bool isDummy) : FmSubAssembly(isDummy)
{
  if (isDummy) return; // No fields in dummy objects

  FFA_REFERENCELIST_FIELD_INIT(segmentField, segment, "BLADE_SEGMENTS");

  FFA_FIELD_INIT(readOnly,false,"READ_ONLY");

  FFA_FIELD_INIT(autoEmod, 2.1e11,"AUTO_STIFFNESS");
  FFA_FIELD_INIT(withAstiff,false,"INCLUDE_AXIAL_STIFFNESS");
  FFA_FIELD_INIT(withBstiff,false,"INCLUDE_BENDING_STIFFNESS");
  FFA_FIELD_INIT(withTstiff,false,"INCLUDE_TORSION_STIFFNESS");
  FFA_FIELD_INIT(withSstiff,false,"INCLUDE_SHEAR_STIFFNESS");
}


bool FmBladeDesign::interactiveErase()
{
  if (!readOnly.getValue())
  {
    // Ask user if we also should remove the associated data file
    std::string path = myModelFile.getValue();
    FFaFilePath::makeItAbsolute(path,FmDB::getMechanismObject()->getPropertyLibPath(false));
    std::string msg = "Erasing " + this->getIdString(true);
    msg += ":\nDo you also want to erase the associated file ";
    msg += path;
    msg += " ?";
    switch (FFaMsg::dialog(msg,FFaMsg::YES_ALL_NO_ALL_CANCEL))
      {
      case 1: // yes
	FmFileSys::deleteFile(path);
        break;

      case 2: // cancel delete
        return false;
      }
  }

  return this->erase();
}


void FmBladeDesign::addBladeProperty(FmBladeProperty* bp, int pos)
{
  if (!bp) return;

  bp->setParentAssembly(this);

  if (pos < 0)
    segment.push_back(bp);
  else
    segment.insert(bp,pos);
}


bool FmBladeDesign::removeBladeProperty(int pos)
{
  FmBladeProperty* p = segment.getPtr(pos);
  if (!p) return false;

  segment.removePtr(p);
  p->erase();
  return true;
}


FmBladeProperty* FmBladeDesign::getBladeProperty(int pos)
{
  if (pos < 0 || pos >= (int)segment.size())
    return NULL;

  return segment.getPtr(pos);
}


void FmBladeDesign::setBladeSegments(const std::vector<FmBladeProperty*>& segs)
{
  for (FmBladeProperty* segment : segs)
    if (segment) segment->setParentAssembly(this);

  segment.setPtrs(segs);
}


void FmBladeDesign::getBladeSegments(std::vector<FmBladeProperty*>& segs) const
{
  segment.getPtrs(segs);
}


bool FmBladeDesign::getSegmentLengths(std::vector<double>& DRNode) const
{
  DRNode.clear();
  if (segment.empty()) return false;

  const double epsTol = 0.001;

  int err = 0;
#ifdef OLD_BLADE_FORMAT
  DRNode.reserve(segment.size()-1);
  double RNode = segment.getFirstPtr()->Length.getValue();
  DRNode.push_back(2.0*RNode);
  for (size_t i = 1; i < segment.size()-1; i++)
  {
    RNode += segment[i]->Length.getValue();
    DRNode.push_back(2.0*segment[i]->Length.getValue() - DRNode.back());
    if (DRNode.back() <= epsTol) err++;
  }
  if (segment.size() > 1)
  {
    double Llast = segment.getLastPtr()->Length.getValue();
    if (fabs(Llast - 0.5*DRNode.back()) > epsTol*Llast)
    {
      ListUI <<"\n===> ERROR: Final blade element length expected to be "
	     << 0.5*DRNode.back() <<", but it is "<< Llast;
      err++;
    }
  }
#else
  DRNode.reserve(segment.size());
  for (size_t i = 0; i < segment.size(); i++)
  {
    DRNode.push_back(segment[i]->Length.getValue());
    if (DRNode.back() <= epsTol) err++;
  }
#endif
  if (err == 0) return true;

  ListUI <<"\n===> ERROR: "<< err <<" invalid blade element lengths detected.\n";
  return false;
}


std::ostream& FmBladeDesign::writeFMF(std::ostream& os)
{
  // Write only the currently used blade design
  FmTurbine* turbine = NULL;
  if (!this->hasReferringObjs(turbine,"bladeDef"))
    return os;

  os <<"TURBINE_BLADE_DESIGN\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  FmDB::reportMembers(os,*this->getHeadMap());

  return os;
}


FmBladeDesign* FmBladeDesign::readAndConnect(std::istream& is, std::ostream&,
					     bool autoConnect)
{
  FmBladeDesign* obj = new FmBladeDesign();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  int oldID = obj->getID();
  if (autoConnect) obj->setID(0); // Assign a new ID to avoid connection trouble

  if (!obj->connect())
    // This blade design assembly already exists.
    // Most likely it was created by other objects owned by it.
    if (!obj->mergeOldHeadMapAndConnect())
      std::cerr <<"ERROR: Logic error while reading "<< obj->getIdString()
		<< std::endl;

  int newID = obj->getID();
  if (autoConnect && oldID != newID)
  {
    for (size_t i = 0; i < obj->segment.size(); i++)
    {
      std::vector<int> aID;
      int uID = obj->segment[i].getRefID();
      int tID = obj->segment[i].getRefTypeID();
      obj->segment[i].getRefAssemblyID(aID);
      if (aID.back() == oldID)
      {
	aID.back() = newID;
	obj->segment[i].setRef(uID,tID,aID);
      }
    }

    oldToNewBP[oldID] = newID;
    ListUI <<"     Blade design ["<< oldID <<"] \""
	   << obj->getUserDescription() <<"\" assigned new ID ["
	   << obj->getID() <<"]\n";
  }

  return obj;
}


bool FmBladeDesign::writeToFMM(const std::string& fileName) const
{
  if (fileName.empty()) return false;

  std::ofstream os(fileName.c_str(),std::ios::out);
  if (!os)
  {
    ListUI <<" ===> Failure opening blade file: "<< fileName <<"\n";
    return false;
  }

  os <<"FEDEMMODELFILE {" << FedemAdmin::getVersion() <<" ASCII}\n";
  os <<"!Module version: "<< FedemAdmin::getVersion() <<" "<< FedemAdmin::getBuildDate() <<"\n";
  os <<"!Model file name: "<< fileName <<"\n\n";

  os <<"TURBINE_BLADE_DESIGN\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  FmDB::reportMembers(os,*this->getHeadMap());

  os <<"END {FEDEMMODELFILE}\n";

  return os ? true : false;
}


FmBladeDesign* FmBladeDesign::readFromFMM(const std::string& fileName,
                                          bool setRdOnly, bool keepAll)
{
  std::ifstream fs(fileName.c_str(),std::ios::in);
  if (fs)
    ListUI <<"  => Reading blade file "<< fileName <<"\n";
  else
  {
    ListUI <<" ==> Non-existing blade file: "<< fileName <<"\n";
    return NULL;
  }

  // Consider only these entries here, everything else is silently ignored
  enum { FEDEMMODELFILE = 1,
	 TURBINE_BLADE_DESIGN = 2,
	 TURBINE_BLADE_PROPERTY = 3,
	 END = 4 };

  static const char* key_words[] = {
    "FEDEMMODELFILE",
    "TURBINE_BLADE_DESIGN",
    "TURBINE_BLADE_PROPERTY",
    "END"
  };

  bool dataIsRead = false;
  std::vector<FmBladeDesign*> blades;
  std::vector<FmBladeProperty*> props;

  oldToNewBP.clear();
  while (!fs.eof() && !dataIsRead)
  {
    std::stringstream statement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord,fs,statement,'{','}'))
      switch (FaParse::findIndex(key_words,keyWord))
	{
	case FEDEMMODELFILE:
	  break;
	case TURBINE_BLADE_DESIGN:
	  blades.push_back(FmBladeDesign::readAndConnect(statement,std::cout,true));
	  break;
	case TURBINE_BLADE_PROPERTY:
	  props.push_back(FmBladeProperty::readAndConnect(statement));
	  break;
	case END:
	  dataIsRead = true;
	  break;
	default:
	  ListUI <<" ==> WARNING: Ignored keyword: "<< keyWord <<"\n";
	  break;
	}
  }

  oldToNewBP.clear();
  if (!dataIsRead)
  {
    ListUI <<" ==> Failure reading blade file: "<< fileName <<"\n";
    for (FmBladeDesign* blade : blades) blade->erase();
    for (FmBladeProperty* prop : props) prop->erase();
    blades.clear();
    props.clear();
  }

  for (FmBladeDesign* blade : blades)
  {
    blade->readOnly.setValue(setRdOnly);
    blade->myModelFile.setValue(fileName);
    FmDB::resolveObject(blade);
  }

  for (FmBladeProperty* prop : props)
    FmDB::resolveObject(prop);

  // If the file contains more than one design, retain only the first one
  for (size_t i = 1; i < blades.size() && !keepAll; i++)
    blades[i]->erase();

  return blades.empty() ? NULL : blades.front();
}


size_t FmBladeDesign::getPropertyIdxAt(double& R) const
{
#ifdef OLD_BLADE_FORMAT
  double Rnode = segment.getFirstPtr()->Length.getValue();
  if (R < Rnode) return 0; // before the first data point

  for (size_t i = 1; i < segment.size(); i++)
    if (R-Rnode >= segment[i]->Length.getValue())
      Rnode += segment[i]->Length.getValue();
    else
    {
      R = (R-Rnode)/segment[i]->Length.getValue();
      return i;
    }
#else
  double Rnode = segment.getFirstPtr()->Length.getValue()*0.5;
  if (R < Rnode) return 0; // before the first data point

  for (size_t i = 1; i < segment.size(); i++)
  {
    double L = 0.5*(segment[i-1]->Length.getValue() + segment[i]->Length.getValue());
    if (R-Rnode >= L)
      Rnode += L;
    else
    {
      R = (R-Rnode)/L;
      return i;
    }
  }
#endif
  return segment.size();
}


bool FmBladeDesign::getEccen(double R, Doubles& ElC, Doubles& CoG) const
{
  if (segment.empty()) return false;

  // Find the proper segment based on R
  const FmBladeProperty* p = NULL;
  int idx = this->getPropertyIdxAt(R);

  if (idx <= 0)
    p = segment.getFirstPtr();
  else if (idx >= (int)segment.size())
    p = segment.getLastPtr();
  else if (R < 0.0001)
    p = segment[idx-1];
  else
  {
    // We have to interpolate
    const FmBladeProperty* p0 = segment[idx-1];
    const FmBladeProperty* p1 = segment[idx];
    Doubles EC0 = p0->getElasticCentre();
    Doubles EC1 = p1->getElasticCentre();
    Doubles MC0 = p0->getMassCentre();
    Doubles MC1 = p1->getMassCentre();

    ElC = EC0*(1.0-R) + EC1*R;
    CoG = MC0*(1.0-R) + MC1*R;
    return true;
  }

  ElC = p->getElasticCentre();
  CoG = p->getMassCentre();
  return true;
}


bool FmBladeDesign::getStructData(double R, double* data, double iceMass) const
{
  if (segment.empty()) return false;

  // Find the proper segment based on R
  const FmBladeProperty* p = NULL;
  int idx = this->getPropertyIdxAt(R);

  if (idx <= 0)
    p = segment.getFirstPtr();
  else if (idx >= (int)segment.size())
    p = segment.getLastPtr();
  else if (R < 0.0001)
    p = segment[idx-1];
  else
  {
    // We have to interpolate
    const FmBladeProperty* p0 = segment[idx-1];
    const FmBladeProperty* p1 = segment[idx];
    Doubles ShearC0 = p0->getShearCentre();
    Doubles ShearC1 = p1->getShearCentre();

    data[0] = (1.0-R)*p0->EA.getValue()
      +            R *p1->EA.getValue();
    data[1] = (1.0-R)*p0->EI.getValue().first
      +            R *p1->EI.getValue().first;
    data[2] = (1.0-R)*p0->EI.getValue().second
      +            R *p1->EI.getValue().second;
    data[3] = (1.0-R)*p0->GIt.getValue()
      +            R *p1->GIt.getValue();
    data[4] = (1.0-R)*p0->GAs.getValue().first
      +            R *p1->GAs.getValue().second;
    data[5] = (1.0-R)*p0->GAs.getValue().first
      +            R *p1->GAs.getValue().second;
    data[6] = (1.0-R)*ShearC0.first  + R*ShearC1.first;
    data[7] = (1.0-R)*ShearC0.second + R*ShearC1.second;
    data[8] = (1.0-R)*p0->getStructMass(iceMass)
      +            R *p1->getStructMass(iceMass);
    data[9] = (1.0-R)*p0->Tinertia.getValue()
      +            R *p1->Tinertia.getValue();
    R = 0.5*((1.0-R)*p0->Chord.getValue() + R*p1->Chord.getValue());
  }
  if (p)
  {
    Doubles ShearC = p->getShearCentre();

    data[0] = p->EA.getValue();
    data[1] = p->EI.getValue().first;
    data[2] = p->EI.getValue().second;
    data[3] = p->GIt.getValue();
    data[4] = p->GAs.getValue().first;
    data[5] = p->GAs.getValue().second;
    data[6] = ShearC.first;
    data[7] = ShearC.second;
    data[8] = p->getStructMass(iceMass);
    data[9] = p->Tinertia.getValue();
    R = 0.5*p->Chord.getValue();
  }

  // Compute stiffness parameters from a massive cylinder if undefined
  double S = autoEmod.getValue()*R*R*M_PI;
  if (!withAstiff.getValue()) data[0] = S;
  if (!withBstiff.getValue()) data[1] = data[2] = 0.25*S*R*R;
  if (!withTstiff.getValue()) data[3] = 0.5*S*R*R;
  if (!withSstiff.getValue()) data[4] = data[5] = data[6] = data[7] = 0.0;

  return true;
}


double FmBladeDesign::getStructMass(double R, double iceMass) const
{
  if (segment.empty()) return false;

  // Find the proper segment based on R
  int idx = this->getPropertyIdxAt(R);

  if (idx >= 0)
    return segment.getFirstPtr()->getStructMass(iceMass);
  else if (idx >= (int)segment.size())
    return segment.getLastPtr()->getStructMass(iceMass);
  else if (R < 0.0001)
    return segment[idx-1]->getStructMass(iceMass);
  else if (R > 0.9999)
    return segment[idx]->getStructMass(iceMass);

  // We have to interpolate
  double M0 = segment[idx-1]->getStructMass(iceMass);
  double M1 = segment[idx]->getStructMass(iceMass);
  return (1.0-R)*M0 + R*M1;
}


double FmBladeDesign::getElementMass(const FmBeam* beam) const
{
  FmTurbine* turbine = NULL;
  if (!this->hasReferringObjs(turbine,"bladeDef"))
    return 0.0; // should not happen, invoked for unused object

  FmTriad* tr1 = beam->getFirstTriad();
  FmTriad* tr2 = beam->getSecondTriad();
  if (!tr1 || !tr2)
    return 0.0;

  // Check for ice
  double iceMass = 0.0;
  FmBlade* blade = dynamic_cast<FmBlade*>(beam->getParentAssembly());
  if (blade && blade->IceLayer.getValue())
    iceMass = blade->IceThickness.getValue()*turbine->rhoIce.getValue();

  double totMass = 0.0;
  FmBladeProperty* prop = dynamic_cast<FmBladeProperty*>(beam->getProperty());
  if (prop)
    totMass = prop->getStructMass(iceMass);
  else
  {
    double R0 = turbine->getRadius(tr1->getGlobalTranslation());
    double R1 = turbine->getRadius(tr2->getGlobalTranslation());
    totMass = 0.5*(this->getStructMass(R0,iceMass)+this->getStructMass(R1,iceMass));
  }

  return totMass * beam->getLength();
}


std::string FmBladeDesign::getModelFileName() const
{
  std::string fName = myModelFile.getValue();
  FFaFilePath::makeItAbsolute(FFaFilePath::checkName(fName),
                              FmDB::getMechanismObject()->getAbsModelFilePath());
  return fName;
}
