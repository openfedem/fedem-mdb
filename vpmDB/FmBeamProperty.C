// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmBeamProperty.H"
#include "vpmDB/FmBeam.H"
#include "vpmDB/FmGenericDBObject.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaAlgebra/FFaMath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"


Fmd_DB_SOURCE_INIT(FcBEAM_PROPERTY, FmBeamProperty, FmStructPropertyBase);


FmBeamProperty::FmBeamProperty()
{
  Fmd_CONSTRUCTOR_INIT(FmBeamProperty);

  const Doubles Zero(0.0,0.0);

  FFA_FIELD_INIT(crossSectionType,PIPE,"TYPE");

  FFA_REFERENCE_FIELD_INIT(materialField, material, "MATERIAL");

  // Tube Cross Section
  FFA_FIELD_INIT(Do,0.5,"TUBE_DO");
  FFA_FIELD_INIT(Di,0.4,"TUBE_DI");

  // Generic Beam Cross Section
  FFA_FIELD_INIT(EA,  0.0,"GENERIC_EA");
  FFA_FIELD_INIT(EI, Zero,"GENERIC_EI");
  FFA_FIELD_INIT(GAs,Zero,"GENERIC_GAS");
  FFA_FIELD_INIT(GIt, 0.0,"GENERIC_GIT");
  FFA_FIELD_INIT(Mass,0.0,"GENERIC_MASS");
  FFA_FIELD_INIT(RoIp,0.0,"GENERIC_IP");

  // Dependent properties
  FFA_FIELD_INIT(breakDependence, false, "BREAK_DEPENDENCY");
  FFA_FIELD_INIT(A,  0.0,"AREA");
  FFA_FIELD_INIT(Iy, 0.0,"AREA_MOMENT_IY");
  FFA_FIELD_INIT(Iz, 0.0,"AREA_MOMENT_IZ");
  FFA_FIELD_INIT(Ip, 0.0,"AREA_MOMENT_IP");
  this->updateDependentValues();

  // Shear
  FFA_FIELD_INIT(ShrRed,Doubles(2.0,2.0),"SHEAR_REDUCTION");
  FFA_FIELD_INIT(ShrCentre,         Zero,"SHEAR_CENTRE");

  // Hydrodynamic properties
  FFA_FIELD_INIT(hydroToggle, false, "HYDRO_TOGGLE");
  FFA_FIELD_INIT(Di_hydro,0.0,"HYDRO_DI");
  FFA_FIELD_INIT(Db,      0.0,"HYDRO_DB");
  FFA_FIELD_INIT(Dd,      0.0,"HYDRO_DD");
  FFA_FIELD_INIT(Cd,      1.0,"HYDRO_CD");
  FFA_FIELD_INIT(Ca,      1.0,"HYDRO_CA");
  FFA_FIELD_INIT(Cm,      2.0,"HYDRO_CM");
  FFA_FIELD_INIT(Cd_axial,0.0,"HYDRO_CD_AXIAL");
  FFA_FIELD_INIT(Ca_axial,0.0,"HYDRO_CA_AXIAL");
  FFA_FIELD_INIT(Cm_axial,0.0,"HYDRO_CM_AXIAL");
  FFA_FIELD_INIT(Cd_spin, 0.0,"HYDRO_CD_SPIN");
}


FmBeamProperty::~FmBeamProperty()
{
  this->disconnect();
}


std::ostream& FmBeamProperty::writeFMF(std::ostream& os)
{
  os <<"BEAM_PROPERTY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  return os;
}


bool FmBeamProperty::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmBeamProperty::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmBeamProperty::getClassTypeID());
}


bool FmBeamProperty::readAndConnect(std::istream& is, std::ostream&)
{
  FmBeamProperty* obj = new FmBeamProperty();

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


void FmBeamProperty::updateDependentValues()
{
  switch (crossSectionType.getValue())
    {
    case PIPE: {
      double D1 = Do.getValue();
      double D2 = D1*D1;
      double D4 = D2*D2;
      double d1 = Di.getValue();
      double d2 = d1*d1;
      double d4 = d2*d2;
      A .setValue(M_PI*(D2-d2)/4.0);
      Ip.setValue(M_PI*(D4-d4)/32.0);
      Iy.setValue(Ip.getValue()/2.0);
      Iz.setValue(Iy.getValue());
      }
    default:
      break;
    }
}


int FmBeamProperty::printSolverEntry(FILE* fp)
{
  // Print only the used beam properties
  int err = 0;
  FmBeam* beam = 0;
  if (!this->hasReferringObjs(beam,"myProp"))
    return 0;

  fprintf(fp,"'Beam properties\n");
  fprintf(fp,"&ELEMENT_PROPERTY\n");
  this->printID(fp);
  if (crossSectionType.getValue() == GENERIC)
  {
    fprintf(fp,"  geometry = %17.9e %17.9e %17.9e %17.9e %17.9e %17.9e %f %f\n",
	    EA.getValue(), EI.getValue().first, EI.getValue().second,
	    GIt.getValue(), GAs.getValue().first, GAs.getValue().second,
	    ShrCentre.getValue().first, ShrCentre.getValue().second);
    // Note: A negative Youngs modulus, E (the second material parameter below),
    // is used to flag that the property parameters EA, EIy, EIz, GAsy, GAsz
    // and GIt are specified on the geometry entry, rather than A, Iy, Iz, etc.
    // The stiffness moduli E and G are then not used.
    fprintf(fp,"  material = %17.9e  -1.0  1.0",Mass.getValue());
    fprintf(fp,"        %17.9e\n",RoIp.getValue());
  }
  else
  {
    fprintf(fp,"  geometry = %17.9e %17.9e %17.9e %17.9e %f %f %f %f\n",
	    A.getValue(), Iy.getValue(), Iz.getValue(), Ip.getValue(),
	    ShrRed.getValue().first, ShrRed.getValue().second,
	    ShrCentre.getValue().first, ShrCentre.getValue().second);
    if (material.isNull())
    {
      err = 1;
      ListUI <<"  -> Error: No material has been assigned to "
	     << this->getIdString(true) <<"\n            which is used by "
	     << beam->getIdString(true) <<"\n";
    }
    else
      fprintf(fp,"  material = %17.9e %17.9e %17.9e\n",
	      material->Rho.getValue(),
	      material->E.getValue(), material->G.getValue());
  }
  if (hydroToggle.getValue())
  {
    double Cs[2] = { 0.0, 0.0 };
    FFaString(this->getUserDescription()).getDoublesAfter("#Cs",2,Cs);
    fprintf(fp,"  hydyn    = %f %f %f %f %f %f %f %f %f %f %f\n",
	    Ca.getValue(), Cm.getValue(), Cd.getValue(),
	    Dd.getValue(), Db.getValue(),
	    Cd_axial.getValue(), Cd_spin.getValue(),
	    Ca_axial.getValue(), Cm_axial.getValue(),
	    Cs[0], Cs[1]);
    double rhoInt = beam->getInternalFluidDensity();
    if (rhoInt > 0.0)
      fprintf(fp,"  rho_int  = %f  D_int  = %f\n", rhoInt, Di_hydro.getValue());
  }
  fprintf(fp, "/\n\n");
  return err;
}


static void parseProp(const std::string& prop, const char* keyword,
		      std::vector<double>& data)
{
  size_t i = prop.find(keyword);
  if (i == std::string::npos) return;
  size_t j = prop.find_first_of('\n',i);
  size_t n = strlen(keyword);
  if (j < i+n) return;

  std::string subp(prop.substr(i+n,j-i-n));
  char* cval = const_cast<char*>(subp.c_str());
  if (strtok(cval," "))
    while ((cval = strtok(NULL," ")))
      data.push_back(atof(cval));
}


void FmBeamProperty::convertFromGenericDBObjects()
{
  std::map<FmGenericDBObject*,FmBeamProperty*> old2new;
  std::map<FmGenericDBObject*,FmBeamProperty*>::const_iterator it;

  std::vector<FmBeam*> allBeams;
  FmDB::getAllBeams(allBeams);
  for (FmBeam* beam : allBeams)
  {
    FmGenericDBObject* gen = dynamic_cast<FmGenericDBObject*>(beam->getProperty());
    if (!gen) continue; // no generic DB object as property for this beam

    it = old2new.find(gen);
    if (it != old2new.end())
      beam->setProperty(it->second);
    else
    {
      // Get keyword
      std::string keyWord = gen->getUserDescription();
      size_t ipos = keyWord.find(" property");
      if (ipos < keyWord.size()) keyWord.erase(ipos);

      // Get data values, i.e. parse the text-blob
      std::vector<double> geo, mat, hyd;
      parseProp(gen->objectDefinition.getValue(),"geometry",geo);
      parseProp(gen->objectDefinition.getValue(),"material",mat);
      parseProp(gen->objectDefinition.getValue(),"hydyn",hyd);

      double A   = geo.size() > 0 ? geo[0] : 0.0;
      double Iyy = geo.size() > 1 ? geo[1] : 0.0;
      double Izz = geo.size() > 2 ? geo[2] : 0.0;
      double Ip  = geo.size() > 3 ? geo[3] : 0.0;

      double Rho = mat.size() > 0 ? mat[0] : 0.0;
      double E   = mat.size() > 1 ? mat[1] : 0.0;
      double G   = mat.size() > 2 ? mat[2] : 0.0;

      double Ca  = hyd.size() > 0 ? hyd[0] : 0.0;
      double Cm  = hyd.size() > 1 ? hyd[1] : 0.0;
      double Cd  = hyd.size() > 2 ? hyd[2] : 0.0;
      double Dd  = hyd.size() > 3 ? hyd[3] : 0.0;
      double Db  = hyd.size() > 4 ? hyd[4] : 0.0;
      double Dih = hyd.size() > 5 ? hyd[5] : 0.0;
      double Cda = hyd.size() > 6 ? hyd[6] : 0.0;
      double Cds = hyd.size() > 7 ? hyd[7] : 0.0;
      double Caa = hyd.size() > 8 ? hyd[8] : 0.0;
      double Cma = hyd.size() > 9 ? hyd[9] : 0.0;

      // Calculate Do and Di (we simply assume it's always a pipe)
      if (A <= 0.0 || Iyy != Izz) continue; // Not a pipe!

      double a = Ip/A;
      double b = 0.5*A/M_PI;
      if (a < b) a = b; // Assuming a massive cross section (Di=0)
      double Do = 2.0*sqrt(a+b);
      double Di = 2.0*sqrt(a-b);

      ListUI <<"  -> Converting "<< gen->getIdString(true)
	     <<": nGeo="<< (int)geo.size() <<" nMat="<< (int)mat.size()
	     <<" nHyd="<< (int)hyd.size()
	     <<"\n     Rho="<< Rho <<" E="<< E <<" G="<< G
	     <<"\n     A="<< A <<" Iyy="<< Iyy <<" Izz="<< Izz <<" Ip="<< Ip;
      if (!hyd.empty())
	ListUI <<"\n     Ca="<< Ca <<" Cm="<< Cm <<" Cd="<< Cd
	       <<" Dd="<< Dd <<" Db="<< Db <<" Di="<< Dih;
      ListUI <<"\n";

      // Create material
      FmMaterialProperty* elmMat = new FmMaterialProperty();
      elmMat->setParentAssembly(gen->getParentAssembly());
      elmMat->setUserDescription(keyWord + " material");
      elmMat->connect();
      elmMat->updateProperties(Rho,E,G);

      // Create cross section
      FmBeamProperty* elmProp = new FmBeamProperty();
      elmProp->setParentAssembly(gen->getParentAssembly());
      elmProp->setUserDescription(keyWord + " cross section");
      elmProp->connect();
      elmProp->material.setRef(elmMat);
      elmProp->crossSectionType.setValue(FmBeamProperty::PIPE);
      elmProp->Do.setValue(Do);
      elmProp->Di.setValue(Di);
      if (a > b)
        elmProp->updateDependentValues();
      else
      {
        // Do not calculate dependent values
        elmProp->breakDependence.setValue(true);
        elmProp->A.setValue(A);
        elmProp->Iy.setValue(Iyy);
        elmProp->Iz.setValue(Izz);
        elmProp->Ip.setValue(Ip);
      }
      if (!hyd.empty()) {
	elmProp->hydroToggle.setValue(true);
	elmProp->Ca.setValue(Ca);
	elmProp->Cm.setValue(Cm);
	elmProp->Cd.setValue(Cd);
	elmProp->Dd.setValue(Dd);
	elmProp->Db.setValue(Db);
	elmProp->Di_hydro.setValue(Dih);
	elmProp->Cd_axial.setValue(Cda);
	elmProp->Cd_spin.setValue(Cds);
	elmProp->Ca_axial.setValue(Caa);
	elmProp->Cm_axial.setValue(Cma);
      }
      beam->setProperty(old2new[gen] = elmProp);
    }
  }

  // Delete the converted generic DB objects
  for (it = old2new.begin(); it != old2new.end(); ++it)
    it->first->erase();
}
