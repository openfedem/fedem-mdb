// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmMaterialProperty.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaString/FFaParse.H"

Fmd_DB_SOURCE_INIT(FcMATERIAL_PROPERTY, FmMaterialProperty, FmStructPropertyBase);


FmMaterialProperty::FmMaterialProperty()
{
  Fmd_CONSTRUCTOR_INIT(FmMaterialProperty);

  FFA_FIELD_INIT(Rho,7.85e3,"MASS_DENSITY");
  FFA_FIELD_INIT(E,  2.1e11,"YOUNGS_MODULUS");
  FFA_FIELD_INIT(nu, 0.29  ,"POISSONS_RATIO");
  FFA_FIELD_INIT(G,  2.1e11/2.58,"SHEAR_MODULUS");
}


FmMaterialProperty::~FmMaterialProperty()
{
  this->disconnect();
}


std::ostream& FmMaterialProperty::writeFMF(std::ostream& os)
{
  os <<"MATERIAL_PROPERTY\n{\n";
  this->writeFields(os);
  os <<"}\n\n";

  return os;
}


bool FmMaterialProperty::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmMaterialProperty::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmMaterialProperty::getClassTypeID());
}


bool FmMaterialProperty::readAndConnect(std::istream& is, std::ostream&)
{
  FmMaterialProperty* obj = new FmMaterialProperty();

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


bool FmMaterialProperty::updateProperties(double d, double e,
                                          double g, double v)
{
  // If the mass density is changed, recalculate the mass properties
  // for all generic parts using this material
  if (Rho.setValue(d))
  {
    std::vector<FmPart*> parts;
    this->getReferringObjs(parts);
    for (size_t i = 0; i < parts.size(); i++)
      if (parts[i]->useGenericProperties.getValue())
        if (parts[i]->myCalculateMass.getValue() == FmPart::FROM_GEOMETRY)
          parts[i]->updateMassProperties();
  }

  // Maintain the dependency between E, G and nu
  // see https//en.wikipedia.org/wiki/Lame_parameters
  if (v < 0.0 || v >= 0.5 || v == nu.getValue())
    v = e/(g+g) - 1.0;
  else if (g == G.getValue())
    g = e/(2.0+v+v);
  else if (e == E.getValue())
    e = (1.0+v)*(g+g);

  if (v < 0.0 || v >= 0.5)
  {
    v = nu.getValue();
    if (g == G.getValue())
      g = e/(2.0+v+v);
    else if (e == E.getValue())
      e = (1.0+v)*(g+g);
    else
      return false; // Do not update parameters if invalid Poisson's ratio
  }

  E.setValue(e);
  G.setValue(g);
  nu.setValue(v);

  return true;
}
