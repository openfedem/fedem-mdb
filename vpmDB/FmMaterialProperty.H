// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MATERIAL_PROPERTY_H
#define FM_MATERIAL_PROPERTY_H

#include "vpmDB/FmSimulationModelBase.H"


class FmMaterialProperty : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmMaterialProperty();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Material"; }

  bool updateProperties(double d, double e, double g, double v = -1.0);

protected:
  virtual ~FmMaterialProperty();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<double> Rho; //!< Mass density
  FFaField<double> E;   //!< Young's modulus
  FFaField<double> nu;  //!< Poisson's ratio
  FFaField<double> G;   //!< Shear modulus
};

#endif
