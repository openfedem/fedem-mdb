// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_BEAM_PROPERTY_H
#define FM_BEAM_PROPERTY_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmMaterialProperty.H"
#include "FFaLib/FFaString/FFaEnum.H"


class FmBeamProperty : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmBeamProperty();

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Beam cross section"; }

  virtual int printSolverEntry(FILE* fp);

  void updateDependentValues();

  static void convertFromGenericDBObjects();

protected:
  virtual ~FmBeamProperty();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  enum FmSection { PIPE=0, GENERIC=1 };
  FFaEnumMapping(FmSection) {
    FFaEnumEntry(PIPE   , "PIPE");
    FFaEnumEntry(GENERIC, "GENERIC");
    FFaEnumEntryEnd;
  };

  FFaField<FmSectionEnum> crossSectionType;

  FFaReference<FmMaterialProperty> material; //!< Material reference
  FFaField<FFaReferenceBase*>      materialField;

  // Tube Cross Section
  FFaField<double>  Do;   //!< Outer diameter
  FFaField<double>  Di;   //!< Inner diameter

  // Generic Beam Cross Section
  FFaField<double>  EA;   //!< Axial stiffness
  FFaField<Doubles> EI;   //!< Bending stiffness
  FFaField<Doubles> GAs;  //!< Shear stiffness
  FFaField<double>  GIt;  //!< Torsional stiffness
  FFaField<double>  Mass; //!< Line mass
  FFaField<double>  RoIp; //!< Torsional inertia

  // Dependent properties
  FFaField<bool>    breakDependence;
  FFaField<double>  A;    //!< Cross section area
  FFaField<double>  Iy;   //!< Moment of inertia about local Y-axis
  FFaField<double>  Iz;   //!< Moment of inertia about local Z-axis
  FFaField<double>  Ip;   //!< Polar moment of inertia

  // Shear
  FFaField<Doubles> ShrRed;    //!< Shear reduction factors
  FFaField<Doubles> ShrCentre; //!< Shear center offset

  // Hydrodynamic properties
  FFaField<bool>   hydroToggle;
  FFaField<double> Di_hydro;
  FFaField<double> Db;
  FFaField<double> Dd;
  FFaField<double> Cd;
  FFaField<double> Ca;
  FFaField<double> Cm;
  FFaField<double> Cd_axial;
  FFaField<double> Ca_axial;
  FFaField<double> Cm_axial;
  FFaField<double> Cd_spin;
};

#endif
