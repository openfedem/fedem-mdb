// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_BLADE_PROPERTY_H
#define FM_BLADE_PROPERTY_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmSubAssembly.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"

class FmBeam;


class FmBladeProperty : public FmStructPropertyBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmBladeProperty();

  static FmBladeProperty* readAndConnect(std::istream& is,
                                         std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Blade property"; }

  //! Hides the blade property objects in the objects list view.
  virtual bool isListable() const { return false; }

  Doubles getAeroCentre() const;
  Doubles getElasticCentre() const;
  Doubles getShearCentre() const;
  Doubles getMassCentre() const;

  void getStructData(double* data, double iceMass = 0.0) const;
  double getStructMass(double iceMass = 0.0) const;

protected:
  virtual ~FmBladeProperty();

  virtual bool cloneLocal(FmBase* obj, int depth);

public:
  FFaField<Doubles> ElCentre;   //!< Elastic centre (w.r.t. leading edge)
  FFaField<double>  ElAxisRot;  //!< Elastic centre rotation (relative to Twist)
  FFaField<Doubles> ShrCentre;  //!< Shear centre (w.r.t Elastic centre)
  FFaField<double>  EA;         //!< Axial stiffness
  FFaField<Doubles> EI;         //!< Bending stiffness
  FFaField<Doubles> GAs;        //!< Shear stiffness
  FFaField<double>  GIt;        //!< Torsional stiffness

  FFaField<Doubles> MassCentre; //!< Mass centre (w.r.t. Elastic centre)
  FFaField<double>  Mass;       //!< Mass per unith length
  FFaField<double>  Tinertia;   //!< Torsional mass inertia

  FFaField<Doubles> PitchCentre; //!< Pitch axis location (w.r.t. leading edge)
  FFaField<double>  AeroCentre;  //!< Aerodynamic centre (w.r.t leading edge)
  FFaField<double>  Twist;       //!< Aerodynamic twist
  FFaField<double>  Chord;       //!< Chord length (blade segment width)
  FFaField<double>  Thick;       //!< Blade segment thickness
  FFaField<double>  Length;      //!< Blade segment length

  FFaField<std::string> AirFoil; //!< Name of airfoil file
};


class FmBladeDesign : public FmSubAssembly
{
  Fmd_HEADER_INIT();

public:
  FmBladeDesign(bool isDummy = false);
  virtual ~FmBladeDesign() {}

  virtual bool interactiveErase();

  static FmBladeDesign* readAndConnect(std::istream& is,
                                       std::ostream& os = std::cout,
                                       bool autoConnect = false);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  bool writeToFMM(const std::string& fileName) const;
  static FmBladeDesign* readFromFMM(const std::string& fileName,
                                    bool setRdOnly = false,
                                    bool keepAll = false);

  std::string getModelFileName() const;

  virtual const char* getUITypeName() const { return "Blade design"; }

  //! Hides the blade design objects in the objects list view.
  virtual bool isListable(bool) const { return false; }

  //! Inserts a blade property at position \a pos (default at end).
  void addBladeProperty(FmBladeProperty* bp, int pos = -1);
  //! Deletes the blade property at position \a pos.
  bool removeBladeProperty(int pos);
  FmBladeProperty* getBladeProperty(int pos);

  void setBladeSegments(const std::vector<FmBladeProperty*>& segs);
  void getBladeSegments(std::vector<FmBladeProperty*>& segs) const;
  bool getSegmentLengths(std::vector<double>& DRNodes) const;

  bool getEccen(double R, Doubles& ElC, Doubles& CoG) const;
  bool getStructData(double R, double* data, double iceMass = 0.0) const;
  double getStructMass(double R, double iceMass = 0.0) const;
  double getElementMass(const FmBeam* beam) const;

private:
  size_t getPropertyIdxAt(double& R) const;

protected:
  FFaReferenceList<FmBladeProperty> segment;
  FFaField<FFaReferenceListBase*>   segmentField;

public:
  FFaNoPrintField<bool> readOnly; //!< Should the blade design be write-locked?

  FFaField<double> autoEmod; //!< Youngs modulus used in auto-stiffness creation
  FFaField<bool> withAstiff; //!< Include axial stiffness?
  FFaField<bool> withBstiff; //!< Include bending stiffness?
  FFaField<bool> withTstiff; //!< Include torsion stiffness?
  FFaField<bool> withSstiff; //!< Include shear stiffness?
};

#endif
