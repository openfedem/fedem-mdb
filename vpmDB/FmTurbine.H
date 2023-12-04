// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_TURBINE_H
#define FM_TURBINE_H

#include "vpmDB/FmAssemblyBase.H"
#include "vpmDB/FmBeamProperty.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"

class FmBladeProperty;
class FmBladeDesign;
class FmTower;
class FmNacelle;
class FmGenerator;
class FmGearBox;
class FmShaft;
class FmRotor;
class FmLink;
class FmJointBase;


class FmTurbine : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  FmTurbine(char createSubAssemblies = '\0', bool isDummy = false);
  virtual ~FmTurbine() {}

  void getParts(FmTower*& tower, FmNacelle*& nacelle,
		FmGenerator*& generator, FmGearBox*& gearbox,
		FmShaft*& lsShaft, FmShaft*& hsShaft, FmRotor*& rotor) const;
  FmTower* getTower() const;
  FmBladeDesign* getBladeProperties(std::vector<FmBladeProperty*>& bprop) const;
  double getRadius(const FaVec3& X) const;
  double getHubRadius() const;
  double getHubHeight() const;
  double getRotorSize() const;

  void draw() const;

  virtual const char* getUITypeName() const { return "Turbine"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual int printSolverEntry(FILE* fp);

  int writeAeroDynFile(const std::string& fileName) const;

  static bool writeBladeElement(FILE* fp, FmLink* beam, int& propId);

protected:
  virtual bool eraseOptions();

public:
  FFaField<double> rhoIce;  //!< Mass density for ice
  FFaField<double> ptfmRef; //!< Platform reference height
  FFaField<int>    nBlade;  //!< Number of blades
  FFaField<bool>   ctrlSys; //!< Include regulation system?

  FFaReference<FmSubAssembly> bladeDef; //!< Blade segment layout
  FFaField<FFaReferenceBase*> bladeDefField;

  // Some additional data required by the Dynamics Solver
  FFaReferenceList<FmModelMemberBase> topology;
  FFaField<FFaReferenceListBase*>     topologyField;

  FFaField<std::string> towerFile;
};


// The other class definitions below are kept in this same file
// since they are closely related to the FmTurbine class.

struct FmTwrSegment
{
  double H; //!< Height of segment
  double D; //!< Diameter at segment top
  int    N; //!< Number of elements
  FmTwrSegment(double h = 10.0, double d = 5.0, int n = 5) : H(h), D(d), N(n) {}
  bool operator==(const FmTwrSegment& s) const
  { return H == s.H && D == s.D && N == s.N; }
};

typedef std::vector<FmTwrSegment> FmTwrSegmentVec;


template<> inline
bool FFaField<FmTwrSegmentVec>::isPrintable() const
{
  return !myData.empty();
}

template<> inline
void FFaField<FmTwrSegmentVec>::write(std::ostream& os) const
{
  for (size_t i = 0; i < myData.size(); i++)
  {
    // write up to 9 numbers on a row
    if (i) os << (i%3 ? " " : "\n\t");
    os << myData[i].H <<" "<< myData[i].D <<" "<< myData[i].N;
  }
}

template<> inline
void FFaField<FmTwrSegmentVec>::read(std::istream& is, FmTwrSegmentVec& v) const
{
  v.clear();
  double h, d, n;
  while (is)
  {
    is >> h >> d >> n;
    if (is)
      v.push_back(FmTwrSegment(h,d,n));
  }
  is.clear();
}


class FmTurbinePart : public FmAssemblyBase
{
protected:
  FmTurbinePart(bool isDummy) : FmAssemblyBase(isDummy) {}
  virtual ~FmTurbinePart() {}

  void initFields(double mass);

public:
  FFaField<Doubles> Stiff;   //!< Stiffness parameters of the component
  FFaField<double>  Mass;    //!< Total mass of the component
  FFaField<FaVec3>  CoG;     //!< Centre of gravity of the component
  FFaField<FaVec3>  Inertia; //!< Mass inertias of the component
  FFaField<FaVec3>  Iaxes;   //!< Inertia axes of the component
};


class FmTower : public FmTurbinePart
{
  Fmd_HEADER_INIT();

public:
  FmTower(bool isDummy = false);
  virtual ~FmTower() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Tower"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaReference<FmMaterialProperty> material;
  FFaField<FFaReferenceBase*>      materialField;

  FFaField<FmTwrSegmentVec> segments; //!< Discretization data
  FFaField<double>          Height;   //!< Tower height
  FFaField<double>          Thick;    //!< Wall thickness

  FFaField<int> visualize3Dts; //!< Visualize the tower in the 3D view (tristate: off,na,on)
};


class FmNacelle : public FmTurbinePart
{
  Fmd_HEADER_INIT();

public:
  FmNacelle(bool isDummy = false);
  virtual ~FmNacelle() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Nacelle"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaField<double> B1; //!< Main bearing location
  FFaField<double> B2; //!< Second bearing location
  FFaField<double> C1; //!< Brake location
  FFaField<double> M2; //!< Yaw-axis shaft intersection
  FFaField<double> M3; //!< Shaft length from yaw axis (N3)
};


class FmGenerator : public FmTurbinePart
{
  Fmd_HEADER_INIT();

public:
  FmGenerator(bool isDummy = false);
  virtual ~FmGenerator() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Generator"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaField<double> Length; //!< Length of generator (N7)
};


class FmGearBox : public FmTurbinePart
{
  Fmd_HEADER_INIT();

public:
  FmGearBox(bool isDummy = false);
  virtual ~FmGearBox() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Gearbox"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaField<double> Ratio;  //!< Gear transmission ratio
  FFaField<double> Length; //!< Length of gear box (N5)
  FFaField<double> O1;     //!< Horizontal offset
  FFaField<double> O2;     //!< Vertical offset
};


class FmShaft : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  FmShaft(bool isDummy = false, double OD = 0.4, double ID = 0.3, double L = 7.0);
  virtual ~FmShaft() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Shaft"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaField<double> Tilt;   //!< Tilt angle in degrees (alpha)
  FFaField<double> Length; //!< Shaft length (N4&N6)

  FFaField<int> visualize3Dts; //!< Visualize the shaft in the 3D view (tristate: off,na,on)

  // Only used temporary :

  FFaNoPrintField<double> Do;  //!< Outer diameter
  FFaNoPrintField<double> Di;  //!< Inner diameter
  FFaNoPrintField<double> Rho; //!< Mass density
  FFaNoPrintField<double> E;   //!< Young's modulus
  FFaNoPrintField<double> G;   //!< Shear modulus
};


class FmRotor : public FmTurbinePart
{
  Fmd_HEADER_INIT();

public:
  FmRotor(bool isDummy = false);
  virtual ~FmRotor() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Rotor"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

public:
  FFaField<double> PreCone; //!< Precone angle in degrees (beta)
  FFaField<double> HubDiam; //!< Hub diameter (D1=2*Hr)
  FFaField<double> HubApex; //!< Hub apex offset (N2)
};


class FmBlade : public FmAssemblyBase
{
  Fmd_HEADER_INIT();

public:
  FmBlade(bool isDummy = false);
  virtual ~FmBlade() {}

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Blade"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isCopyable() const { return false; }

  FmJointBase* getPitchJoint() const;

public:
  FFaField<bool>   IceLayer;     //!< Ice layer?
  FFaField<double> IceThickness; //!< Ice thickness

  FFaField<int> visualize3Dts; //!< Visualize the blade in the 3D view (tristate: off,na,on)
};

#endif
