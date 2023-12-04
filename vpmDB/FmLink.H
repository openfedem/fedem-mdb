// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_LINK_H
#define FM_LINK_H

#include "vpmDB/FmIsPositionedBase.H"
#include "vpmDB/FmColor.H"
#include "FFaLib/FFaAlgebra/FFaUnitCalculator.H"
#include "FFaLib/FFaContainers/FFaByteArray.H"
#include "FFaLib/FFaString/FFaEnum.H"


template<>
inline bool FFaField<FFaUnitCalculator>::isPrintable() const
{
  return !this->isDefault();
}

template<>
inline bool FFaField<FFaByteArray>::isPrintable() const
{
  return !myData.empty();
}


//! \brief Same as FFaField<int>, but hiding negative-valued instances.
class FFaFieldUnsignedInt : public FFaField<int>
{
public:
  virtual bool isPrintable() const { return this->myData >= 0; }
};


class FmTriad;
class FmJointBase;
class FmMMJointBase;
class FmSMJointBase;
class VTFAFile;


class FmLink : public FmIsPositionedBase
{
  Fmd_DB_HEADER_INIT();

public:

  // 3D display values

  const FmColor& getLineRGBColor() const { return myLineRGBColor.getValue(); }
  bool setLineRGBColor(const FmColor& col);

  double getShininess() const { return myShininess.getValue(); }
  bool setShininess(double var);

  const FmColor& getRGBColor() const { return myRGBColor.getValue(); }
  bool setRGBColor(const FmColor& col);
  bool setRGBColor(float r, float g, float b);

  double getTransparency() const { return myTransparency.getValue(); }
  bool setTransparency(double var);

  enum Detail { FULL = 0, SURFACE = 1, REDUCED = 2,
                BBOX = 3, OFF = 4, RED_FULL = 5,
                REDUCED_NO1D = 6, SIMPLIFIED = 7 };

  FFaEnumMapping(Detail) {
    FFaEnumEntry(FULL,         "FULL");
    FFaEnumEntry(SURFACE,      "SURFACE");
    FFaEnumEntry(REDUCED,      "REDUCED");
    FFaEnumEntry(BBOX,         "BBOX");
    FFaEnumEntry(OFF,          "OFF");
    FFaEnumEntry(RED_FULL,     "RED_FULL");
    FFaEnumEntry(REDUCED_NO1D, "REDUCED_NO1D");
    FFaEnumEntry(SIMPLIFIED,   "SIMPLIFIED");
    FFaEnumEntryEnd;
  };

  Detail getMeshType() const { return myMeshType.getValue(); }
  bool   setMeshType(Detail detailType);

  Detail getModelType() const { return myModelType.getValue(); }
  bool   setModelType(Detail detailType);

  virtual bool highlight(bool trueOrFalse);

  // Utility methods

  virtual bool interactiveErase();

  bool isEarthLink() const;
  virtual bool isAttachable() const { return false; }
  virtual bool isDisabled() const { return false; }
  virtual bool isSuppressed() const { return false; }
  virtual bool isGenericPart() const { return false; }
  virtual bool isFEPart(bool = false) const { return false; }

  FmTriad* getTriadAtPoint(const FaVec3& point, double tolerance,
                           bool globalPoint = false) const;

  FaVec3 getExtents() const;
  virtual bool getBBox(FaVec3& max, FaVec3& min) const;

  virtual void updateChildrenDisplayTopology();

  bool attach(FmBase* attachObject, bool isSilent = false);

  virtual std::string getLinkIDString(bool objPrefix = false) const;

  // DB Topology

  virtual int getNumberOfTriads() const = 0;
  virtual FmTriad* findTriad(int baseID) const = 0;
  virtual void getTriads(std::vector<FmTriad*>& toFill,
                         bool sortOnId = false) const = 0;
  void getJoints(std::vector<FmJointBase*>& toFill) const;

  // Co-rotated coordinate system control

  virtual bool getRefPoints(FmTriad*& node1, FmTriad*& node2, FmTriad*& node3,
                            FaVec3& offset1, FaVec3& offset2, FaVec3& offset3) const;

  virtual double getMass() const = 0;
  virtual FaMat34 getPositionCG(bool globalCS = true) const = 0;
  virtual FmModelMemberBase* getProperty() const { return NULL; }

  // File handling

  virtual int printSolverEntry(FILE*, int) { return 0; }
  virtual int printSolverEntry(FILE* fp) { return this->printSolverEntry(fp,0); }
  virtual bool writeToVTF(VTFAFile& file, IntVec* = NULL, IntVec* = NULL) = 0;

  bool openCadData();
  bool saveCadData();
  bool isCADLoaded() const;
  bool isUsingGenPartVis() const;

  virtual std::string getAbsFilePath(bool createDir = false) const;
  std::string getBaseCadFile(bool createDir = false) const;
  std::string getGeometryFile() const;

  int getStructDmpEngineId() const;

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  FmLink(const FaVec3& globalPos = FaVec3());
  virtual ~FmLink();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmLink* obj);

  bool attachTriad(FmTriad* attachTr, bool isSilent, bool doUpdate = true);
  virtual bool attachSMJoint(FmSMJointBase* attachJnt, bool isSilent = false);
  virtual bool attachMMJoint(FmMMJointBase* attachJnt, bool isSilent = false);
  virtual bool attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool silence);

  bool isTriadAttachable(FmTriad*& existingTriad,
                         FmTriad* attachTr, bool isSilent = false);

  virtual std::pair<FmTriad*,bool> getExistingTriad(const FmTriad* triad);

public:
  //! File for generic part visualization
  FFaField<std::string>       visDataFile;
  //! Unit converter for visualization file
  FFaField<FFaUnitCalculator> visDataFileUnitConverter;
  //! File that stores internal CAD data
  FFaField<std::string>       baseCadFileName;
  //! Id used to look up this link in the CAD system
  FFaField<FFaByteArray>      cadMainComponentId;


  // Structural damping properties
  FFaField<double> alpha1; //!< Mass proportional damping
  FFaField<double> alpha2; //!< Stiffness proportional damping

  // Scaling of dynamic properties
  FFaField<double> massScale;      //!< Mass matrix scaling factor
  FFaField<double> stiffnessScale; //!< Stiffness matrix scaling factor

  // Parameters related to shadow element positioning algorithm

  enum CoordSysOption {
    MODEL_DEFAULT,
    MAX_TRI_UNIT_OFFSET,
    MAX_TRI_LINK_SCALE_OFFSET,
    MASS_BASED_AVERAGE
  };

  FFaEnumMapping(CoordSysOption) {
    FFaEnumEntry(MODEL_DEFAULT,            "MODEL_DEFAULT");
    FFaEnumEntry(MAX_TRI_UNIT_OFFSET,      "MAX_TRI_UNIT_OFFSET");
    FFaEnumEntry(MAX_TRI_LINK_SCALE_OFFSET,"MAX_TRI_LINK_SCALE_OFFSET");
    FFaEnumEntry(MASS_BASED_AVERAGE,       "MASS_BASED_AVERAGE");
    FFaEnumEntryEnd;
  };

  FFaField<CoordSysOptionEnum> myCSOption;

  FFaFieldUnsignedInt objFileGroupIndex; //!< Group index in obj-file

protected:
  // 3D display variables
  FFaField<FmColor>    myLineRGBColor;
  FFaField<FmColor>    myRGBColor;
  FFaField<double>     myShininess;
  FFaField<double>     myTransparency;
  FFaField<DetailEnum> myMeshType;
  FFaField<DetailEnum> myModelType;
};

#endif
