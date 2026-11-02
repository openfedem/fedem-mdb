// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_LINK_H
#define FM_LINK_H

#include "FFaLib/FFaAlgebra/FFaUnitCalculator.H"
#include "FFaLib/FFaString/FFaEnum.H"

#include "vpmDB/FmColor.H"
#include "vpmDB/FmIsPositionedBase.H"


//! \cond TEMPLATE_SPECIALIZATION
template<>
inline bool FFaField<FFaUnitCalculator>::isPrintable() const
{
  return !this->isDefault();
}
//! \endcond


//! \brief Same as FFaField<int>, but hiding negative-valued instances.
class FFaFieldUnsignedInt : public FFaField<int>
{
public:
  bool isPrintable() const final { return this->myData >= 0; }
};


class FmTriad;
class FmJointBase;
class FmMMJointBase;
class FmSMJointBase;
class VTFAFile;


class FmLink : public FmIsPositionedBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

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

  //! \brief Enumerator for different visualization modes.
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

  // From FmIsRenderedBase
  bool highlight(bool trueOrFalse) override;

  // Utility methods

  // From FmModelMemberBase
  bool interactiveErase() override;

  //! \brief Returns \e true if \a *this is the earth link.
  bool isEarthLink() const { return this->getID() == -1; }

  //! \brief Returns \e true if objects can be attached to this link.
  virtual bool isAttachable() const { return false; }
  //! \brief Returns \e true if the FE data of this link is disabled.
  virtual bool isDisabled() const { return false; }
  //! \brief Returns \e true if this link is suppressed.
  virtual bool isSuppressed() const { return false; }
  //! \brief Returns \e true if this link is a generic part.
  virtual bool isGenericPart() const { return false; }
  //! \brief Returns \e true if this link is a (loaded) FE part.
  virtual bool isFEPart(bool = false) const { return false; }

  //! \brief Returns the closest triad to \a point, or NULL if none found.
  FmTriad* getTriadAtPoint(const FaVec3& point, double tolerance,
                           bool globalPoint = false,
                           bool unconnectedOnly = false) const;

  //! \brief Returns the extension of this link in each coordinate direction.
  FaVec3 getExtents() const;
  //! \brief Calculates the bounding box of this link.
  virtual bool getBBox(FaVec3& max, FaVec3& min) const;

  // From FmIsRenderedBase
  void updateChildrenDisplayTopology() override;

  //! \brief Attaches the object \a attachObject to this link.
  bool attach(FmBase* attachObject, bool isSilent = false);

  // From FmIsPositionedBase
  std::string getLinkIDString(bool objPrefix = false) const override;

  // DB Topology

  //! \brief Returns the number of triads attached to this link.
  virtual int getNumberOfTriads() const = 0;
  //! \brief Returns a specified triad attached to this link.
  virtual FmTriad* findTriad(int baseID) const = 0;
  //! \brief Returns all triads attached to this link.
  virtual void getTriads(std::vector<FmTriad*>& toFill,
                         bool sortOnId = false) const = 0;
  //! \brief Returns all joints attached to this link.
  void getJoints(std::vector<FmJointBase*>& toFill) const;

  //! \brief Returns the points used for co-rotated coordinate system control.
  virtual bool getRefPoints(FmTriad*& node1, FmTriad*& node2, FmTriad*& node3,
                            FaVec3& offset1, FaVec3& offset2, FaVec3& offset3) const;

  //! \brief Returns the total mass of this link.
  virtual double getMass() const = 0;
  //! \brief Returns the position of the center of gravity of this link.
  virtual FaMat34 getPositionCG(bool globalCS = true) const = 0;
  //! \brief Returns the physical property object associated with this link.
  virtual FmModelMemberBase* getProperty() const { return NULL; }

  // File handling

  // Hook for VTF output of the FE model of this link.
  virtual bool writeToVTF(VTFAFile& file, IntVec* = NULL, IntVec* = NULL) = 0;
  // Hook for writing solver input file entries for beam elements.
  virtual int printSolverEntry(FILE*, int) { return 0; }
  // From FmModelMemberBase
  int printSolverEntry(FILE* f) override { return this->printSolverEntry(f,0); }

  //! \brief Reads the CAD model associated with this link for visualization.
  bool openCadData();
  //! \brief Writes visualization datA associated with this link to CAD file.
  bool saveCadData();
  //! \brief Returns \e true if the link's CAD model currently is loaded.
  bool isCADLoaded() const;
  //! \brief Returns \e true if the link uses generic part visualization.
  bool isUsingGenPartVis() const;
  //! \brief Updates the simplified generic part visualization.
  void updateGPVisualization();

  //! \brief Returns the path to the directory in which the link will be saved.
  virtual std::string getAbsFilePath(bool createDir = false) const;
  //! \brief Returns the full path of the saved CAD file of this link.
  std::string getBaseCadFile(bool createDir = false) const;
  //! \brief Returns the full path of the geometry file of this link.
  std::string getGeometryFile() const;

  //! \brief Returns the base ID of structural damping function for this link.
  int getStructDmpEngineId() const;

  //! \brief Creates a new link object by reading data from a stream.
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

protected:
  //! \brief Default constructor.
  FmLink(const FaVec3& globalPos = FaVec3());
  //! \brief Constructor for earth link.
  FmLink(bool);
  //! \brief The destructor disconnects the link.
  virtual ~FmLink();

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmLink* obj);

  //! \brief Attaches the triad \a attachTr to this link.
  bool attachTriad(FmTriad* attachTr, bool isSilent, bool doUpdate = true);
  //! \brief Attaches the single-master joint \a attachJnt to this link.
  virtual bool attachSMJoint(FmSMJointBase* attachJnt, bool isSilent = false);
  //! \brief Attaches the multi-master joint \a attachJnt to this link.
  virtual bool attachMMJoint(FmMMJointBase* attachJnt, bool isSilent = false);
  //! \brief Attaches a triad when having an old triad in the same place.
  virtual bool attachTriad(FmTriad* attachTr, FmTriad* oldTr, bool silence);

  //! \brief Checks whether the triad \a attachTr can be attached to this link.
  bool isTriadAttachable(FmTriad*& existingTriad,
                         FmTriad* attachTr, bool isSilent = false);

  //! \brief Returns any triad at the same location as \a triad on this link.
  virtual std::pair<FmTriad*,bool> getExistingTriad(const FmTriad* triad);

public:
  //! File for generic part visualization
  FFaField<std::string>       visDataFile;
  //! Unit converter for visualization file
  FFaField<FFaUnitCalculator> visDataFileUnitConverter;
  //! File that stores internal CAD data
  FFaField<std::string>       baseCadFileName;

  // Structural damping properties
  FFaField<double> alpha1; //!< Mass proportional damping
  FFaField<double> alpha2; //!< Stiffness proportional damping

  // Scaling of dynamic properties
  FFaField<double> massScale;      //!< Mass matrix scaling factor
  FFaField<double> stiffnessScale; //!< Stiffness matrix scaling factor

  //! \brief Enumerator related to shadow element positioning algorithm.
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

  FFaField<CoordSysOptionEnum> myCSOption; //!< Selected CS option for \a *this

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
