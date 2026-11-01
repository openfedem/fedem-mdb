// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_TRIAD_H
#define FM_TRIAD_H

#include "vpmDB/FmHasDOFsBase.H"
#include "vpmDB/FmLink.H"
#ifdef FT_USE_CONNECTORS
#include "FFlLib/FFlConnectorItems.H"
#include "FFaLib/FFaGeometry/FFaCompoundGeometry.H"


template<> inline
bool FFaField<FFaCompoundGeometry>::isPrintable() const
{
  return !myData.empty();
}

template<> inline
bool FFaField<FFlConnectorItems>::isPrintable() const
{
  return !myData.empty();
}
#endif


class FmBeam;
class FmPart;
class FmJointBase;
class FmAxialSpring;
class FmAxialDamper;
class FmLoad;
class FmTire;
#ifdef FF_NAMESPACE
namespace FF_NAMESPACE { class FFlNode; }
using FFlNode = FF_NAMESPACE::FFlNode;
#else
class FFlNode;
#endif


class FmTriad : public FmHasDOFsBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

  //! \brief Common initializer method used by both constructors.
  void init();

public:
  //! \brief Default constructor.
  FmTriad();
  //! \brief Constructor creating a triad at specified location.
  FmTriad(const FaVec3& globalPos);

protected:
  //! \brief The destructor diconnects the triad from the model database
  virtual ~FmTriad();

  // From FmBase
  bool cloneLocal(FmBase* obj, int depth) override;

  //! \copybrief FmModelMemberBase::eraseOptions()
  bool eraseOptions() override;

public:
  //! \brief Adds this triad to the model database.
  bool connect(FmBase* parent, int nodeId = -1);
  using FmHasDOFsBase::connect;

  // From FmBase
  bool disconnect() override;

  // From FmIsRenderedBase
  bool highlight(bool trueOrFalse) override;
  // From FmSimulationModelBase
  void initAfterResolve() override;
  // From FmModelMemberBase
  bool interactiveErase() override;

  //! \brief Syncronizes the FE node reference of the triad.
  int syncOnFEmodel(bool useDialog);

  //! \brief Returns the link this triad is attached to, if any.
  FmLink* getOwnerLink(int linkIndex = -1) const;
  //! \brief Returns the part this triad is attached to, if any.
  FmPart* getOwnerPart(int partIndex = -1) const;
  //! \brief Returns the FE part this triad is attached to, if any.
  FmPart* getOwnerFEPart() const;

  //! \brief Returns the joint in which this triad is the slave.
  FmJointBase* getJointWhereSlave() const;
  //! \brief Sets this triad as the slave in the joint \a jnt.
  bool setAsSlave(FmJointBase* jnt);

  //! \brief Checks if this triad is attached to the specified link.
  bool isAttached(const FmLink* link, bool exceptForThis = false) const;
  //! \brief Checks if this triad is attached to the specified link.
  bool isAttached(bool ignoreGPandEarth = false,
                  bool allowMultipleLinks = true) const;

  //! \brief Returns \e true if this triad is the slave in a joint.
  bool isSlaveTriad(bool realSlavesOnly = false) const;
  //! \brief Returns \e true if this triad is a master in a joint.
  bool isMasterTriad(bool realMastersOnly = false) const;
  //! \brief Returns \e true if this triad is a master in a multi-master joint.
  bool isMultiMaster(bool includingCam = true) const;
  //! \brief Returns \e true if this triad connected to a multi-master joint.
  bool isInLinJoint() const;

  // From FmIsPositionedBase
  std::string getLinkIDString(bool objPrefix) const override;

  //! \brief Returns the number of DOFs in this triad.
  int  getNDOFs(bool checkForSuppressedOwner = false) const;
  //! \brief Sets the number of DOFs for this triad.
  bool setNDOFs(int NDOFs);

  //! \brief Returns the FE node #itsNode (if any) connected to this triad.
  FFlNode* getFENode() const { return itsNode; }

  //! \brief Assigns the status code for the <i>dof</i>'th DOF in this triad.
  bool      setStatusForDOF(int dof, DOFStatus status) override;
  //! \brief Returns the status code for the <i>dof</i>'th DOF in this triad.
  DOFStatus getStatusOfDOF(int dof) const override;
  //! \brief Returns \e true if the DOF \a dof exists for this triad.
  bool      isLegalDOF(int dof) const override;

  // From FmHasDOFsBase
  bool isSuppressed() const override;
  bool hasConstraints(bool fixedOnly = false) const override;
  //! \brief Returns \e true if all DOFs in this triad is fixed or prescribed.
  bool fullyConstrained(bool fixedOnly = false) const;

  //! \brief Returns the global position matrix of this triad.
  FaMat34 getGlobalCS() const override;
  //! \brief Sets the triad position to be aligned with the given global matrix.
  void setGlobalCS(const FaMat34& globalMat,
                   bool moveRelationsAlong = false) override;
  //! \brief Sets the triad position relative to the part it is attached to.
  void setLocalCS(const FaMat34& localMat) override;

  //! \brief Returns the translation part of the global position of this triad.
  FaVec3 getGlobalTranslation() const;
  //! \brief Returns the translation part of the local position of this triad.
  FaVec3 getLocalTranslation(const FmLink* link = NULL) const;
  //! \brief Returns local coordinate system of triad, relative to given link.
  FaMat34 getRelativeCS(const FmLink* link) const;

  // From FmIsPositionedBase
  bool isTranslatable() const override { return this->isTranslatable(NULL); }
  char isRotatable() const override { return this->isRotatable(NULL); }
  //! \brief Checks if this triad is free to be translated.
  bool isTranslatable(const FmJointBase* jointToIgnore) const;
  //! \brief Checks if this triad is free to be rotated.
  char isRotatable(const FmJointBase* jointToIgnore) const;

  //! \brief Returns \e true if any object of any type refers to this triad.
  bool hasReferences() const;
  //! \brief Returns \e true if the triad symbol is to be displayed.
  bool showSymbol() const;
  //! \brief Returns \e true if the triad represents something that
  //! requires its directions to be visualized.
  bool showDirections();
  //! \brief Returns \e true if the triad represents something that
  //! is referring to its directions.
  bool importantDirections();
  //! \brief Returns \e true if only free joints are connected to this triad.
  bool hasOnlyFreeJoints() const;

  void getBeamBinding(std::vector<FmBeam*>& toFill) const;
  void getElementBinding(std::vector<FmLink*>& toFill) const;
  void getJointBinding(std::vector<FmJointBase*>& toFill) const;
  void getSpringBinding(std::vector<FmAxialSpring*>& toFill) const;
  void getDamperBinding(std::vector<FmAxialDamper*>& toFill) const;
  void getMotionBinding(std::vector<FmDofMotion*>& toFill) const;
  void getLoadBinding(std::vector<FmDofLoad*>& toFill) const;
  void getLoadBinding(std::vector<FmLoad*>& toFill) const;
  void getTireBinding(std::vector<FmTire*>& toFill) const;

  bool hasBeamBinding() const;
  bool hasElementBinding() const;
  bool hasJointBinding() const;
  bool hasSpringBinding() const;
  bool hasDamperBinding() const;
  bool hasLoadBinding() const;
  bool hasTireBinding() const;

  bool hasLoad(int DOF) const;
  bool hasAddMass() const;
  bool hasInitVel() const;
  bool hasInitAcc() const;

  //! \brief Returns the added mass on this triad, if any.
  //! \details \a DOF &lt; 0 means the average of the diagonal mass terms.
  double getAddMass(int DOF = -1) const;

  bool setDOFStatus(int DOF, DOFStatus status);
  void setAddMass(int DOF, double mass);
  void setAddedMass(double mass);
  void setInitVel(int DOF, double vel);
  void setInitAcc(int DOF, double acc);

  void setMassEngine(FmEngine* engine) { myMassEngine.setRef(engine); }
  FmEngine* getMassEngine() const { return myMassEngine.getPointer(); }

  //! \brief Creates a new triad object by reading data from a stream.
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

  // From FmBase
  std::ostream& writeFMF(std::ostream& os = std::cout) override;
  bool clone(FmBase* obj, int depth) override;
  const char* getUITypeName() const override { return "Triad"; }
  //! \brief Returns the list view icon to be used in the Objects browser.
  const char** getListViewPixmap() const override;

  // From FmModelMemberBase
  int printSolverEntry(FILE* fp) override;
  //! \brief Prints the link-relative position of the triad for the solver.
  void printLocalPos(FILE* fp, const FmLink* link, int triadID = 0,
                     bool endOfRecord = true) const;
  //! \brief Prints the additional masses of this triad for the solver.
  int printAdditionalMass(FILE* fp);

  // From FmIsRenderedBase
  void updateChildrenDisplayTopology() override;
  bool detach() override { return this->detach(NULL,true); }
  //! \brief Detaches this triad from the part it is connected to.
  bool detach(FmLink* fromThisonly, bool notFromDisabledPart = false);

  // From FmIsMeasuredBase
  void getEntities(std::vector<FmSensorChoice>& choicesToFill, int) override;
  void getDofs(std::vector<FmSensorChoice>& choicesToFill) override;

  //! \brief Checks the consistency of the triad topology.
  static int checkTriads();
  //! \brief Gets all triads along a generated beamstring.
  static int traverseBeam(FmBase* start, std::vector<FmIsPlottedBase*>& triads);
  //! \brief Creates a triad at specified \a node.
  static FmTriad* createAtNode(FFlNode* node, FmBase* parent,
                               int IDoffset, int& nTriad);

private:
  //! \brief Internal method used only from connect() and initAfterResolve().
  bool updateFENodeAndDofs(FmPart* ownerPart, int nodeId = -1);
  //! \brief Deletes all joints that this triad is a member of.
  bool removeJointBinding();

public:

  enum LocalDirection {
    GLOBAL = 0,
    LOCAL_INITIAL = 1,
    LOCAL_WITHROTATED = 2
  };

  enum ConnectorType {
    NONE = 0,
    RIGID = 2,
    AVERAGE_MOTION = 3
  };

  FFaEnumMapping(LocalDirection) {
    FFaEnumEntry(GLOBAL,            "GLOBAL");
    FFaEnumEntry(LOCAL_INITIAL,     "LOCAL_INITIAL");
    FFaEnumEntry(LOCAL_WITHROTATED, "LOCAL_WITHROTATED");
    FFaEnumEntryEnd;
  };

  FFaEnumMapping(ConnectorType) {
    FFaEnumEntry(NONE,           "NONE");
    FFaEnumEntry(RIGID,          "RIGID");
    FFaEnumEntry(AVERAGE_MOTION, "AVERAGE_MOTION");
    FFaEnumEntryEnd;
  };

#ifdef FT_USE_CONNECTORS
  //! \brief Updates a connector using the triad's stored info.
  bool updateConnector(ConnectorType type, FmPart* ownerPart = NULL);
#endif

  FFaFieldUnsignedInt           FENodeNo;
  FFaField<LocalDirectionEnum>  itsLocalDir;
#ifdef FT_USE_CONNECTORS
  FFaField<FFaCompoundGeometry> itsConnectorGeometry;
  FFaField<FFlConnectorItems>   itsConnectorItems;
  FFaField<ConnectorTypeEnum>   itsConnectorType;
#endif

  //! Convenience type alias
  using DOFStatusVec = std::vector<FmHasDOFsBase::DOFStatusEnum>;

private:
  FFaReferenceList<FmLink>        myAttachedLinks;
  FFaField<FFaReferenceListBase*> myAttachedLinksField;

  FFaReference<FmEngine>      myMassEngine;
  FFaField<FFaReferenceBase*> myMassEngineField;

  FFaField<DOFStatusVec> itsBndC;  //!< Boundary conditions codes for this triad
  FFaField<DoubleVec>    itsMass;  //!< Added mass terms for this triad
  FFaField<int>          itsNDOFs; //!< Number of triad DOFs (either 0, 3 or 6)

  FFlNode* itsNode; //!< Pointer to the FE node associated with this triad
};


#ifdef FT_USE_CONNECTORS
template<> inline
bool FFaField<FmTriad::ConnectorTypeEnum>::isPrintable() const
{
  return myData > FmTriad::NONE;
}
#endif


template<> inline
bool FFaField<FmTriad::DOFStatusVec>::isPrintable() const
{
  return !myData.empty();
}

#endif
