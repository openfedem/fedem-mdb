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
class FFlNode;


class FmTriad : public FmHasDOFsBase
{
  Fmd_DB_HEADER_INIT();

  void init();

public:
  FmTriad();
  FmTriad(const FaVec3& globalPos);

protected:
  virtual ~FmTriad();

  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual bool eraseOptions();

public:
  bool connect(FmBase* parent);
  using FmHasDOFsBase::connect;
  virtual bool disconnect();

  virtual bool highlight(bool trueOrFalse);
  virtual void initAfterResolve();
  virtual bool interactiveErase();

  int syncOnFEmodel();

  // linkIndex = -1 : Expect only one owner link and return 0 otherwise
  // linkIndex >= 0 : Return the linkIndex'th link this triad is attached to
  FmLink* getOwnerLink(int linkIndex = -1) const;
  FmPart* getOwnerPart(int partIndex = -1) const;
  FmPart* getOwnerFEPart() const;

  FmJointBase* getJointWhereSlave() const;
  bool         setAsSlave(FmJointBase* jnt);

  bool isAttached(const FmLink* link, bool exceptForThis = false) const;
  bool isAttached(bool ignoreGPandEarth = false,
                  bool allowMultipleLinks = true) const;

  bool isSlaveTriad(bool realSlavesOnly = false) const;
  bool isMasterTriad(bool realMastersOnly = false) const;
  bool isMultiMaster() const;
  bool isInLinJoint() const;

  virtual std::string getLinkIDString(bool objPrefix) const;

  int  getNDOFs(bool checkForSuppressedOwner = false) const;
  bool setNDOFs(int NDOFs);

  virtual bool      setStatusForDOF(int dof, DOFStatus status);
  virtual DOFStatus getStatusOfDOF(int dof) const;
  virtual bool      isLegalDOF(int dof) const;

  virtual bool isSuppressed() const;
  virtual bool hasConstraints(bool fixedOnly = false) const;
  bool fullyConstrained(bool fixedOnly = false) const;

  virtual FaMat34 getGlobalCS() const;
  virtual void    setGlobalCS(const FaMat34& globalMat, bool moveAlong = false);
  virtual void    setLocalCS(const FaMat34& localMat);

  FaVec3 getGlobalTranslation() const;
  FaVec3 getLocalTranslation(const FmLink* link = NULL) const;
  FaMat34 getRelativeCS(const FmLink* link) const;

  virtual bool isTranslatable() const { return this->isTranslatable(NULL); }
  virtual bool isRotatable() const { return this->isRotatable(NULL); }
  bool isTranslatable(const FmJointBase* jointToIgnore) const;
  bool isRotatable(const FmJointBase* jointToIgnore) const;

  bool hasReferences() const;
  bool showSymbol() const;
  bool showDirections();
  bool importantDirections();
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

  double getAddMass(int DOF = -1) const; // DOF < 0 means (m[0]+m[1]+m[2])/3.0

  bool setDOFStatus(int DOF, DOFStatus status);
  void setAddMass(int DOF, double mass);
  void setAddedMass(double mass);
  void setInitVel(int DOF, double vel);
  void setInitAcc(int DOF, double acc);

  void setMassEngine(FmEngine* engine) { myMassEngine.setRef(engine); }
  FmEngine* getMassEngine() const { return myMassEngine.getPointer(); }

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual int printSolverEntry(FILE* fp);
  void printLocalPos(FILE* fp, const FmLink* link, int triadID = 0,
                     bool endOfRecord = true) const;
  int printAdditionalMass(FILE* fp);

  virtual void updateChildrenDisplayTopology();

  virtual bool detach() { return this->detach(NULL,true); }
  bool detach(FmLink* fromThisonly, bool notFromDisabledPart = false);

  virtual const char*  getUITypeName() const { return "Triad"; }
  virtual const char** getListViewPixmap() const;

  virtual void getEntities(std::vector<FmSensorChoice>& choicesToFill, int);
  virtual void getDofs(std::vector<FmSensorChoice>& choicesToFill);

  static int checkTriads();
  static int traverseBeam(FmBase* start, std::vector<FmIsPlottedBase*>& triads);
  static FmTriad* createAtNode(FFlNode* node, FmBase* parent,
                               int IDoffset, int& nTriad);

private:
  bool updateFENodeAndDofs(FmPart* ownerPart);
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
  bool updateConnector(ConnectorType type, FmPart* ownerPart = NULL);
#endif

  typedef std::vector<FmHasDOFsBase::DOFStatusEnum> DOFStatusVec;

  FFaFieldUnsignedInt           FENodeNo;
  FFaField<LocalDirectionEnum>  itsLocalDir;
#ifdef FT_USE_CONNECTORS
  FFaField<FFaCompoundGeometry> itsConnectorGeometry;
  FFaField<FFlConnectorItems>   itsConnectorItems;
  FFaField<ConnectorTypeEnum>   itsConnectorType;
#endif

private:
  FFaReferenceList<FmLink>        myAttachedLinks;
  FFaField<FFaReferenceListBase*> myAttachedLinksField;

  FFaReference<FmEngine>      myMassEngine;
  FFaField<FFaReferenceBase*> myMassEngineField;

  FFaField<DOFStatusVec> itsBndC;
  FFaField<DoubleVec>    itsMass;
  FFaField<int>          itsNDOFs; // 0, 3 or 6
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
