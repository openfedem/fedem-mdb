// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_BEAM_H
#define FM_BEAM_H

#include "vpmDB/FmLink.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"

class FmBeamProperty;
class FmBladeProperty;


class FmBeam : public FmLink
{
  Fmd_DB_HEADER_INIT();

public:
  typedef std::pair<int,int> Ints;

  FmBeam();

  bool connect(FmTriad* tr1, FmTriad* tr2);
  using FmLink::connect;

  void setTriad(FmTriad* tr, int pos = 0) { myTriads.setPtr(tr,pos); }
  void setTriads(FmTriad* tr1, FmTriad* tr2);
  bool replaceTriad(FmTriad* oldTr, FmTriad* newTr);
  void removeTriads();

  FmTriad* getFirstTriad() const;
  FmTriad* getSecondTriad() const;
  FmTriad* getOtherTriad(FmTriad* tr) const;

  virtual const char* getUITypeName() const { return "Beam"; }

  // DB Topology

  virtual int getNumberOfTriads() const { return 2; }
  virtual FmTriad* findTriad(int baseID) const;
  virtual void getTriads(std::vector<FmTriad*>& toFill,
                         bool sortOnId = false) const;

  virtual const FaMat34& getLocalCS() const;
  virtual void setLocalCS(const FaMat34& localMat);

  virtual FaMat34 getGlobalCS() const;
  virtual void    setGlobalCS(const FaMat34& globalMat, bool = false);

  virtual FaVec3  getTranslation() const;
  virtual void    setTranslation(const FaVec3& translation);

  virtual FaMat33 getOrientation() const;
  virtual void    setOrientation(const FaMat33& orientation);

  FaMat33 getGlobalOrientation() const;
  void setOrientation(const FaVec3& Zaxis) { myLocalZaxis.setValue(Zaxis); }

  virtual void updateLocation(bool) {}

  virtual bool getRefPoints(FmTriad*& node1, FmTriad*& node2, FmTriad*& node3,
                            FaVec3& offset1, FaVec3& offset2, FaVec3& offset3) const;

  virtual double getMass() const;
  virtual FaMat34 getPositionCG(bool globalCS = true) const;
  virtual FmModelMemberBase* getProperty() const { return myProp.getPointer(); }
  FmBeamProperty* getBeamProperty() const;
  FmBladeProperty* getBladeProperty() const;
  void setProperty(FmModelMemberBase* prop);

  double getInternalFluidDensity(bool checkLevel = false) const;
  double getLength() const;

  static int traverse(FmBase* start, std::vector<FmIsPlottedBase*>& objs);
  int traverse(FmTriad* triad, std::vector<FmIsPlottedBase*>& objs) const;

  // Basic DB methods

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();

  using FmLink::printSolverEntry;
  int printSolverEntry(FILE* fp, int propId, FmBeamProperty* bProp, const std::string* rdbPath);
  virtual int printSolverEntry(FILE* fp, int propId) { return this->printSolverEntry(fp,propId,NULL,NULL); }
  virtual bool writeToVTF(VTFAFile& file, IntVec*, IntVec*);

  // Splits the beam into several beams

  bool split(size_t nb) { return this->split(DoubleVec(nb,1.0)); }
  bool split(const DoubleVec& rlen);

protected:
  virtual ~FmBeam() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

  virtual bool attachSMJoint(FmSMJointBase* attachJnt, bool isSilent = false);
  virtual bool attachMMJoint(FmMMJointBase* attachJnt, bool isSilent = false);

private:
  FFaReferenceList<FmTriad>       myTriads;
  FFaField<FFaReferenceListBase*> myTriadsField;

  FFaReference<FmModelMemberBase> myProp;
  FFaField<FFaReferenceBase*>     myPropField;

public:
  FFaField<FaVec3> myLocalZaxis;
  FFaField<bool>   myVisualize3D;
  FFaField<Ints>   myVisualize3DAngles;
};

#endif
