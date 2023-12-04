// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MM_JOINT_BASE_H
#define FM_MM_JOINT_BASE_H

#include "vpmDB/FmJointBase.H"
#include "vpmDB/Fm1DMaster.H"


class FmMMJointBase : public FmJointBase
{
  Fmd_DB_HEADER_INIT();

public:
  void setMaster(Fm1DMaster* master) { myMaster.setRef(master); }
  Fm1DMaster* getMaster() const { return myMaster.getPointer(); }

  bool insertAsMaster(FmTriad* triad, size_t pos);

  static  bool addMasterTriad(const FaVec3& globalPoint);
  virtual bool addMasterOnPoint(const FaVec3& globalPoint);
  virtual bool addAsMasterTriad(FmTriad* triad);

  virtual void getMasterTriads(std::vector<FmTriad*>& triadsToFill) const;
  virtual bool isMasterTriad(const FmTriad* triad) const;

  FmTriad* getFirstMaster() const;
  FmTriad* getLastMaster() const;

  virtual bool isMasterAttachedToLink(bool allowMultipleLinks = false) const;
  virtual FmLink* getMasterLink() const;
  virtual FmPart* getMasterPart(bool noEarth = false) const;

  virtual FaVec3 getTransJointVariables() const;
  virtual FaVec3 getRotJointVariables() const;
  virtual void setRotJointVariables(const FaVec3& rotations);

  virtual bool detach();

  bool isLooping() const;

  static void connectTmpMasters();

protected:
  FmMMJointBase();
  virtual ~FmMMJointBase();

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmMMJointBase* obj);

  virtual bool eraseOptions();

  FFaReference<Fm1DMaster>    myMaster;
  FFaField<FFaReferenceBase*> myMasterField;

public:
  static Fm1DMaster* editedMaster; // used by add master command

private:
  static std::vector<Fm1DMaster*> tmpMasters; // for conversion of old model files
};

#endif
