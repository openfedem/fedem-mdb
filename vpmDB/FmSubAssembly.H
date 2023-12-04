// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SUB_ASSEMBLY_H
#define FM_SUB_ASSEMBLY_H

#include "vpmDB/FmModelMemberBase.H"

class FmRingStart;
class FmFuncTree;
class FaVec3;


class FmSubAssembly : public FmModelMemberBase
{
  Fmd_DB_HEADER_INIT();

  typedef std::map<int,FmRingStart*> FmHeadMap;

public:
  FmSubAssembly(bool isDummy = false);

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "Sub Assembly"; }
  virtual bool isListable(bool resultView = false) const;
  virtual bool isCopyable() const { return true; }

  bool hasObjects(int typeId) const;

  const FmHeadMap* getHeadMap() const { return &myHeadMap; }

  virtual void updateLocation(char = 'A') {}
  virtual double getMass(FaVec3&, bool = true) const { return 0.0; }

  virtual FmBase* duplicate() const { return this->duplicate(NULL); }

  static bool readFMF(const std::string& fileName, FmSubAssembly** = NULL);

  void resolveAfterRead();

protected:
  virtual ~FmSubAssembly();

  bool mergeOldHeadMapAndConnect();
  FmSubAssembly* duplicate(FmSubAssembly* parent) const;

  virtual bool cloneLocal(FmBase* obj, int depth);
  static bool localParse(const char* keyWord, std::istream& activeStatement,
			 FmSubAssembly* obj);

  bool writeFMF(const std::string& fileName) const;

  virtual bool eraseOptions();

public:
  FFaField<std::string> myModelFile; //! Model file for storage of this assembly

  static std::string mainFilePath; //!< Full path to the top-level model file

  static std::pair<int,int> old2newAssID; //!< Parent assembly ID switching

  static FmHeadMap* tmpHeadMap; //!< Temporary ring start map used during import

private:
  FmHeadMap   myHeadMap;  //!< The assemblys typeID to ring start map
  FmFuncTree* myFuncTree; //!< Tree structure for functions
};

#endif
