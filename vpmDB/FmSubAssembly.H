// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmSubAssembly.H
  \brief Base class for all sub-assembly objects.
*/

#ifndef FM_SUB_ASSEMBLY_H
#define FM_SUB_ASSEMBLY_H

#include "vpmDB/FmModelMemberBase.H"

class FmRingStart;
class FmFuncTree;
class FaVec3;


/*!
  \brief Base class for all sub-assembly objects.
*/

class FmSubAssembly : public FmModelMemberBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

  using FmHeadMap = std::map<int,FmRingStart*>; //!< Convenience type alias

public:
  //! \brief Default constructor.
  FmSubAssembly(bool isDummy = false);

  //! \brief Creates a new sub-assmbly object by reading data from a stream.
  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);

  // From FmBase
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);
  virtual const char* getUITypeName() const { return "Sub Assembly"; }

  //! \brief Returns \e true if this object should be visible in the tree views.
  virtual bool isListable(bool resultView = false) const;
  //! \brief Returns \e true if this sub-assembly object can be copied.
  virtual bool isCopyable() const { return true; }

  //! \brief Returns \e true if this sub-assembly has objects of the given type.
  bool hasObjects(int typeId) const;

  //! \brief Returns a pointer to the #myHeadMap member of this sub-assembly.
  const FmHeadMap* getHeadMap() const { return &myHeadMap; }

  //! \brief Updates the location data for positioned sub-assemblies.
  virtual void updateLocation(char = 'A') {}
  //! \brief Returns the total mass of this sub-assembly
  virtual double getMass(FaVec3&, bool = true) const { return 0.0; }

  // From FmModelMemberBase
  virtual FmBase* duplicate() const { return this->duplicate(NULL); }

  //! \brief Creates sub-assembly members by reading from specified file.
  static bool readFMF(const std::string& fileName,
                      FmSubAssembly** subAss = NULL);

  //! \brief Resolves topological cross-references after finished parsing.
  void resolveAfterRead();

protected:
  //! \brief The destructor disconnects the object from the model database.
  virtual ~FmSubAssembly();

  //! \brief Merges existing sub-assembly with same ID (if any) into this one.
  bool mergeOldHeadMapAndConnect();
  //! \brief Makes a copy of this sub-assebly with all its members.
  FmSubAssembly* duplicate(FmSubAssembly* parent) const;

  //! \brief Writes out this sub-assembly to the specified file.
  bool writeFMF(const std::string& fileName) const;

  // From FmBase
  virtual bool cloneLocal(FmBase* obj, int depth);
  virtual bool eraseOptions();

public:
  FFaField<std::string> myModelFile; //! Model file for storing the sub-assembly

  static std::string mainFilePath; //!< Full path to the top-level model file
  static std::string newLinkRepo; //!< Model link repository of imported model

  static std::pair<int,int> old2newAssID; //!< Parent sub-assembly ID switching

  static FmHeadMap* tmpHeadMap; //!< Temporary ring start map used during import

private:
  FmHeadMap   myHeadMap;  //!< The sub-assemblys typeID to ring start map
  FmFuncTree* myFuncTree; //!< Tree structure for functions
};

#endif
