// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmBase.H
  \brief Base parent class for the model data structure entries.
*/

#ifndef FM_BASE_H
#define FM_BASE_H

#include "FFaLib/FFaContainers/FFaReference.H"
#include "FFaLib/FFaContainers/FFaField.H"
#include "FFaLib/FFaContainers/FFaFieldContainer.H"

#include "vpmDB/FmArray.H"
#include "vpmDB/FmTypeCheck.H"

class FmRingStart;
class FmSubAssembly;


/*!
  \brief Base parent class for the model data structure entries.
  \details Very little functionality is included in this class.
  It is mostly concerned with the user ID and description,
  and pointers used for storing objects in the linked lists
  which constitute the model database.
  \author Jens LIEN
*/

class FmBase : public FFaFieldContainer
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
  //! \brief Returns the next object in the linked list.
  FmBase* getNext() const { return itsNextRingPt; }
  //! \brief Returns the previous object in the linked list.
  FmBase* getPrev() const { return itsPrevRingPt; }

  // Description text

  //! \brief Returns the user-defined description of this object.
  virtual std::string getUserDescription(size_t maxLength = 0) const;
  //! \brief Sets the user-defined description of this object.
  virtual bool setUserDescription(const std::string& descr);

  //! \brief Prepends "Copy of " to the user description.
  bool makeCopyDescr();

  //! \brief Returns a composed string describing this object.
  virtual std::string getInfoString() const;
  //! \brief Returns the user ID including the sub-assembly path.
  virtual std::string getIdPath(bool withBrackets = true) const;
  //! \brief Returns a composed string describing this object.
  std::string getIdString(bool withDescription = false) const;

  //! \brief Returns the user ID of this object.
  int  getID() const { return myID.getValue(); }
  //! \brief Sets the user ID of this object.
  void setID(int Id) { myID.setValue(Id); }

  //! \brief Retrieves the assembly ID of this object.
  void getAssemblyID(IntVec& assID) const;
  //! \brief Returns the sub-assembly this object is part of, if any.
  FmBase* getParentAssembly() const { return myParentAssembly.getPointer(); }
  //! \brief Returns the sub-assembly both \a *this and \a *other descent from.
  FmBase* getCommonAncestor(const FmBase* other) const;
  //! \brief Sets the parent sub-assembly for this object.
  void setParentAssembly(FmBase* subAss);
  //! \brief Sets the parent sub-assembly for this object.
  void setParentAssembly(int Id, int classType = -1);

  // Reimplemented from FFaFieldContainer

  virtual int getResolvedID() const { return this->getID(); }
  //! \copybrief FFaFieldContainer::getResolvedAssemblyID()
  virtual void getResolvedAssemblyID(IntVec& assID) const;

  // Topology

  //! \brief Inserts or copies this object into the model database.
  bool cloneOrConnect();
  //! \brief Inserts this object into the model database.
  virtual bool connect() { return this->mainConnect(); }
  //! \brief Removes this object from the model database.
  virtual bool disconnect() { return this->mainDisconnect(); }
  //! \brief Visualization interface for objects that can be rendered.
  virtual void drawObject() {}

  //! \brief Returns \e true if \a *this is part of the specified sub-assembly.
  bool isPartOf(const FmSubAssembly* subAss) const;

  // File IO

  //! \brief Writes all data fields of this object to the given output stream.
  std::ostream& writeFields(std::ostream& os) const;

  //! \brief Writes out this object in the model database file format.
  virtual std::ostream& writeFMF(std::ostream& os) { return os; }
  //! \brief Copies the fields of another object into \a *this.
  virtual bool clone(FmBase*, int) { return false; }
  //! \brief Returns an unconnected copy of this object.
  virtual FmBase* clone(int) const { return NULL; }

  //! \brief Returns the type name of this object for use in the GUI.
  virtual const char* getUITypeName() const;

  //! \brief Invokes call-backs after this object is inserted into the database.
  virtual void onMainConnected() {}
  //! \brief Invokes call-backs after this object is removed from the database.
  virtual void onMainDisconnected() {}
  //! \brief Invokes call-backs before this object is removed from the database.
  virtual void onMainAboutToDisconnect() {}

  //! \brief Enumerator for the different copying modes.
  enum CopyMode {
    NOTHING,         //!< Don't copy any data fields
    SHALLOW,         //!< Copy all data fields, but not the references
    DEEP_UNRESOLVED, //!< Also copy references, but leave them unresolved
    DEEP_APPEND,     //!< Also copy and resolve references
    DEEP_REPLACE     //!< Also copy references, replacing existing ones
  };

private:
  FFaField<std::string> myDescription; //!< User-defined description text
  FFaNoCopyField<int>   myID;          //!< User ID, unique only within one type

  //! Reference to the parent sub-assembly of this object, if any
  FFaReference<FmBase>        myParentAssembly;
  //! Data field for the reference to the parent sub-assembly
  FFaField<FFaReferenceBase*> myParentAssemblyField;

  FmBase* itsNextRingPt; //!< Pointer to the next entry in the main ring
  FmBase* itsPrevRingPt; //!< Pointer to the previous entry in the main ring

protected:
  //! \brief Default constructor.
  explicit FmBase(bool isDummy = false);
  //! \brief Empty destructor.
  virtual ~FmBase();

  //! \brief Conditionally inserts this object into the model database.
  bool eraseOrConnect();
  //! \brief Inserts this object into the model database, unless aready present.
  bool mainConnect(bool allowNonUniqueIDs = false);
  //! \brief Removes this object from the model database, if present.
  bool mainDisconnect();

  //! \brief Inserts \a *this after \a *afterPt in the database ring.
  void insertAfter(FmBase* afterPt);

  //! \brief Parses the \a keyWord field for \a *obj, if it is a defined field.
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
                          FmBase* obj);
  //! \brief Clones the data fields of \a *obj into *this.
  virtual bool cloneLocal(FmBase* obj, int depth);
};

#endif
