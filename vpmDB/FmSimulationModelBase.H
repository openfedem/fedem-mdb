// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*!
  \file FmSimulationModelBase.H
  \brief Base class for all objects that affect the simulation model.
*/

#ifndef FM_SIMULATION_MODEL_BASE_H
#define FM_SIMULATION_MODEL_BASE_H

#include "vpmDB/FmModelMemberBase.H"

class FmSimulationEvent;


/*!
  \brief Base class for all objects that affect the simulation model.
  \details This class does not add new fields to the class hierarchy,
  but contains methods for handling simulation event manipulations.
*/

class FmSimulationModelBase : public FmModelMemberBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
  //! \brief Hook for initialisation after resolving cross references.
  virtual void initAfterResolve();

  //! \brief Hook for list view filtering.
  virtual bool isListable() const { return false; }

  //! \brief Hook for simulation event handling.
  virtual void onEventSwitched(const FmSimulationEvent*) {}
  //! \brief Hook for initialisation after simulation event switching.
  virtual void initAfterParse() {}

  //! \brief Parses the field \a keyWord from a text string.
  bool parseField(const std::string& keyWord, const std::string& fieldValue);

  //! \brief Removes all fields from this object, except for the specified ones.
  void removeFieldsExceptFor(const std::vector<FDictIt>& keep);
  //! \brief Updates default values to the current value for specified fields.
  void setAsDefault(const std::vector<FDictIt>& fieldNames);
  //! \brief Returns all field names present in this object.
  void getFields(std::vector<FDictIt>& fieldNames) const;

  //! \brief Checks if \a *this is referred by simulation events.
  bool isEventModified() const;

  //! \brief Returns a string with all information stored in this object.
  std::string getObjectInfo() const;

  //! \brief Makes a copy of this object.
  FmSimulationModelBase* copy(int cloneDepth) const;

  // From FmBase
  virtual FmBase* clone(int depth) const { return this->copy(depth); }
  using FmBase::clone;

  static std::string relPathCorrection; //!< For writing of solver input file

protected:
  //! \brief Default constructor.
  FmSimulationModelBase();

  // From FmBase
  virtual bool cloneLocal(FmBase* obj, int depth);
};


/*!
  \brief Base class for structural property objects without physical shape.
*/

class FmStructPropertyBase : public FmSimulationModelBase
{
  //! \cond DO_NOT_DOCUMENT
  Fmd_DB_HEADER_INIT();
  //! \endcond

public:
  // From FmSimulationModelBase
  virtual bool isListable() const { return true; }

  // From FmModelMemberBase
  virtual FmBase* duplicate() const;

protected:
  //! \brief Default constructor.
  FmStructPropertyBase();
};

#endif
