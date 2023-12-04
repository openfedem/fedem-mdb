// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_USER_DEFINED_ELEMENT_H
#define FM_USER_DEFINED_ELEMENT_H

#include "vpmDB/FmLink.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class FmUserDefinedElement : public FmLink
{
  Fmd_DB_HEADER_INIT();

public:
  FmUserDefinedElement();

  void init(int eType, const char* typeName,
            const std::vector<FmTriad*>& triads);

  void clearWork();

  virtual const char* getUITypeName() const { return myName.getValue().c_str(); }
  virtual const char* getItemName() const { return "User-defined element"; }

  // DB Topology

  virtual int getNumberOfTriads() const { return myTriads.size(); }
  virtual FmTriad* findTriad(int baseID) const;
  virtual void getTriads(std::vector<FmTriad*>& toFill,
                         bool sortOnId = false) const;

  virtual const FaMat34& getLocalCS() const;
  virtual void setLocalCS(const FaMat34&) {}

  virtual FaMat34 getGlobalCS() const;
  virtual void    setGlobalCS(const FaMat34&, bool = false) {}

  virtual FaVec3  getTranslation() const;
  virtual void    setTranslation(const FaVec3&) {}

  virtual FaMat33 getOrientation() const;
  virtual void    setOrientation(const FaMat33&) {}

  virtual void updateLocation(bool) {}

  virtual FaMat34 getPositionCG(bool globalCS = true) const;
  virtual double getMass() const;

  // Basic DB methods

  static  bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual bool writeToVTF(VTFAFile&, IntVec*, IntVec*);

  virtual int printSolverEntry(FILE* fp);

protected:
  virtual ~FmUserDefinedElement() { this->clearWork(); }

  FmUserDefinedElement(const FmUserDefinedElement&) = delete;
  FmUserDefinedElement& operator=(const FmUserDefinedElement&) = delete;

  virtual bool cloneLocal(FmBase* obj, int depth);

  int initWork(int nenod, int nedof,
               const double* X = NULL, const double* T = NULL) const;

public:
  FFaField<int>                   myType;
  FFaField<std::string>           myName;
  FFaReferenceList<FmTriad>       myTriads;

private:
  FFaField<FFaReferenceListBase*> myTriadsField;

  mutable int*    iwork;
  mutable double* rwork;
};

#endif
