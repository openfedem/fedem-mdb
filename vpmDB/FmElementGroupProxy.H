// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ELEMENT_GROUP_PROXY_H
#define FM_ELEMENT_GROUP_PROXY_H

#include "vpmDB/FmSimulationModelBase.H"
#include "vpmDB/FmPart.H"
#include "FFaLib/FFaContainers/FFaReference.H"

class FFlNamedPartBase;


class FmElementGroupProxy : public FmSimulationModelBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmElementGroupProxy();

  virtual std::string getUserDescription(size_t = 0) const;
  virtual bool setUserDescription(const std::string& descr);
  virtual std::string getIdPath(bool withBrackets) const;
  virtual std::string getInfoString() const;

  std::string getGroupId() const;

  void setRealObject(FFlNamedPartBase* group);
  FFlNamedPartBase* getRealObject() const { return myElementGroup; }
  FmPart* getOwner() const { return myOwner.getPointer(); }

  bool connect(FmBase* parent);
  using FmSimulationModelBase::connect;
  virtual bool disconnect();

  const std::string& getTypeName() const { return myTypeName.getValue(); }

  void toggleFatigue(bool onOff);
  bool doFatigue() const { return useFatigueOpt.getValue(); }

  virtual const char* getUITypeName() const { return "Element group"; }
  virtual const char** getListViewPixmap() const;
  virtual bool isListable() const { return true; }

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  FFaField<int>    myFatigueSNCurve;
  FFaField<int>    myFatigueSNStd;
  FFaField<double> myFatigueSCF;

protected:
  virtual ~FmElementGroupProxy();

private:
  FFaReference<FmPart>        myOwner;
  FFaField<FFaReferenceBase*> myOwnerField;

  FFaField<bool>        useFatigueOpt;
  FFaField<std::string> myTypeName;
  FFlNamedPartBase*     myElementGroup;
  bool                  saveGroup;
};

#endif
