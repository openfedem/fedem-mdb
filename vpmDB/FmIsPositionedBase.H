// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * CLASS: FmIsPositionedBase
 * AUTHOR: Jens LIEN
 *
 * DESCRIPTION:
 * Parent class for the database entries.
 * Has an additional element for the positioning.
 ******************************************************************************/

#ifndef FM_IS_POSITIONED_BASE_H
#define FM_IS_POSITIONED_BASE_H

#include "vpmDB/FmIsPlottedBase.H"
#include "FFaLib/FFaContainers/FFaReference.H"
#include "FFaLib/FFaAlgebra/FFa3DLocation.H"
#include "FFaLib/FFaAlgebra/FFaMat34.H"
#include "FFaLib/FFaAlgebra/FFaMat33.H"

class FmSticker;
class FmAssemblyBase;


class FmIsPositionedBase : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual const FaMat34& getLocalCS() const { return myCS.getValue(); }
  virtual void setLocalCS(const FaMat34& localMat);

  virtual FaMat34 getGlobalCS() const;
  virtual void    setGlobalCS(const FaMat34& globalMat, bool = false);

  virtual FaVec3  getTranslation() const;
  virtual void    setTranslation(const FaVec3& translation);

  virtual FaMat33 getOrientation() const;
  virtual void    setOrientation(const FaMat33& orientation);

  virtual void getStickers(std::vector<FmSticker*>& stickers) const;

  bool hasStickers() const;

  bool addSticker(FmSticker* sticker);
  bool addStickers(const std::vector<FmSticker*>& stickers);
  bool removeSticker(FmSticker* sticker);
  void removeAllStickers();
  int  getNumberOfStickers() const;

  virtual void updateChildrenDisplayTopology();

  virtual std::string getLinkIDString(bool = false) const { return ""; }

  virtual bool isTranslatable() const { return false; }
  virtual bool isRotatable() const { return false; }

  FmAssemblyBase* getPositionedAssembly() const;

  bool setPosRef(FmIsPositionedBase* ref);
  FmIsPositionedBase* getPosRef() const { return myPosRef.getPointer(); }

  bool setRotRef(FmIsPositionedBase* ref);
  FmIsPositionedBase* getRotRef() const { return myRotRef.getPointer(); }

  void setLocation(const FFa3DLocation& loc);
  const FFa3DLocation& getLocation() const { return myLocation.getValue(); }

  virtual void updateLocation(bool updateReferringObjs = true);

protected:
  FmIsPositionedBase();
  virtual ~FmIsPositionedBase();

  void getLocalStickers(std::vector<FmSticker*>& stickers) const;

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  FmIsPositionedBase* obj);

  FFaReference<FmIsPositionedBase> myPosRef;
  FFaField<FFaReferenceBase*>      myPosRefField;

  FFaReference<FmIsPositionedBase> myRotRef;
  FFaField<FFaReferenceBase*>      myRotRefField;

  FFaField<FFa3DLocation> myLocation;

public: // To allow the earth link CS field be added to the FmMechanism object
  FFaField<FaMat34> myCS;

private:
  bool IAmSettingLocation;
};

#endif
