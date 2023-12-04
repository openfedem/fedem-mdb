// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSticker.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdSticker.H"
#include "vpmDisplay/FaDOF.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcSTICKER, FmSticker, FmIsRenderedBase);


FmSticker::FmSticker(const FaVec3& pos)
{
  Fmd_CONSTRUCTOR_INIT(FmSticker);

  this->removeField("BASE_ID"); // Ignore the base ID field on read/write

  FFA_REFERENCE_FIELD_INIT(myOwnerField, myOwner, "OWNER");

  FFA_FIELD_INIT(myPosition, pos, "POSITION");

#ifdef USE_INVENTOR
  itsDisplayPt = new FdSticker(this);
#endif
}


FmSticker::~FmSticker()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmSticker::connect(FmBase* parent)
{
  bool status = this->mainConnect();
  if (status && parent)
    if (parent->isOfType(FmIsPositionedBase::getClassTypeID()))
      myOwner = (FmIsPositionedBase*)parent;

  return status;
}


bool FmSticker::disconnect()
{
  bool status = this->mainDisconnect();
  myOwner = NULL;

  return status;
}


/***********************************************************************
 *
 * Input and output from stream.
 *
 ************************************************************************/

std::ostream& FmSticker::writeFMF(std::ostream& os)
{
  os <<"STICKER\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmSticker::readAndConnect(std::istream& is, std::ostream&)
{
  FmSticker* obj = new FmSticker();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  obj->connect();
  return true;
}


bool FmSticker::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmSticker::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmSticker::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  this->disconnect();
  this->connect(static_cast<FmSticker*>(obj)->getStuckObject());

  return true;
}


#ifdef USE_INVENTOR
FaDOF FmSticker::getObjDegOfFreedom() const
{
  return FaDOF(myPosition.getValue(),FaVec3(0.0,0.0,1.0),FaDOF::BALL);
}
#endif
