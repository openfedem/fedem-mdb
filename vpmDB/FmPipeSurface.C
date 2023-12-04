// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmPipeSurface.H"
#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdPipeSurface.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcPIPE_SURFACE, FmPipeSurface, FmArcSegmentMaster);


FmPipeSurface::FmPipeSurface() : FmArcSegmentMaster()
{
  Fmd_CONSTRUCTOR_INIT(FmPipeSurface);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdPipeSurface(this);
#endif

  FFA_FIELD_INIT(myVisiblePart, LEFT, "VISIBLE_PART");
  FFA_FIELD_INIT(myRadius,    0.1492, "PIPE_RADIUS");

  FFA_FIELD_INIT(myFacesIsOn, true, "FACES_VISIBLE");
  FFA_FIELD_INIT(myLinesIsOn, true, "LINES_VISIBLE");

  FFA_FIELD_INIT(myColor, FmColor(0.3f,0.3f,0.8f), "COLOR");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

void FmPipeSurface::setPipeRadius(double radius)
{
  myRadius.setValue(radius);
  this->updateDisplayDetails();
}


std::ostream& FmPipeSurface::writeFMF(std::ostream& os)
{
  os << "PIPE_SURFACE\n{\n";
  this->writeFields(os);
  os << "}\n\n";
  return os;
}


bool FmPipeSurface::readAndConnect(std::istream& is, std::ostream&)
{
  FmPipeSurface* obj = new FmPipeSurface();

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	localParse(keyWord, activeStatement, obj);
    }

  obj->connect();
  return true;
}


bool FmPipeSurface::localParse(const char* keyWord, std::istream& activeStatement,
			       FmPipeSurface* obj)
{
  if (strcmp(keyWord,"SURFACE_TRIADS") == 0)
    return parentParse("TRIADS", activeStatement, obj);
  else
    return parentParse(keyWord, activeStatement, obj);
}


bool FmPipeSurface::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmPipeSurface::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmPipeSurface::getClassTypeID());
}
