// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdLinJoint.H"
#endif

#include "vpmDB/FmCylJoint.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcCYL_JOINT, FmCylJoint, FmMMJointBase);

FmCylJoint::FmCylJoint()
{
  Fmd_CONSTRUCTOR_INIT(FmCylJoint);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdLinJoint(this);
#endif
  myLegalDOFs[Z_TRANS] = true;
  myLegalDOFs[Z_ROT] = true;
  this->completeInitJVars();

  FFA_FIELD_INIT(myScrewTransFlag, false, "HAS_TRANS");
  FFA_FIELD_INIT(myScrewTransRatio,  1.0, "TRANS_OUTPUTRATIO");
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

double FmCylJoint::getScrewRatio() const
{
  if (myScrewTransFlag.getValue())
    return myScrewTransRatio.getValue();
  else
    return 0.0;
}


bool FmCylJoint::setScrewRatio(double ratio)
{
  if (myScrewTransFlag.getValue())
    myScrewTransRatio.setValue(ratio);

  return myScrewTransFlag.getValue();
}


std::ostream& FmCylJoint::writeFMF(std::ostream& os)
{
  os <<"CYL_JOINT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmCylJoint::readAndConnect(std::istream& is, std::ostream&)
{
  FmCylJoint* obj = new FmCylJoint();

  // Old model files without the DOFStatus field
  // should be initialized to SPRING_CONSTRAINED
  for (int i = 0; i < MAX_DOF; i++)
    if (obj->isLegalDOF(i))
      obj->myDofStatus[i] = SPRING_CONSTRAINED;

  // Obsolete field
  FFaObsoleteField<double> transRatio;
  FFA_OBSOLETE_FIELD_INIT(transRatio,0.0,"TRANS_RATIO",obj);

  while (is.good())
    {
      std::stringstream activeStatement;
      char keyWord[BUFSIZ];
      if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
	parentParse(keyWord, activeStatement, obj);
    }

  FFA_OBSOLETE_FIELD_REMOVE("TRANS_RATIO",obj);

  if (transRatio.getValue() != 0.0)
    obj->myScrewTransRatio.setValue(1.0/transRatio.getValue());

  obj->connect();
  return true;
}


bool FmCylJoint::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmCylJoint::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmCylJoint::getClassTypeID());
}
