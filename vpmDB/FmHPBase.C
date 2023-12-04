// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmHPBase.H"
#include "vpmDB/FmRevJoint.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcHP_BASE, FmHPBase, FmIsRenderedBase);

FmHPBase::FmHPBase()
{
  Fmd_CONSTRUCTOR_INIT(FmHPBase);

  FFA_REFERENCE_FIELD_INIT(itsInputJointField, itsInputJoint, "INPUT_JOINT");

  FFA_FIELD_INIT(itsHPRatio, 1.0, "OUTPUT_RATIO");
}


FmHPBase::~FmHPBase()
{
  itsInputJoint.setPointer(0);
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

bool FmHPBase::setInputJoint(FmJointBase* jnt)
{
  if (jnt->getHPConnection())
    return false; // the joint already has a higher pair connection

  itsInputJoint.setPointer(jnt);
  return true;
}


FmJointBase* FmHPBase::getInputJoint() const
{
  return itsInputJoint.getPointer();
}


bool FmHPBase::setTransmissionRatio(double ratio)
{
  itsHPRatio.setValue(ratio);
  return true;
}


double FmHPBase::getTransmissionRatio() const
{
  return itsHPRatio.getValue();
}


bool FmHPBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmHPBase::getClassTypeID());
}


bool FmHPBase::localParse(const char* keyWord, std::istream& activeStatement,
			  FmHPBase* obj)
{
  bool retVal;
  if (strcmp(keyWord,"INPUT_REV_JOINT") == 0)
  {
    // For old model file compatibility (R4.2.1 and earlier)
    retVal = parentParse("INPUT_JOINT", activeStatement, obj);
    if (obj->itsInputJoint.getRefID() > 0)
      obj->itsInputJoint.setRef(obj->itsInputJoint.getRefID(),
				FmRevJoint::getClassTypeID());
  }
  else if (strcmp(keyWord,"OUTPUTRATIO") == 0)
    retVal = parentParse("OUTPUT_RATIO", activeStatement, obj);
  else if (strcmp(keyWord,"RATIO") == 0)
  {
    // For old model file compatibility (R2.1.2 and earlier)
    retVal = parentParse("OUTPUT_RATIO", activeStatement, obj);
    if (obj->itsHPRatio.getValue() != 0.0)
      obj->itsHPRatio.setValue(1.0/obj->itsHPRatio.getValue());
  }
  else
    retVal = parentParse(keyWord, activeStatement, obj);

  return retVal;
}


int FmHPBase::printSolverEntry(FILE* fp)
{
  fprintf(fp,"! %s transmission internal connection\n",this->getUITypeName());
  fprintf(fp,"&HIGHER_PAIR\n");
  this->printID(fp);
  fprintf(fp,"  slaveJoint     = %d\n",this->getOutputJoint()->getBaseID());
  fprintf(fp,"  slaveJointDof  = %d\n",this->getOutputJointDof());
  fprintf(fp,"  masterJoint    = %d\n",this->getInputJoint()->getBaseID());
  fprintf(fp,"  masterJointDof = %d\n",this->getInputJointDof());
  fprintf(fp,"  coeff          = %17.9e\n",itsHPRatio.getValue());
  fprintf(fp,"/\n\n");
  return 0;
}
