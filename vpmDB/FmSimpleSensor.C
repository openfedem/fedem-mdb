// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmJointBase.H"
#include "vpmDB/FmAxialSpring.H"
#include "vpmDB/FmAxialDamper.H"
#include "vpmDB/FmcOutput.H"
#include "vpmDB/FmCtrlLine.H"
#ifdef FT_HAS_EXTCTRL
#include "vpmDB/FmExternalCtrlSys.H"
#endif
#include "vpmDB/FmStrainRosette.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdSensor.H"
#endif


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcSIMPLE_SENSOR, FmSimpleSensor, FmSensorBase);


FmSimpleSensor::FmSimpleSensor()
{
  Fmd_CONSTRUCTOR_INIT(FmSimpleSensor);

  FFA_REFERENCE_FIELD_INIT(itsMeasuredPtField, itsMeasuredPt, "MEASURED");

#ifdef USE_INVENTOR
  itsDisplayPt = new FdSensor(this);
#endif
}


FmSimpleSensor::~FmSimpleSensor()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

std::string FmSimpleSensor::getInfoString() const
{
  if (itsMeasuredPt.isNull())
    return this->FmSensorBase::getInfoString();

  return itsMeasuredPt->getInfoString();
}


#ifdef FT_HAS_EXTCTRL
bool FmSimpleSensor::isExternalCtrlSys() const
{
  if (itsMeasuredPt.isNull()) return false;

  return itsMeasuredPt->isOfType(FmExternalCtrlSys::getClassTypeID());
}
#endif


bool FmSimpleSensor::isControlOutput() const
{
  if (itsMeasuredPt.isNull()) return false;

  return itsMeasuredPt->isOfType(FmcOutput::getClassTypeID());
}


bool FmSimpleSensor::isDrawable() const
{
  if (itsMeasuredPt.isNull()) return false;

  return (itsMeasuredPt->isOfType(FmTriad::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmJointBase::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmAxialSpring::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmAxialDamper::getClassTypeID()));
}


bool FmSimpleSensor::isListable() const
{
  // Note: return true to get Output List message on interactive erase
  if (itsMeasuredPt.isNull()) return true;

  return !(itsMeasuredPt->isOfType(FmcOutput::getClassTypeID()) ||
#ifdef FT_HAS_EXTCTRL
	   itsMeasuredPt->isOfType(FmExternalCtrlSys::getClassTypeID()) ||
#endif
	   itsMeasuredPt->isOfType(FmEngine::getClassTypeID()));
}


bool FmSimpleSensor::hasEntityChoice() const
{
  if (itsMeasuredPt.isNull()) return false;

  return (itsMeasuredPt->isOfType(FmTriad::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmJointBase::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmAxialSpring::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmAxialDamper::getClassTypeID()) ||
#ifdef FT_HAS_EXTCTRL
	  itsMeasuredPt->isOfType(FmExternalCtrlSys::getClassTypeID()) ||
#endif
	  itsMeasuredPt->isOfType(FmStrainRosette::getClassTypeID()));
}


bool FmSimpleSensor::hasDofChoice() const
{
  if (itsMeasuredPt.isNull()) return false;

  return (itsMeasuredPt->isOfType(FmTriad::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmJointBase::getClassTypeID()) ||
	  itsMeasuredPt->isOfType(FmStrainRosette::getClassTypeID()));
}


void FmSimpleSensor::getSensorEntities(std::vector<FmSensorChoice>& choicesToFill,
				       int dof)
{
  choicesToFill.clear();
  if (!itsMeasuredPt.isNull())
    itsMeasuredPt->getEntities(choicesToFill,dof);
}


void FmSimpleSensor::getSensorDofs(std::vector<FmSensorChoice>& choicesToFill)
{
  choicesToFill.clear();
  if (!itsMeasuredPt.isNull())
    itsMeasuredPt->getDofs(choicesToFill);
}


FmIsMeasuredBase* FmSimpleSensor::getMeasured(int) const
{
  return itsMeasuredPt.getPointer();
}


void FmSimpleSensor::getMeasured(std::vector<FmIsMeasuredBase*>& vectorToFill) const
{
  vectorToFill.clear();
  if (!itsMeasuredPt.isNull())
    vectorToFill = { itsMeasuredPt.getPointer() };
}


void FmSimpleSensor::setMeasured(FmIsMeasuredBase* newPt)
{
  FmIsMeasuredBase* oldPt = itsMeasuredPt.getPointer();
  itsMeasuredPt.setRef(newPt);

  if (oldPt)
    if (oldPt->isOfType(FmTriad::getClassTypeID()))
      static_cast<FmTriad*>(oldPt)->updateDisplayDetails();

  if (newPt)
    if (newPt->isOfType(FmTriad::getClassTypeID()))
      static_cast<FmTriad*>(newPt)->updateDisplayDetails();
}


void FmSimpleSensor::removeMeasured()
{
  itsMeasuredPt.setRef(NULL);
}


bool FmSimpleSensor::eraseOptions()
{
  this->setMeasured(NULL);
  return FmSensorBase::eraseOptions();
}


bool FmSimpleSensor::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmSimpleSensor::getClassTypeID());
}


std::ostream& FmSimpleSensor::writeFMF(std::ostream& os)
{
  os <<"SENSOR\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmSimpleSensor::readAndConnect(std::istream& is, std::ostream&)
{
  FmSimpleSensor* obj = new FmSimpleSensor();

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


bool FmSimpleSensor::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


void FmSimpleSensor::initAfterResolve()
{
  this->FmSensorBase::initAfterResolve();

  this->setMeasured(itsMeasuredPt.getPointer());
}


int FmSimpleSensor::printSolverData(FILE* fp, FmEngine* engine, int iarg) const
{
  // Lambda function to print error message when wrong entity type
  auto&& entityError=[this,engine,iarg](int stat) -> int
  {
    ListUI <<" --> Error: Invalid entity "
           << engine->getEntity(iarg) <<" for "
           << this->getIdString(true) <<"\n";
    return stat;
  };

  if (itsMeasuredPt.isNull())
    return 1;

  else if (itsMeasuredPt->isOfType(FmEngine::getClassTypeID()))
  {
    fprintf(fp,"  type = 'ENGINE'\n");
    fprintf(fp,"  engineId = %d\n", itsMeasuredPt->getBaseID());
  }
  else if (itsMeasuredPt->isOfType(FmcOutput::getClassTypeID()))
  {
    FmcOutput* ctrlOut = static_cast<FmcOutput*>(itsMeasuredPt.getPointer());
    fprintf(fp,"  type = 'CONTROL'\n");
    fprintf(fp,"  ctrlVarId = %d\n", ctrlOut->getLine(1)->getControlVarNo());
  }

#ifdef FT_HAS_EXTCTRL
  else if (itsMeasuredPt->isOfType(FmExternalCtrlSys::getClassTypeID()))
  {
    fprintf(fp, "  type = 'MATLAB_WS'\n");
    fprintf(fp, "  extCtrlSysId = %d\n", itsMeasuredPt->getBaseID());
    fprintf(fp, "  match = '%s'\n", engine->getEntityName(iarg).c_str());
  }
#endif

  else if (itsMeasuredPt->isOfType(FmTriad::getClassTypeID()))
  {
    int dof = engine->getDof(iarg) + 1;
    // Beta feature: Sensor measuring rotation in terms of Rodrigues
    if (engine->getEntity(iarg) == FmIsMeasuredBase::POS && dof >= 4 && dof <= 6 &&
	this->getUserDescription().find("#Rodrig") != std::string::npos)
      dof += 3;
    else if (engine->getEntity(iarg) == FmIsMeasuredBase::WIND_SPEED && dof < 4)
      dof += 3; // dof=4,5,6: Free wind

    fprintf(fp,"  type = 'TRIAD'\n");
    fprintf(fp,"  triad1Id  = %d\n",itsMeasuredPt->getBaseID());
    fprintf(fp,"  dof       = %d\n",dof);

    switch (engine->getEntity(iarg))
      {
      case FmIsMeasuredBase::POS:
	fprintf(fp,"  dofEntity = 'POS'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::GLOBAL_VEL:
	fprintf(fp,"  dofEntity = 'VEL'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::GLOBAL_ACC:
	fprintf(fp,"  dofEntity = 'ACC'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::LOCAL_VEL:
	fprintf(fp,"  dofEntity = 'VEL'\n");
	fprintf(fp,"  dofSystem = 'LOCAL'\n");
	break;
      case FmIsMeasuredBase::LOCAL_ACC:
	fprintf(fp,"  dofEntity = 'ACC'\n");
	fprintf(fp,"  dofSystem = 'LOCAL'\n");
	break;
      case FmIsMeasuredBase::LOCAL_FORCE:
	fprintf(fp,"  dofEntity = 'FORCE'\n");
	fprintf(fp,"  dofSystem = 'LOCAL'\n");
	break;
      case FmIsMeasuredBase::GLOBAL_FORCE:
	fprintf(fp,"  dofEntity = 'FORCE'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::WIND_SPEED:
	fprintf(fp,"  dofEntity = 'W_SPEED'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::FLUID_VEL:
	fprintf(fp,"  dofEntity = 'F_VEL'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::FLUID_ACC:
	fprintf(fp,"  dofEntity = 'F_ACC'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      case FmIsMeasuredBase::DYN_PRESS:
	fprintf(fp,"  dofEntity = 'DYN_P'\n");
	fprintf(fp,"  dofSystem = 'GLOBAL'\n");
	break;
      default:
        return entityError(1);
      }
  }

  else if (itsMeasuredPt->isOfType(FmAxialDamper::getClassTypeID()))
  {
    fprintf(fp,"  type = 'DAMPER_AXIAL'\n");
    fprintf(fp,"  damperId  = %d\n",itsMeasuredPt->getBaseID());
    switch (engine->getEntity(iarg))
      {
      case FmIsMeasuredBase::LENGTH:
	fprintf(fp,"  dofEntity = 'LENGTH'\n");
	break;
      case FmIsMeasuredBase::VEL:
	fprintf(fp,"  dofEntity = 'VEL'\n");
	break;
      case FmIsMeasuredBase::FORCE:
	fprintf(fp,"  dofEntity = 'FORCE'\n");
	break;
      default:
        return entityError(1);
      }
  }

  else if (itsMeasuredPt->isOfType(FmAxialSpring::getClassTypeID()))
  {
    fprintf(fp,"  type = 'SPRING_AXIAL'\n");
    fprintf(fp,"  springId  = %d\n",itsMeasuredPt->getBaseID());
    switch (engine->getEntity(iarg))
      {
      case FmIsMeasuredBase::LENGTH:
	fprintf(fp,"  dofEntity = 'LENGTH'\n");
	break;
      case FmIsMeasuredBase::DEFL:
	fprintf(fp,"  dofEntity = 'DEFL'\n");
	break;
      case FmIsMeasuredBase::FORCE:
	fprintf(fp,"  dofEntity = 'FORCE'\n");
	break;
      default:
        return entityError(1);
      }
  }

  else if (itsMeasuredPt->isOfType(FmJointBase::getClassTypeID()))
  {
    FmJointBase* joint = static_cast<FmJointBase*>(itsMeasuredPt.getPointer());
    int thisDof = engine->getDof(iarg);
    int thisEnt = engine->getEntity(iarg);

    switch (thisEnt)
      {
      case FmIsMeasuredBase::REL_POS:
      case FmIsMeasuredBase::VEL:
      case FmIsMeasuredBase::ACCEL:
      case FmIsMeasuredBase::FORCE:
	fprintf(fp,"  type = 'JOINT_VARIABLE'\n");
	fprintf(fp,"  jointId   = %d\n",joint->getBaseID());
	fprintf(fp,"  dof       = %d\n",thisDof+1);
	switch (thisEnt)
	  {
	  case FmIsMeasuredBase::REL_POS:
	    fprintf(fp,"  dofEntity = 'REL_POS'\n");
	    break;
	  case FmIsMeasuredBase::VEL:
	    fprintf(fp,"  dofEntity = 'VEL'\n");
	    break;
	  case FmIsMeasuredBase::ACCEL:
	    fprintf(fp,"  dofEntity = 'ACC'\n");
	    break;
	  case FmIsMeasuredBase::FORCE:
	    fprintf(fp,"  dofEntity = 'FORCE'\n");
	    break;
	  }
	break;

      case FmIsMeasuredBase::JSPR_ANG:
      case FmIsMeasuredBase::JSPR_DEFL:
      case FmIsMeasuredBase::JSPR_FORCE:
	fprintf(fp,"  type = 'SPRING_JOINT'\n");
	fprintf(fp,"  springId  = %d\n",joint->getSpringBaseID(thisDof));
	switch (thisEnt)
	  {
	  case FmIsMeasuredBase::JSPR_ANG:
	    fprintf(fp,"  dofEntity = 'LENGTH'\n");
	    break;
	  case FmIsMeasuredBase::JSPR_DEFL:
	    fprintf(fp,"  dofEntity = 'DEFL'\n");
	    break;
	  case FmIsMeasuredBase::JSPR_FORCE:
	    fprintf(fp,"  dofEntity = 'FORCE'\n");
	    break;
	  }
	break;

      case FmIsMeasuredBase::JDAMP_ANG:
      case FmIsMeasuredBase::JDAMP_VEL:
      case FmIsMeasuredBase::JDAMP_FORCE:
	fprintf(fp,"  type = 'DAMPER_JOINT'\n");
	fprintf(fp,"  damperId  = %d\n",joint->getDamperBaseID(thisDof));
	switch (thisEnt)
	  {
	  case FmIsMeasuredBase::JDAMP_ANG:
	    fprintf(fp,"  dofEntity = 'LENGTH'\n");
	    break;
	  case FmIsMeasuredBase::JDAMP_VEL:
	    fprintf(fp,"  dofEntity = 'VEL'\n");
	    break;
	  case FmIsMeasuredBase::JDAMP_FORCE:
	    fprintf(fp,"  dofEntity = 'FORCE'\n");
	    break;
	  }
	break;

      default:
        return entityError(1);
      }
  }

  else if (itsMeasuredPt->isOfType(FmStrainRosette::getClassTypeID()))
  {
    fprintf(fp,"  type = 'STRAIN_GAGE'\n");
    fprintf(fp,"  engineId  = %d\n",itsMeasuredPt->getBaseID());
    fprintf(fp,"  dof       = %d\n",engine->getDof(iarg)-FmIsMeasuredBase::MAX_PR+1);
    switch (engine->getEntity(iarg))
      {
      case FmIsMeasuredBase::STRAIN:
	fprintf(fp,"  dofEntity = 'STRAIN'\n");
	break;
      case FmIsMeasuredBase::STRESS:
	fprintf(fp,"  dofEntity = 'STRESS'\n");
	break;
      default:
        return entityError(1);
      }
  }
  else
  {
    ListUI <<" --> Error: Invalid object type ("
           << itsMeasuredPt->getUITypeName() <<") for "
           << this->getIdString(true) <<"\n";
    return 1;
  }

  return 0;
}
