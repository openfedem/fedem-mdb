// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaFunctionLib/FFaFunctionManager.H"

#include "vpmDB/FmEngine.H"
#include "vpmDB/FmSpringBase.H"
#include "vpmDB/FmDamperBase.H"
#include "vpmDB/FmRoad.H"
#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmVesselMotion.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFuncAdmin.H"
#include "vpmDB/FmMathFuncBase.H"


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcMATH_FUNC_BASE, FmMathFuncBase, FmParamObjectBase);

FmMathFuncBase::FmMathFuncBase()
{
  Fmd_CONSTRUCTOR_INIT(FmMathFuncBase);

  FFA_FIELD_INIT(myUse, NONE, "FUNC_USE");

  myExplType = -1;
}


FmMathFuncBase::~FmMathFuncBase()
{
  std::vector<FmCurveSet*> referredCurves;
  this->getReferringObjs(referredCurves,"myFunction");
  for (FmCurveSet* curve : referredCurves)
    if (curve->usingInputMode() == FmCurveSet::PREVIEW_FUNC)
      curve->erase();
    else
    {
      curve->setFunctionRef(NULL);
      curve->reload();
    }

  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

FmMathFuncBase* FmMathFuncBase::copy() const
{
  FmMathFuncBase* newFunc = FmFuncAdmin::createFunction(this->getTypeID());
  if (newFunc) {
    newFunc->clone((FmBase*)this,FmBase::SHALLOW);
    newFunc->setUserDescription("Copy of " + this->getInfoString());
  }
  return newFunc;
}


int FmMathFuncBase::getExtrapolationType() const
{
  if (this->getFunctionUse() == GENERAL)
  {
    // Beta feature: Check if ramping should be deactivated
    std::vector<FmEngine*> engines;
    this->getEngines(engines);
    for (FmEngine* engine : engines)
      if (engine->getUserDescription().find("#noramp") != std::string::npos)
        return -1;
  }

  return 0;
}


bool FmMathFuncBase::initGetValue()
{
  myExplType = FFaFunctionManager::getTypeID(this->getFunctionFsiName());

  std::vector<FmFuncVariable> fVars;
  this->getFunctionVariables(fVars,true);

  myExplData.clear();
  myExplData.reserve(fVars.size());
  for (const FmFuncVariable& var : fVars)
    myExplData.push_back(var.getFcn(this));

  return true;
}


double FmMathFuncBase::getValue(double x, int& ierr) const
{
  return FFaFunctionManager::getValue(this->getBaseID(), myExplType,
                                      this->getExtrapolationType(),
                                      this->getData(), x, ierr);
}


double FmMathFuncBase::getValue(double g, double d,
                                const FaVec3& X, double t) const
{
  return FFaFunctionManager::getWaveValue(this->getData(),g,d,X,t,myExplType);
}


int FmMathFuncBase::getSmartPoints(double start, double stop,
                                   DoubleVec& x, DoubleVec& y)
{
  if (start > stop)
    return -1;
  else if (!this->initGetValue())
    return -2;

  return FFaFunctionManager::getSmartPoints(myExplType,
                                            this->getExtrapolationType(),
                                            start,stop,this->getData(),x,y);
}


int FmMathFuncBase::getCurvePoints(double start, double stop, double inc,
				   DoubleVec& x, DoubleVec& y)
{
  size_t i, nvals = 0;
  this->getXAxisDomain(start,stop);
  if (stop > start && inc > 0.0)
    nvals = (size_t)floor((stop-start)/inc);

  bool updateXaxis = false;
  if (x.size() < 1)
    updateXaxis = true;
  else if (x[0] != start || x[x.size()-1] != stop)
    updateXaxis = true;
  else if (x[1]-x[0] != inc)
    updateXaxis = true;
  else if (nvals > 0)
    updateXaxis = nvals+1 != x.size();

  if (updateXaxis)
  {
    if (nvals > 0)
    {
      x.clear();
      x.reserve(nvals+1);
      for (i = 0; i <= nvals; i++)
	x.push_back(start + i*inc);
      if (x[nvals] < stop)
	x.push_back(stop);
    }
    else if (start == stop)
      x = { start };
    else
      return -1;
  }

  if (!this->initGetValue()) return -2;

  int ierr = 0;
  y.clear();
  y.reserve(x.size());
  for (i = 0; i < x.size() && !ierr; i++)
    y.push_back(this->getValue(x[i],ierr));

  return ierr;
}


const char* FmMathFuncBase::getFunctionFsiName() const
{
  return this->getTypeIDName()+3;
}


const char* FmMathFuncBase::getUITypeName() const
{
  switch (this->getFunctionUse())
    {
    case DRIVE_FILE:
      return "Time history input file";

    case ROAD_FUNCTION:
      return "Road elevation";

    case WAVE_FUNCTION:
      return "Wave function";
    case CURR_FUNCTION:
      return "Current function";

    case SPR_TRA_STIFF:
      return "K(d): Spring stiff. (trans. def.)";
    case SPR_TRA_FORCE:
      return "F(d): Spring force (trans. def.)";
    case SPR_ROT_STIFF:
      return "K(a): Spring stiff. (ang. def.)";
    case SPR_ROT_TORQUE:
      return "T(a): Spring torque (ang. def.)";

    case DA_TRA_COEFF:
      return "C(v): Damper coeff. (trans. vel.)";
    case DA_TRA_FORCE:
      return "F(v): Damper force (trans. vel.)";
    case DA_ROT_COEFF:
      return "C(w): Damper coeff. (ang. vel.)";
    case DA_ROT_TORQUE:
      return "T(w): Damper torque (ang. vel.)";

    default:
      return this->getFunctionUIName();
    }
}


/*!
  Re-implementation of FmBase::getInfoString to obtain
  compatible names in the Function query menus and the object browser.
*/

std::string FmMathFuncBase::getInfoString() const
{
  std::string infoString;
  switch (this->getFunctionUse())
  {
    case GENERAL:
    {
      // This function is a General Function (referred by Engine(s))
      std::vector<FmEngine*> engines;
      this->getEngines(engines);
      for (FmEngine* engine : engines)
        if (!engine->isFunctionLinked())
	{
	  // Use ID and UserDescription of the owner Engine instead,
	  // but replace the Engine type name with that of the function
	  infoString = engine->getInfoString();
	  size_t ipos = infoString.find_last_of('(')+1;
	  size_t jpos = infoString.find_last_of(')');
	  infoString.replace(ipos,jpos-ipos,this->getUITypeName());
	  return infoString;
	}
    }

    case DRIVE_FILE:
    case NONE:
      // Default info string: [ID] UserDescription (UITypeName)
      infoString = this->FmBase::getInfoString();
      break;

    default:
      // Road, wave, current, spring or damper function
      infoString = this->FmBase::getInfoString();
      // Append the actual function type name to the info string
      // since the getUITypeName method only returns the function use name
      infoString.insert(infoString.find_last_of(')'),
			std::string(", ") + this->getFunctionUIName());
  }

  return infoString;
}


FmCurveSet* FmMathFuncBase::getPreviewCurve() const
{
  std::vector<FmCurveSet*> referredCurves;
  this->getReferringObjs(referredCurves,"myFunction");
  for (FmCurveSet* curve : referredCurves)
    if (curve->usingInputMode() == FmCurveSet::PREVIEW_FUNC)
      return curve; // there should only be one

  return NULL;
}


bool FmMathFuncBase::setFunctionUsage(int usage)
{
  if (usage >= 0 && usage < DA_ROT_TORQUE)
    return this->setFunctionUse(static_cast<FuncUse>(usage));

  return false;
}


bool FmMathFuncBase::setFunctionUse(FuncUse newUse, bool checkUniqueID)
{
  if (myUse.getValue() == newUse) return false;

  myUse.setValue(newUse);

  if (!checkUniqueID || newUse <= DRIVE_FILE) return false;

  // Check all other functions with same use to find a unique ID of the
  // this function that doesn't conflict with existing functions (TT #2606).
  std::set<int> usedIDs;
  std::vector<FmMathFuncBase*> allFuncs;
  FmDB::getAllFunctions(allFuncs);
  for (FmMathFuncBase* f : allFuncs)
    if (f != this)
    {
      if (f->getTypeID() == this->getTypeID())
	usedIDs.insert(f->getID());
      else if (f->getFunctionUse() == newUse && f->getID() != this->getID())
        usedIDs.insert(f->getID());
    }

  if (this->getID() > 0)
    if (usedIDs.find(this->getID()) == usedIDs.end())
      return false; // The current user ID was unique so keep it

  int newID = 1; // We need to assign a new ID to this function
  while (usedIDs.find(newID) != usedIDs.end()) newID++;
  this->setID(newID);
  return true;
}


void FmMathFuncBase::onEventSwitched(const FmSimulationEvent*)
{
  std::vector<FmVesselMotion*> vms;
  this->getReferringObjs(vms,"waveFunction");
  for (FmVesselMotion* vm : vms)
    if (vm == vms.front())
      vm->onWaveChanged();
    else
      vm->onRAOChanged();
}


bool FmMathFuncBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmMathFuncBase::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_REPLACE)
    return true;

  FmMathFuncBase* copyObj = static_cast<FmMathFuncBase*>(obj);

  std::vector<FmEngine*> engines;
  copyObj->getEngines(engines);
  for (FmEngine* engine : engines)
    engine->setFunction(this);

  return true;
}


static std::vector<int> oldPreviewFunc;

bool FmMathFuncBase::localParse(const char* keyWord, std::istream& activeStatement,
				FmMathFuncBase* obj)
{
  if (strcmp(keyWord,"PREVIEW_FUNCTION") == 0)
  {
    // Obsolete field, present in R5.0 files and older
    int curveID;
    activeStatement >> curveID;
    oldPreviewFunc.push_back(curveID);
    return true;
  }

  return parentParse(keyWord, activeStatement, obj);
}


/*!
  Returns all engines referring to this function.
*/

void FmMathFuncBase::getEngines(std::vector<FmEngine*>& toFill) const
{
  toFill.clear();
  this->getReferringObjs(toFill);
}


/*!
  To be called after the model is read.
  Sorts functions according to new function/engine regime.
  If a road/spring/damper function is used in several different places,
  copies are created. Only functions with use type NONE are considered.
*/

void FmMathFuncBase::resolveAfterRead()
{
  std::vector<FmMathFuncBase*> funcs;
  FmDB::getAllFunctions(funcs);

  // Loop over all functions
  for (FmMathFuncBase* f : funcs)
  {
    if (!f->getParentAssembly())
    {
      // Set parent assembly from the owner engine and reconnect.
      // To fixup pre R7.1.3 models where functions might be on top level only.
      std::vector<FmEngine*> engines;
      f->getReferringObjs(engines);
      for (FmEngine* e : engines)
        if (e->getParentAssembly())
        {
          f->disconnect();
          f->setParentAssembly(e->getParentAssembly());
          f->connect();
          break;
        }
    }

    // Only doing this for functions with NONE flag
    if (f->getFunctionUse() != NONE)
      continue;

    bool newID = false;

    // Get all objects that are using this function
    std::multimap<std::string,FFaFieldContainer*> refs;
    std::multimap<std::string,FFaFieldContainer*>::const_iterator it;
    f->getReferringObjs(refs);

    std::vector<FmSpringBase*> springs;
    std::vector<FmDamperBase*> dampers;
    std::vector<FmEngine*>     engines;
    std::vector<FmRoad*>       roads;
    for (it = refs.begin(); it != refs.end(); it++) {
      FmBase* refObj = (FmBase*)it->second;
      if (refObj->isOfType(FmSpringBase::getClassTypeID()))
	springs.push_back((FmSpringBase*)refObj);
      else if (refObj->isOfType(FmDamperBase::getClassTypeID()))
	dampers.push_back((FmDamperBase*)refObj);
      else if (refObj->isOfType(FmEngine::getClassTypeID()))
	engines.push_back((FmEngine*)refObj);
      else if (refObj->isOfType(FmRoad::getClassTypeID()))
	roads.push_back((FmRoad*)refObj);
    }

    // Engines

    if (!engines.empty())
      f->setFunctionUse(GENERAL);

    // Roads

    if (!roads.empty())
    {
      if (f->getFunctionUse() == NONE)
	newID = f->setFunctionUse(ROAD_FUNCTION,true);
      else
      {
	FmMathFuncBase* newF = f->copy();
	newF->setFunctionUse(ROAD_FUNCTION);
	newF->connect();
	for (FmRoad* road : roads)
	  road->roadFunction = newF;
      }
    }

    // Springs

    std::map<FuncUse,FmMathFuncBase*> createMap;
    for (FmSpringBase* spring : springs)
    {
      bool isT = spring->getDOF() < 3 ? true : false;
      bool isF = spring->isForceFuncFromFile().second;

      FuncUse ft;
      if (isT)
	ft = isF ? SPR_TRA_FORCE  : SPR_TRA_STIFF;
      else
	ft = isF ? SPR_ROT_TORQUE : SPR_ROT_STIFF;

      // If current function is still NONE, change use type to <ft> and proceed
      // If the function has a different use type than this spring should have,
      // assign a copy of the function with use type <ft> to the spring
      if (f->getFunctionUse() == NONE)
	newID = f->setFunctionUse(ft,true);

      else if (f->getFunctionUse() != ft) {
	std::map<FuncUse,FmMathFuncBase*>::const_iterator fit = createMap.find(ft);
	if (fit == createMap.end()) {
	  // No copy of this type exist yet, create one
	  FmMathFuncBase* newF = f->copy();
	  newF->setFunctionUse(ft);
	  newF->connect();
	  fit = createMap.insert(std::make_pair(ft,newF)).first;
	}
	spring->setSpringCharOrStiffFunction(fit->second);
      }
    }

    // Dampers

    createMap.clear();
    for (FmDamperBase* damper : dampers)
    {
      bool isT = damper->getDOF() < 3 ? true : false;
      bool isF = damper->isForceFuncFromFile().second;

      FuncUse ft;
      if (isT)
	ft = isF ? DA_TRA_FORCE  : DA_TRA_COEFF;
      else
	ft = isF ? DA_ROT_TORQUE : DA_ROT_COEFF;

      // If current function is still NONE, change use type to <ft> and proceed
      // If the function has a different use type than this damper should have,
      // assign a copy of the function with use type <ft> to the damper
      if (f->getFunctionUse() == NONE)
	newID = f->setFunctionUse(ft,true);

      else if (f->getFunctionUse() != ft) {
	std::map<FuncUse,FmMathFuncBase*>::const_iterator fit = createMap.find(ft);
	if (fit == createMap.end()) {
	  // No copy of this type exist yet, create one
	  FmMathFuncBase* newF = f->copy();
	  newF->setFunctionUse(ft);
	  newF->connect();
	  fit = createMap.insert(std::make_pair(ft,newF)).first;
	}
	damper->setFunction(fit->second);
      }
    }

    // If no usage is detected, touch the function, to make it appear in the
    // objects browser. We also need to reconnect if it was assigned a new ID.

    if (f->getFunctionUse() == NONE || newID) {
      f->disconnect();
      f->connect();
    }
  }

  // Correct the curve type of the old function preview curves, if any
  for (int oldID : oldPreviewFunc)
  {
    FmBase* found = FmDB::findID(FmCurveSet::getClassTypeID(),oldID);
    if (found) {
      FmCurveSet* curve = static_cast<FmCurveSet*>(found);
      if (curve->usingInputMode() == FmCurveSet::INT_FUNCTION)
	curve->useInputMode(FmCurveSet::PREVIEW_FUNC);
    }
  }

  oldPreviewFunc.clear();
}


/*!
  Re-implementation of FmSimulationModelBase::isListable to filter out
  functions used in engines (general functions).
*/

bool FmMathFuncBase::isListable() const
{
  return this->getFunctionUse() != GENERAL;
}


bool FmMathFuncBase::isLegalSprDmpFunc() const
{
  for (int legalType : FmFuncAdmin::getAllowableSprDmpFuncTypes())
    if (this->isOfType(legalType))
      return true;

  return false;
}


bool FmMathFuncBase::hasSmartPoints() const
{
  return FmFuncAdmin::hasSmartPoints(this->getTypeID());
}


int FmMathFuncBase::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&FUNCTION\n");
  this->printID(fp);
  fprintf(fp,"  type = '%s'\n",this->getFunctionFsiName());
  int extrap = this->getExtrapolationType();
  if (extrap)
    fprintf(fp,"  extrapolationType = %d\n", extrap);
  int err = this->printSolverData(fp);
  fprintf(fp,"/\n\n");
  return err;
}
