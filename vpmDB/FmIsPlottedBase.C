// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmCurveSet.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include <algorithm>


Fmd_DB_SOURCE_INIT(FcIS_PLOTTED_BASE, FmIsPlottedBase, FmIsRenderedBase);


FmIsPlottedBase::FmIsPlottedBase()
{
  Fmd_CONSTRUCTOR_INIT(FmIsPlottedBase);

  FFA_FIELD_DEFAULT_INIT(mySaveVar, "SAVE_VAR");
}


bool FmIsPlottedBase::hasCurveSets() const
{
  FmCurveSet* curve = NULL;
  return (this->hasReferringObjs(curve,"myResultObject[YAXIS]") ||
	  this->hasReferringObjs(curve,"myResultObject[XAXIS]"));
}

void FmIsPlottedBase::getCurveSets(std::vector<FmCurveSet*>& curves) const
{
  curves.clear();
  this->getReferringObjs(curves,"myResultObject[YAXIS]");

  // Avoid adding a curve twice if this object is referred by both axes
  std::vector<FmCurveSet*> xcurves;
  this->getReferringObjs(xcurves,"myResultObject[XAXIS]");
  std::vector<FmCurveSet*>::reverse_iterator rit;
  for (rit = xcurves.rbegin(); rit != xcurves.rend(); rit++)
    if (std::find(curves.begin(),curves.end(),*rit) == curves.end())
      curves.insert(curves.begin(),*rit);
}


bool FmIsPlottedBase::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmIsPlottedBase::getClassTypeID());
}


bool FmIsPlottedBase::eraseOptions()
{
  if (FmModelMemberBase::inInteractiveErase && this->hasCurveSets())
  {
    std::vector<FmCurveSet*> curves;
    this->getCurveSets(curves);

    std::string message = "Deleting " + this->getIdString(true)
      + ".\nThis object is used in the axis definition of\n";
    for (const FmCurveSet* c : curves) message += c->getIdString(true) + "\n";
    message += "Do you want to delete these axis definitions too ?";

    if (FFaMsg::dialog(message,FFaMsg::YES_ALL_NO))
      for (FmCurveSet* curve : curves)
        for (int a = 0; a < FmCurveSet::NAXES; a++)
          if (curve->getResultObj(a) == this)
          {
            curve->clearResult(a);
            curve->onDataChanged();
          }
  }

  return FmIsRenderedBase::eraseOptions();
}


bool FmIsPlottedBase::getSaveVar(unsigned int& nVar, IntVec& toggles) const
{
  if (mySaveVar.getValue().size() < nVar)
    nVar = mySaveVar.getValue().size();
  for (unsigned int i = 0; i < nVar; i++)
    toggles[i] = mySaveVar.getValue()[i] ? 1 : 0;

  return true;
}


void FmIsPlottedBase::writeSaveVar(FILE* fp, unsigned int nVar) const
{
  unsigned int nS = nVar;
  IntVec toggles(nVar,0);
  if (!this->getSaveVar(nVar,toggles))
    return;

  FFaString aDesc = this->getUserDescription();
  if (aDesc.hasSubString("#saveVar"))
  {
    // Beta feature: Toggling saving of individual variables on/off
    if (nVar == 0)
      nVar = aDesc.getIntsAfter("#saveVar",nS,toggles.data());
    else
      ListUI <<"---> WARNING: Ignoring #saveVar in the description field"
             <<" for "<< this->getIdString()
             <<".\n     Using the toggles in the property panel instead.\n";
  }

  // Write to file only if toggles content consist of ones
  if (std::find(toggles.begin(),toggles.begin()+nVar,1) != toggles.begin()+nVar)
  {
    fprintf(fp,"  saveVar =");
    for (unsigned int i = 0; i < nVar; i++)
      fprintf(fp," %d",toggles[i]);
    fprintf(fp,"\n");
  }
}
