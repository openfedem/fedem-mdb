// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/Icons/FmIconPixmaps.H"
#include "vpmDB/FmMathFuncBase.H"
#include "vpmDB/FmSubAssembly.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmFuncTree.H"


FmFuncStart::FmFuncStart(const std::string& UIString, const char** pixmap, int fUse)
  : FmRingStart(UIString,pixmap)
{
  myFuncUse = fUse;
  myRingMemberType = FmMathFuncBase::getClassTypeID();
}


bool FmFuncStart::hasRingMembers() const
{
  std::vector<FmMathFuncBase*> allFuncs;
  FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(this->getParentAssembly());
  FmDB::getAllFunctions(allFuncs,subAss,true);

  for (size_t i = 0; i < allFuncs.size(); i++)
    if (allFuncs[i]->getFunctionUse() == myFuncUse)
      return true;

  return false;
}


void FmFuncStart::getModelMembers(std::vector<FmModelMemberBase*>& list)
{
  std::vector<FmMathFuncBase*> allFuncs;
  FmSubAssembly* subAss = dynamic_cast<FmSubAssembly*>(this->getParentAssembly());
  FmDB::getAllFunctions(allFuncs,subAss,true);

  for (size_t i = 0; i < allFuncs.size(); i++)
    if (allFuncs[i]->getFunctionUse() == myFuncUse)
      list.push_back(allFuncs[i]);
}


FmRingStart* FmFuncStart::searchFuncHead(int funcUse) const
{
  return myFuncUse == funcUse ? const_cast<FmFuncStart*>(this) : NULL;
}


FmFuncTree::FmFuncTree(FmRingStart* root) : myNodes(15,NULL)
{
  myHead = new FmRingStart("Function definitions", function_xpm);

  myNodes[0]  = new FmFuncStart("Time history input files",
				timeHistInputFile_xpm,
				FmMathFuncBase::DRIVE_FILE);
  myNodes[1]  = new FmFuncStart("Road elevations",
				createRoad_xpm,
				FmMathFuncBase::ROAD_FUNCTION);
  myNodes[2]  = new FmFuncStart("Wave functions",
				f_of_xt_xpm,
				FmMathFuncBase::WAVE_FUNCTION);
  myNodes[3]  = new FmFuncStart("Current functions",
				f_of_xt_xpm,
				FmMathFuncBase::CURR_FUNCTION);
  myNodes[4]  = new FmFuncStart("Unconverted functions (not used)",
				f_of_x_xpm,
				FmMathFuncBase::NONE);

  myNodes[5]  = new FmRingStart("Spring characteristics", spring_xpm);
  myNodes[6]  = new FmRingStart("Damper characteristics", damper_xpm);

  myNodes[7]  = new FmFuncStart("Stiffness - Translation",
				K_Tspring_xpm,
				FmMathFuncBase::SPR_TRA_STIFF);
  myNodes[8]  = new FmFuncStart("Force - Translation",
				F_Tspring_xpm,
				FmMathFuncBase::SPR_TRA_FORCE);
  myNodes[9]  = new FmFuncStart("Stiffness - Rotation",
				K_Rspring_xpm,
				FmMathFuncBase::SPR_ROT_STIFF);
  myNodes[10] = new FmFuncStart("Torque - Rotation",
				T_Rspring_xpm,
				FmMathFuncBase::SPR_ROT_TORQUE);
  myNodes[11] = new FmFuncStart("Coefficient - Translational vel.",
				C_Tdamper_xpm,
				FmMathFuncBase::DA_TRA_COEFF);
  myNodes[12] = new FmFuncStart("Force - Translational vel.",
				F_Tdamper_xpm,
				FmMathFuncBase::DA_TRA_FORCE);
  myNodes[13] = new FmFuncStart("Coefficient - Rotational vel.",
				C_Rdamper_xpm,
				FmMathFuncBase::DA_ROT_COEFF);
  myNodes[14] = new FmFuncStart("Torque - Rotational vel.",
				T_Rdamper_xpm,
				FmMathFuncBase::DA_ROT_TORQUE);

  myNodes[5]->setRingMemberType(FmMathFuncBase::getClassTypeID());
  myNodes[6]->setRingMemberType(FmMathFuncBase::getClassTypeID());

  int i;
  for (i = 0; i < 7; i++)
    myNodes[i]->setParent(root);
  for (i = 7; i < 11; i++)
    myNodes[i]->setParent(myNodes[5]);
  for (i = 11; i < 15; i++)
    myNodes[i]->setParent(myNodes[6]);
}


FmFuncTree::~FmFuncTree()
{
  for (size_t i = 0; i < myNodes.size(); i++)
    myNodes[i]->erase();

  myHead->erase();
}


void FmFuncTree::setParentAssembly(FmBase* subAss)
{
  for (size_t i = 0; i < myNodes.size(); i++)
    myNodes[i]->setParentAssembly(subAss);
}
