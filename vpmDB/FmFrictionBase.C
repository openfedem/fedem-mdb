// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmFrictionBase.H"
#include "FFaLib/FFaString/FFaStringExt.H"


Fmd_DB_SOURCE_INIT(FcFRICTION_BASE, FmFrictionBase, FmParamObjectBase);


FmFrictionBase::FmFrictionBase() : FmParamObjectBase()
{
  Fmd_CONSTRUCTOR_INIT(FmFrictionBase);

  FFA_FIELD_INIT(myPrestressLoad , 0.0, "PRESTRESS_LOAD");
  FFA_FIELD_INIT(myCoulombCoeff  , 0.0, "COULOMB_COEFF");
  FFA_FIELD_INIT(myStribeckMagn  , 0.0, "STRIBECK_MAGN");
  FFA_FIELD_INIT(myStribeckSpeed , 0.0, "STRIBECK_SPEED");

/* Removed for usability, not in use since R2.1.2
  FFA_FIELD_INIT(myViscCoeff     , 0.0, "VISC_COEFF");
  FFA_FIELD_INIT(myAsymmetrMagn  , 0.0, "ASYMMETRIC_MAGN");
  FFA_FIELD_INIT(myFricAmpl      , 0.0, "FRIC_AMPL");
  FFA_FIELD_INIT(myFreq          , 0.0, "FREQUENCY");
  FFA_FIELD_INIT(myPhaseAngle    , 0.0, "PHASE_ANGLE");
*/
}


FmFrictionBase::~FmFrictionBase()
{
  this->disconnect();
}


double FmFrictionBase::getStickStiffness() const
{
  // Beta feature: friction spring
  return FFaString(this->getUserDescription()).getDoubleAfter("#Kstick");
}


void FmFrictionBase::getParameters(std::vector<FmParameter>& retArray) const
{
  M_APPEND_PARAMS("Coulomb coefficient",CoulombCoeff,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Magnitude of Stribeck effect, S",StribeckMagn,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Critical Stribeck speed, Vslip",StribeckSpeed,FmFrictionBase,retArray);

  /* Removed for usability
  M_APPEND_PARAMS("Viscous friction coefficient",ViscCoeff,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Magnitude of asymmetries, %",AsymmetrMagn,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Friction amplitude",FricAmpl,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Frequency",Freq,FmFrictionBase,retArray);
  M_APPEND_PARAMS("Phase Angle",PhaseAngle,FmFrictionBase,retArray);
  */
}


int FmFrictionBase::printSolverEntry(FILE* fp)
{
  fprintf(fp,"&FRICTION_SET\n");
  this->printID(fp);
  fprintf(fp,"  type = '%s'\n", this->getFrictionFsiName());

  std::vector<double> vars;
  this->getTypeDepVars(vars);
  if (!vars.empty())
  {
    fprintf(fp,"  typeDepParams =");
    for (size_t i = 0; i < vars.size(); i++)
      fprintf(fp," %14.6e", vars[i]);
    fprintf(fp,"\n");
  }

  fprintf(fp,"  PrestressLoad = %14.6e\n", myPrestressLoad.getValue());
  fprintf(fp,"  CoulombCoeff  = %14.6e\n", myCoulombCoeff.getValue());
  fprintf(fp,"  StribeckMagn  = %14.6e\n", myStribeckMagn.getValue());
  fprintf(fp,"  StribeckSpeed = %14.6e\n", myStribeckSpeed.getValue());

/* Removed for usability, not in use since R2.1.2
  fprintf(fp,"  ViscCoeff     = %14.6e\n", myViscCoeff.getValue());
  fprintf(fp,"  AsymMagn      = %14.6e\n", myAsymmetrMagn.getValue());
  fprintf(fp,"  FricAmpl      = %14.6e\n", myFricAmpl.getValue());
  fprintf(fp,"  FricFreq      = %14.6e\n", myFreq.getValue());
  fprintf(fp,"  FricPhase     = %14.6e\n", myPhaseAngle.getValue());
*/

  // Beta feature: friction spring
  FFaString fDesc = this->getUserDescription();
  if (fDesc.hasSubString("#Kstick"))
    fprintf(fp,"  StickStiffness = %14.6e\n", fDesc.getDoubleAfter("#Kstick"));

  fprintf(fp,"/\n\n");
  return 0;
}


bool FmFrictionBase::localParse(const char* keyWord, std::istream& activeStatement,
				FmFrictionBase* obj)
{
  return parentParse(keyWord, activeStatement, obj);
}


bool FmFrictionBase::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmFrictionBase::getClassTypeID()))
    return false;

  if (depth == FmBase::DEEP_REPLACE)
    obj->releaseReferencesToMe("myFriction",this);

  return true;
}
