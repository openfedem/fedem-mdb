// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdUserDefinedElement.H"
#endif

#include "vpmDB/FmUserDefinedElement.H"
#include "vpmDB/FmAssemblyBase.H"
#include "vpmDB/FmTriad.H"
#include "vpmDB/FmDB.H"

#include "FiUserElmPlugin/FiUserElmPlugin.H"

#include <algorithm>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FcUserDefinedElement, FmUserDefinedElement, FmLink);


FmUserDefinedElement::FmUserDefinedElement()
{
  Fmd_CONSTRUCTOR_INIT(FmUserDefinedElement);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdUserDefinedElement(this);
#endif

  // Remove irrelevant fields inherited from FmIsPositionedBase.
  // These fields will then be ignored on read, write and copy.

  this->removeField("COORDINATE_SYSTEM");
  this->removeField("LOCATION3D_DATA");
  this->removeField("LOCATION3D_POS_VIEW_REF");
  this->removeField("LOCATION3D_ROT_VIEW_REF");

  // Initialize fields

  FFA_REFERENCELIST_FIELD_INIT(myTriadsField, myTriads, "TRIADS");

  FFA_FIELD_INIT(myType, 0, "ELEMENT_TYPE");
  FFA_FIELD_DEFAULT_INIT(myName, "ELEMENT_NAME");

  iwork = NULL;
  rwork = NULL;
}


void FmUserDefinedElement::init(int eType, const char* typeName,
                                const std::vector<FmTriad*>& triads)
{
  // Check number of nodal DOFs and update the triads, if no other connections
  int ni = 0, nr = 0;
  int ndof = FiUserElmPlugin::instance()->init(this->getBaseID(),eType,0,0,ni,nr);
  for (FmTriad* triad : triads)
    if (ndof > 0 && !triad->hasElementBinding() && !triad->hasJointBinding())
      triad->setNDOFs(ndof);

  myType.setValue(eType);
  myName.setValue(typeName);
  myTriads.setPtrs(triads);
}


void FmUserDefinedElement::clearWork()
{
  delete[] iwork;
  delete[] rwork;
  iwork = NULL;
  rwork = NULL;
}


int FmUserDefinedElement::initWork(int nenod, int nedof,
                                   const double* X, const double* T) const
{
  if (iwork || rwork)
    return 0;

  int niwork = 0, nrwork = 0;
  int ierr = FiUserElmPlugin::instance()->init(this->getBaseID(),myType.getValue(),
                                               nenod,nedof,niwork,nrwork);
  if (ierr < 0)
  {
    std::cerr <<"Error: Invalid user-defined element "<< this->getIdString()
              <<" (type="<< myType.getValue() <<")"<< std::endl;
    return ierr;
  }

  iwork = new int[niwork];
  rwork = new double[nrwork];
  if (niwork > 0) memset(iwork,0,niwork*sizeof(int));
  if (nrwork > 0) memset(rwork,0,nrwork*sizeof(double));

  // Initialize the work arrays with property data in exactly the same way
  // as in the solver subroutine initiateUserdefElTypeModule::ReadUserdefEls
  FFaString uDesc(this->getUserDescription());
  if (niwork > 0) iwork[0] = niwork;
  if (niwork > 1) iwork[1] = nrwork;
  if (niwork > 2) uDesc.getIntsAfter("#Params",niwork-2,iwork+2);
  if (nrwork > 0) rwork[0] = alpha1.getValue();
  if (nrwork > 1) rwork[1] = alpha2.getValue();
  if (nrwork > 2) rwork[2] = stiffnessScale.getValue();
  if (nrwork > 3) rwork[3] = massScale.getValue();
  if (nrwork > 4) uDesc.getDoublesAfter("#Property",nrwork-4,rwork+4);

  ierr = FiUserElmPlugin::instance()->init(this->getBaseID(),myType.getValue(),
                                           nenod,nedof,X,T,iwork,rwork);
  if (ierr < 0)
    std::cerr <<"Error: Failed to initialize user-defined element "
              << this->getIdString() << std::endl;
  return ierr;
}


FmTriad* FmUserDefinedElement::findTriad(int baseID) const
{
  for (size_t i = 0; i < myTriads.size(); i++)
    if (!myTriads[i].isNull())
      if (myTriads[i]->getBaseID() == baseID)
        return myTriads.getPtr(i);

  return NULL;
}


inline bool TriadIdLess(const FmTriad* lhs, const FmTriad* rhs)
{
  return lhs->getID() < rhs->getID(); // Sorting w.r.t. triad IDs
}

void FmUserDefinedElement::getTriads(std::vector<FmTriad*>& tr, bool sortOnId) const
{
  myTriads.getPtrs(tr);
  if (sortOnId && tr.size() > 1)
    std::sort(tr.begin(),tr.end(),TriadIdLess);
}


const FaMat34& FmUserDefinedElement::getLocalCS() const
{
  FaMat34& itsCS = const_cast<FaMat34&>(myCS.getValue());
  FmAssemblyBase* parent = this->getPositionedAssembly();
  if (parent)
    itsCS = parent->toLocal(this->getGlobalCS());
  else
    itsCS = this->getGlobalCS();

  return myCS.getValue();
}


FaMat34 FmUserDefinedElement::getGlobalCS() const
{
  int nenod = myTriads.size();
  int nedof = 0;

  std::vector<double> X(3*nenod);
  std::vector<double> T(9*nenod);
  for (int i = 0; i < nenod; i++)
  {
    nedof += myTriads[i]->getNDOFs();
    FaMat34 triadCS = myTriads[i]->getGlobalCS();
    for (int j = 0; j < 4; j++)
      std::copy(triadCS[j].getPt(), triadCS[j].getPt()+3,
                j < 3 ? T.begin()+9*i+3*j : X.begin()+3*i);
  }

  if (this->initWork(nenod, nedof, X.data(), T.data()) < 0)
    return FaMat34();

  double Tlg[12];
  if (FiUserElmPlugin::instance()->origin(this->getBaseID(), myType.getValue(),
                                          nenod, X.data(), T.data(),
                                          iwork, rwork, Tlg) >= 0)
    return FaMat34(Tlg);

  std::cerr <<"Error: Failed to get coordinate system "
            <<"for user-defined element "<< this->getIdString() << std::endl;
  return FaMat34();
}


FaVec3 FmUserDefinedElement::getTranslation() const
{
  FaVec3 pos = this->getGlobalCS().translation();

  FmAssemblyBase* parent = this->getPositionedAssembly();
  return parent ? parent->toLocal(pos) : pos;
}


FaMat33 FmUserDefinedElement::getOrientation() const
{
  FaMat33 pos = this->getGlobalCS().direction();

  FmAssemblyBase* parent = this->getPositionedAssembly();
  return parent ? parent->toLocal(pos) : pos;
}


FaMat34 FmUserDefinedElement::getPositionCG(bool globalCS) const
{
  FaVec3 Xcg;
  for (size_t i = 0; i < myTriads.size(); i++)
    Xcg += myTriads[i]->getGlobalTranslation();
  Xcg *= 1.0/(double)myTriads.size();

  return globalCS ? FaMat34(Xcg) : FaMat34(this->getGlobalCS().inverse()*Xcg);
}


double FmUserDefinedElement::getMass() const
{
  int nenod = myTriads.size();
  int nedof = 0;

  std::vector<double> X(3*nenod);
  for (int i = 0; i < nenod; i++)
  {
    nedof += myTriads[i]->getNDOFs();
    FaMat34 triadCS = myTriads[i]->getGlobalCS();
    std::copy(triadCS[3].getPt(), triadCS[3].getPt()+3, X.begin()+3*i);
  }

  if (this->initWork(nenod, nedof, X.data()) < 0)
    return 0.0;

  double mass = 0.0;
  if (FiUserElmPlugin::instance()->mass(this->getBaseID(), myType.getValue(),
                                        nenod, X.data(), iwork, rwork, mass) < 0)
    std::cerr <<"Error: Failed to get mass for user-defined element "
              << this->getIdString() << std::endl;

  return mass;
}


bool FmUserDefinedElement::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj,depth);
}


bool FmUserDefinedElement::cloneLocal(FmBase* obj, int depth)
{
  if (!obj->isOfType(FmUserDefinedElement::getClassTypeID()))
    return false;
  else if (depth < FmBase::DEEP_APPEND)
    return true;

  FmUserDefinedElement* copyObj = static_cast<FmUserDefinedElement*>(obj);
  std::vector<FmTriad*> triads;
  copyObj->getTriads(triads);
  this->myTriads.setPtrs(triads);
  if (depth == FmBase::DEEP_REPLACE)
    copyObj->myTriads.clear();

  return true;
}


/***********************************************************************
*
* Input and output from stream.
*
************************************************************************/

std::ostream& FmUserDefinedElement::writeFMF(std::ostream& os)
{
  os <<"USER_DEFINED_ELEMENT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmUserDefinedElement::readAndConnect(std::istream& is, std::ostream&)
{
  FmUserDefinedElement* obj = new FmUserDefinedElement();

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


bool FmUserDefinedElement::writeToVTF(VTFAFile&, IntVec*, IntVec*)
{
  return true; // Silently ignore all user-defined elements for now
}


int FmUserDefinedElement::printSolverEntry(FILE* fp)
{
  fprintf(fp,"' %s\n",myName.getValue().c_str());
  fprintf(fp,"&USER_EL\n");
  this->printID(fp);

  fprintf(fp,"  eType = %d\n", myType.getValue());
  fprintf(fp,"  numTriads = %u\n", (unsigned int)myTriads.size());
  fprintf(fp,"  triadIDs =");
  for (size_t i = 0; i < myTriads.size(); i++)
    fprintf(fp," %d", myTriads[i]->getBaseID());

  // Scaling of dynamic properties
  if (stiffnessScale.getValue() != 1.0)
    fprintf(fp,"  stiffScale =%17.9e\n", stiffnessScale.getValue());
  if (massScale.getValue() != 1.0)
    fprintf(fp,"  massScale  =%17.9e\n", massScale.getValue());

  // Structural damping coefficients
  fprintf(fp,"\n  alpha1 =%17.9e, alpha2 =%17.9e\n",
          alpha1.getValue(), alpha2.getValue());

  // Beta feature: Extract user-defined properties from the description string
  FFaString uDesc(this->getUserDescription());
  int ipar[100]; double rpar[100];
  int idVar[10];
  int nipar = uDesc.getIntsAfter("#Params",100,ipar);
  int nrpar = uDesc.getDoublesAfter("#Property",100,rpar);
  int nvar  = uDesc.getIntsAfter("#Engine",10,idVar);
  fprintf(fp,"  nipar = %d, nrpar = %d", nipar, nrpar);
  if (nipar > 0)
  {
    fprintf(fp,"\n  ipar =");
    for (int i = 0; i < nipar; i++) fprintf(fp," %d", ipar[i]);
  }
  if (nrpar > 0)
  {
    fprintf(fp,"\n  rpar =");
    for (int i = 0; i < nrpar; i++) fprintf(fp,"%17.9e", rpar[i]);
  }
  if (nvar > 0)
  {
    fprintf(fp,"  nvar = %d\n  idVar =", nvar);
    for (int i = 0; i < nvar; i++) {
      fprintf(fp," %d", idVar[i]);
      FmEngine::betaFeatureEngines.insert(idVar[i]);
    }
  }

  // Beta feature: Extract hydrodynamic properties from the description string
  nrpar = uDesc.getDoublesAfter("#Morison",10,rpar);
  if (nrpar > 0)
  {
    fprintf(fp,"\n  morison =");
    for (int i = 0; i < nrpar; i++) fprintf(fp,"%17.9e", rpar[i]);
  }
  fprintf(fp,"\n/\n\n");

  return myTriads.empty() ? 1 : 0;
}
