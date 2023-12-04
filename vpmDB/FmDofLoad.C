// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmDofLoad.H"
#include "vpmDB/FmMMJointBase.H"
#include "vpmDB/FmfDeviceFunction.H"
#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaParse.H"
#include "FFaLib/FFaString/FFaStringExt.H"


Fmd_DB_SOURCE_INIT(FcDOF_LOAD, FmDofLoad, FmIsControlledBase);


FmDofLoad::FmDofLoad()
{
  Fmd_CONSTRUCTOR_INIT(FmDofLoad);

  FFA_FIELD_INIT(myLoadVal, 0.0, "VALUE");
  FFA_FIELD_INIT(freqDomain, false, "FREQUENCY_DOMAIN");
}


FmDofLoad::~FmDofLoad()
{
  this->disconnect();
}


bool FmDofLoad::disconnect()
{
  this->mainDisconnect();

  FmHasDOFsBase* owner = this->getOwner();
  if (!owner) return true;

  owner->releaseLoadAtDOF(owner->atWhatDOF(this));
  return true;
}


FmModelMemberBase* FmDofLoad::getActiveOwner() const
{
  FmHasDOFsBase* owner = this->getOwner();
  if (owner)
  {
    int dof = owner->atWhatDOF(this);
    if (!owner->isLegalDOF(dof))
      return NULL;

    FmHasDOFsBase::DOFStatus dstat = owner->getStatusOfDOF(dof);
    if (dstat == FmHasDOFsBase::FIXED || dstat == FmHasDOFsBase::PRESCRIBED)
      return NULL;

    if (!this->getEngine() && fabs(this->getInitLoad()) < 1.0e-30)
      return NULL;
  }

  return owner;
}


FmHasDOFsBase* FmDofLoad::getOwner() const
{
  FmHasDOFsBase* owner = NULL;
  if (this->hasReferringObjs(owner)) // there should only be one owner
    return owner;
  else
    return NULL;
}


std::ostream& FmDofLoad::writeFMF(std::ostream& os)
{
  os <<"DOF_LOAD\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


static bool localParse(const char* keyWord, std::istream& is, FmDofLoad* obj)
{
  // Conversion of old keywords
  if (strcmp(keyWord,"INIT_LOAD") == 0)
    return FmDofLoad::parentParse("VALUE",is,obj);

  return FmDofLoad::parentParse(keyWord,is,obj);
}

bool FmDofLoad::readAndConnect(std::istream& is, std::ostream&)
{
  FmDofLoad* obj = new FmDofLoad();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord,is,activeStatement,'=',';'))
      ::localParse(keyWord,activeStatement,obj);
  }

  obj->connect();
  return true;
}


bool FmDofLoad::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmDofLoad::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmDofLoad::getClassTypeID());
}


int FmDofLoad::printSolverEntry(FILE* fp)
{
  if (!this->getActiveOwner())
    return 0; // DOF has no load

  FmHasDOFsBase* owner = this->getOwner();
  if (owner->isSuppressed())
    return 0; // Owner is suppressed, ignore load

  fprintf(fp,"&LOAD\n");
  this->printID(fp);

  int lDof = 1 + owner->atWhatDOF(this);
  if (lDof == 3 && owner->isOfType(FmMMJointBase::getClassTypeID()))
    lDof = 7; // Slider dof

  double f0 = 0.0; // Constant load part
  int loadEngine = 0;
  if (this->getEngine())
    loadEngine = this->getEngine()->getBaseID();
  else
    f0 = this->getInitLoad();

  // Beta feature: Update external forces based on previous configuration
  FFaString lDesc = this->getUserDescription();
  int updateFlag = lDesc.hasSubString("#PrevStep") ? 1 : 2;
  if (lDesc.hasSubString("#LocalAx")) updateFlag += 10;

  if (owner->isOfType(FmJointBase::getClassTypeID()))
    fprintf(fp,"  jointId = %d\n", owner->getBaseID());
  else
    fprintf(fp,"  triadId = %d\n", owner->getBaseID());
  if (updateFlag != 2)
    fprintf(fp,"  updateFlag = %d\n", updateFlag);
  fprintf(fp,"  lDof = %d\n", lDof);
  if (loadEngine < 1)
    fprintf(fp,"  f0 = %17.9e\n", f0);
  else if (freqDomain.getValue())
  {
    fprintf(fp,"  loadType = 1\n");
    FmfDeviceFunction* f = dynamic_cast<FmfDeviceFunction*>(this->getEngine()->getFunction());
    if (f)
    {
      std::string fileName(f->getActualDeviceName(false));
      if (FFaFilePath::isRelativePath(fileName) && !relPathCorrection.empty())
        fileName = relPathCorrection + fileName;
      fprintf(fp,"  fileName = '%s'\n", fileName.c_str());
    }
    else
      fprintf(fp,"  f1 = 1.0, loadEngineId = %d\n", loadEngine);
  }
  else
    fprintf(fp,"  f1 = 1.0, loadEngineId = %d\n", loadEngine);

  // Variables to be saved:
  // 1 - Global force vector
  // 2 - Signed force amplitude
  // 3 - Energies
  if (!freqDomain.getValue())
    this->writeSaveVar(fp,3);

  fprintf(fp,"/\n\n");
  return 0;
}
