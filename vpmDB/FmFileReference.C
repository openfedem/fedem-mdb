// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmGraph.H"
#include "vpmDB/FmFileReference.H"

std::vector<FmFileRefExt> FmFileReference::extensions;

Fmd_DB_SOURCE_INIT(FcFILE_REFERENCE, FmFileReference, FmSimulationModelBase);


FmFileReference::FmFileReference()
{
  Fmd_CONSTRUCTOR_INIT(FmFileReference);

  FFA_FIELD_DEFAULT_INIT(fileName,"FILE_NAME");
}


FmFileReference::~FmFileReference()
{
  this->disconnect();
}


/*!
  Returns the type menu items to be used in the file browser.
*/

const std::vector<FmFileRefExt>& FmFileReference::getExtensions()
{
  if (extensions.empty())
  {
    extensions.push_back(FmFileRefExt("ASCII file",FmGraph::asc()));
    extensions.push_back(FmFileRefExt("nCode DAC file",FmGraph::dac()));
    extensions.push_back(FmFileRefExt("MTS RPC Time history file",FmGraph::rpc()));
    extensions.push_back(FmFileRefExt("TNO Tire file",{"tpf"}));
    extensions.push_back(FmFileRefExt("Tire property file",{"tir"}));
    extensions.push_back(FmFileRefExt("Road property file",{"rdf"}));
  }

  return extensions;
}


std::ostream& FmFileReference::writeFMF(std::ostream& os)
{
  os <<"FILE_REFERENCE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmFileReference::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmFileReference::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmFileReference::getClassTypeID());
}


bool FmFileReference::readAndConnect(std::istream& is, std::ostream&)
{
  FmFileReference* obj = new FmFileReference();

  while (is.good())
  {
    std::stringstream activeStatement;
    char keyWord[BUFSIZ];
    if (FaParse::parseFMFASCII(keyWord, is, activeStatement, '=', ';'))
      parentParse(keyWord, activeStatement, obj);
  }

  FFaFilePath::checkName(obj->fileName.getValue());

  obj->connect();
  return true;
}
