// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include <cstdio>

#include "FFaLib/FFaString/FFaParse.H"

#include "vpmDB/FmGenericDBObject.H"


Fmd_DB_SOURCE_INIT(FcGENERIC_DB_OBJECT, FmGenericDBObject, FmSimulationModelBase);


FmGenericDBObject::FmGenericDBObject()
{
  Fmd_CONSTRUCTOR_INIT(FmGenericDBObject);

  // Add the base ID to the list of fields to be saved in the model file,
  // as this object may be referred by other Generic DB objects in the model.
  // It should therefore preserve the same base ID from session to session.
  FFA_FIELD_INIT(myBaseID, -1, "BASE_ID");

  FFA_FIELD_DEFAULT_INIT(objectType,"OBJECT_TYPE");
  FFA_FIELD_DEFAULT_INIT(objectDefinition,"OBJECT_DEFINITION");
}


FmGenericDBObject::~FmGenericDBObject()
{
  this->disconnect();
}


std::ostream& FmGenericDBObject::writeFMF(std::ostream& os)
{
  os <<"GENERIC_DB_OBJECT\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmGenericDBObject::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmGenericDBObject::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmGenericDBObject::getClassTypeID());
}


bool FmGenericDBObject::readAndConnect(std::istream& is, std::ostream&)
{
  FmGenericDBObject* obj = new FmGenericDBObject();

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


int FmGenericDBObject::printSolverEntry(FILE* fp)
{
  std::string definition = objectDefinition.getValue();
  // Remove trailing newlines, if any
  while (definition.back() == '\n')
    definition.pop_back();
  // Insert two spaces in front of each line
  for (size_t i = definition.size(); i > 0; i--)
    if (definition[i] == '\n')
      definition.insert(i+1,2,' ');

  fprintf(fp, "'Generic DB-object\n");
  fprintf(fp, "&%s\n", objectType.getValue().c_str());
  this->printID(fp);
  fprintf(fp, "  %s\n/\n\n", definition.c_str());
  return 0;
}
