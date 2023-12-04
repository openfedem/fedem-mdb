<!---
  SPDX-FileCopyrightText: 2023 SAP SE

  SPDX-License-Identifier: Apache-2.0

  This file is part of FEDEM - https://openfedem.org
--->

# The FEDEM model database (Fm class hierarchy)

The source files in this directory make up the model database of FEDEM.

## Creating database classes

The FEDEM model database is designed with strong focus on inheritance.
All classes that are going to be part of the database inherit `FmBase`.
This ensures a common interface for management of I/O, ID numbering, etc.
(At this point, ignore the class relationship to `FFaFieldContainer`
and different flavors of `FFaViewItem` - they might be explained later.)

Inheriting from `FmBase`, several general classes define common interfaces to,
e.g., positioned objects (`FmIsPositionedBase`),
objects where sensors can be attached (`FmIsMeasuredBase`), and so on.
The class naming gives a hint on what to expect from a base class,
and by browsing the inheritance tree, even better understanding can be gained.

To illustrate the steps needed to create a new class in the database,
the class representing a "Generic DataBase Object", `FmGenericDBObject`,
is used as an example in the following.

NOTE: Browse the sources as you walk trough this text,
so that you can get a better overview of the class hierarchy and structure.

### Finding the place in the hierarchy

Each database object has specific properties that might common to other classes.
If this is the case, there is usually a base class handling the common variables
and references (e.g., `FmDamperBase`).

For `FmGenericDBObject`, the base class would be `FmSimulationModelBase`,
since our new class is very simple and has no special references.

* Step 1: Creating the new files.
  Copy a "simple" class from the level selected.
  For our example, we copy the files `FmGageOptions.[CH]` to `FmGenericDBObject.[CH]`.
  Remember to add the files to the build system in the `CMakeLists.txt` file.

* Step 2: Update class names etc.
  Review the header file and replace the class names in the `#define` statement:
```
  #ifndef FM_GENERIC_DB_OBJECT_H
  #define FM_GENERIC_DB_OBJECT_H
```
  And in the class definition, constructor/destructor and UI string method:
```
  class FmGenericDBObject : public FmSimulationModelBase
  {
  public:
    FmGenericDBObject();
    ...
    virtual const char* getUITypeName() const { return "Generic object"; }

  protected:
    virtual ~FmGenericDBObject();
    ...
  };
```
  In the implementation file, the include statement and all class names
  in the method implementation need to be replaced.

  In addition to this, there is a few 'not-so-obvious' places to update:

  The `Fmd_DB_SOURCE_INIT` macro:
```
  Fmd_DB_SOURCE_INIT(FcGENERIC_DB_OBJECT, FmGenericDBObject, FmSimulationModelBase);
```
  The first parameter is a string used for internal reference, the second
  argument is the class name and the third is the class inherited from.
  This macro manages type checking, and some database-specific methods
  for traversing the inheritance tree when erasing or printing the object.

  The same goes with the `Fmd_CONSTRUCTOR_INIT` macro:
```
  Fmd_CONSTRUCTOR_INIT(FmGenericDBObject);
```

  The last place which is important to update is the I/O methods:
```
  std::ostream& FmGenericDBObject::writeFMF(std::ostream& os)
  {
    os <<"GENERIC_DB_OBJECT\n{\n";
    this->writeFields(os);
    os <<"}\n\n";
    return os;
  }
```
  Here, the object type name as found in the FMM-file is written.
  This string is important in the parsing in `FmDB.C`.

### Model file I/O

To get instances of the new class down on disk and back again,
several updates needs to be done in the source file `FmDB.C`.

* Step 1: Add the proper include statement:
```
  #include "FmGenericDBObject.H"
```

* Step 2: Update the `FmDB::init()` method.
  This method initiates the "head-table", e.g., the data structure that manages
  all the different objects in the database when the model is loaded.
  At the end of the headMap list, add:
```
  myHeadMap[FmGenericDBObject::getClassTypeID()] = new FmRingStart("Generic objects");
```
  The text string at the end is the text that is used in the GUI list views.
  (If there is a need for more topology information in the list views,
  this can be specified with the `setParent` method,
  as done just below the headMap initialization.)

* Step 3: Update the `FmDB::readFMF()` method.
  This method is used when the model file is read from disk, and
  it dispatches the local parsing methods of the various model entity classes.

  Add an enum to the end of the enum list, and update the numbers
  (NOTE: The position is essential!):
```
  enum { FEDEMMODELFILE = 1,
         MECHANISM = 2,
         ANALYSIS = 3,
         ...
         GAGEOPTIONS = 80,
         GENERIC_DB_OBJECT = 81,
         END = 82 };
```
  Add the entity name to the keyword array:
```
  static const char* key_words[] = {
    "FEDEMMODELFILE",
    "MECHANISM",
    "ANALYSIS",
    ...
    "GAGEOPTIONS",
    "GENERIC_DB_OBJECT",
    "END",
    NULL };
```
  Add the proper case statement to the switch sentence:
```
  case GENERIC_DB_OBJECT: FmdPARSE_AND_BUILD_LOG(FmGenericDBObject); break;
```

Now the code can be compiled. Lot's of stuff will probably choke,
but I/O and list view manipulation should work.

When adding a class for a new object type you must also consider
the impacts and behaviour for the other program features:

* Create (of course)
* Delete
* Select / highlight
* Attach
* Detach
* Smart Move
* Stickers
* Snapping
* View filter
* Appearance/detail
* ... ?

## Adding variables

Variables (or fields) are added to the objects using the
`FFaField` and `FFaFieldContainer` classes.
The variables can be of arbitrary type, but the type needs to have definitions
of input stream (>>), output stream (<<) and assignment (=) operators.
All trivial C++ data types have this, so adding, e.g., strings, integers
and floats works automatically. For user-defined types, see the implementation
in, e.g., `FmResultStatusData`.

* Step 1: Updating the header file.
  Each variable is defined in public scope in the header file.
  (They can of course also be protected or private, see comment below.)
```
  FFaField<std::string> objectDefinition;
```
  Here the `FFaField` template is instantiated with a `std::string`.
  For examples concerning other data types, visit, e.g., `FmGageOptions`.

* Step 2: Updating the implementation file.
  The macros `FFA_FIELD_INIT` and  `FFA_FIELD_DEFAULT_INIT`,
  defined in `FFaFieldContainer.H` are used to add the defined fields.
```
  FFA_FIELD_INIT(objectDefinition,"","OBJECT_DEFINITION");
```
  Here, the first macro parameter is the variable name, the second is the
  default value (use `FFA_FIELD_DEFAULT_INIT` instead if you want to use
  the default class constructor), and the third argument is the field name
  that will be used in the model file.

This is all that needs to be done for fields.
Access methods are documented in the `FFaField` class.

When you work with fields, compatibility must always be taken into account.
This means that if a field is introduced in a public release, this field
needs to stick with the software forever to ensure backward compatibility.
We should not be flooded with fields, so be sure that the syntax and meaning
is right before you promote a new field.

## Adding references

Guarded pointers, or references to other objects can be added using the class
`FFaReference` for single references, and `FFaReferenceList` for multiple ones.
These classes work together with `FFaField`, and `FFaFieldContainer`
to provide automated I/O of the references as well.

Please refer to the documentation on the `FFaReference` and `FFaReferenceList`
on how to add them to your class, and how they are to be used with `FFaField`.

## Communication with the FEDEM solvers

To be able to communicate with the dynamics solver, the information in the
database objects must be written to the solver input file `fedem_solver.fsi`.
This is done in `FmSolverParser.C`.

* Step 1: Add new call to `writeAllOfType()` in `FmSolverParser::writeFullFile`:
```
  int FmSolverParser::writeFullFile()
  {
    if (!myFile) return 999;

    int err = writeHeading();
    err += writeEnvironment();
    ....
    err += writeAllOfType(FmGenericDBObject::getClassTypeID());
  }
```

* Step 2: Implement the virtual method that writes the object data.
  In the header file `FmGenericDBObj.H`:
```
  virtual int printSolverEntry(FILE* fp);
```
  And in the implementation file `FmGenericDBObj.C`:
```
  int FmGenericDBObject::printSolverEntry(FILE* fp)
  {
    fprintf(fp,"'Generic DB-object\n");
    ...
    fprintf(fp,"/\n\n");
    return 0;
  }
```
  This method will write the class instance to the fsi-file.
  Writing the general object information is done by first specifying the
  solver object type, and then print ID, baseID and externalDescription.
  The latter three actions are done by calling the printID() method implemented
  in the base class `FmModelMemberBase`:
```
  fprintf(fp,"&TRIAD\n");
  this->printID(fp);
```

  The `FFaField`-defined variables of a class are printed like, e.g.:
```
  fprintf(fp,"  numGenDOFs = %d\n",this->nGenModes.getValue());
```
  Remember to end all fields with a new-line (`\n`).
