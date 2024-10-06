// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFlLib/FFlLinkHandler.H"
#include "FFlLib/FFlElementBase.H"
#include "FFlLib/FFlFEParts/FFlNode.H"
#include "FFlLib/FFlFEParts/FFlPTHICK.H"
#include "FFlLib/FFlFEParts/FFlPMAT.H"

#include "FFaLib/FFaOS/FFaFilePath.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaParse.H"
#ifndef FT_NO_FATIGUE
#include "FFpLib/FFpFatigue/FFpSNCurve.H"
#include "FFpLib/FFpFatigue/FFpSNCurveLib.H"
#endif

#include "vpmDB/FmMechanism.H"
#include "vpmDB/FmCurveSet.H"
#include "vpmDB/FmDB.H"
#include "vpmDB/FmStrainRosette.H"

#include <fstream>

#ifdef USE_INVENTOR
#include "vpmDisplay/FdStrainRosette.H"
#include "vpmDisplay/FdPart.H"
#endif


/*!
  Position             \
                        } Pick CS/internal part CS
  In-plane Orientation /

  Leg configuration (Type)

  Height position of measurement (shell elms):
    Use thickness from element underneath/Layer position
    Inside/outside
  Material properties (E , nu) from underlying element
*/

const Strings& FmStrainRosette::getRosetteUINames()
{
  static Strings rosetteTypes;
  if (rosetteTypes.empty()) {
    rosetteTypes.push_back("Single gage");
    rosetteTypes.push_back("Double gage 90");
    rosetteTypes.push_back("Triple gage 60");
    rosetteTypes.push_back("Triple gage 45");
  }
  return rosetteTypes;
}


Fmd_DB_SOURCE_INIT(FcSTRAIN_ROSETTE, FmStrainRosette, FmIsPlottedBase);


FmStrainRosette::FmStrainRosette()
{
  Fmd_CONSTRUCTOR_INIT(FmStrainRosette);

  FFA_REFERENCE_FIELD_INIT(rosetteLinkField, rosetteLink, "ROSETTE_LINK"  );

  FFA_FIELD_INIT( rosetteType      , TRIPLE_GAGE_45, "ROSETTE_TYPE"       );

  FFA_FIELD_INIT( numNodes         ,              4, "NUM_NODES"          );

  FFA_FIELD_DEFAULT_INIT( nodePos1 ,                 "NODE_1_POSITION"    );
  FFA_FIELD_DEFAULT_INIT( nodePos2 ,                 "NODE_2_POSITION"    );
  FFA_FIELD_DEFAULT_INIT( nodePos3 ,                 "NODE_3_POSITION"    );
  FFA_FIELD_DEFAULT_INIT( nodePos4 ,                 "NODE_4_POSITION"    );

  FFA_FIELD_INIT( node1            ,             -1, "NODE_1"             );
  FFA_FIELD_INIT( node2            ,             -1, "NODE_2"             );
  FFA_FIELD_INIT( node3            ,             -1, "NODE_3"             );
  FFA_FIELD_INIT( node4            ,             -1, "NODE_4"             );

  FFA_FIELD_INIT( useFEThickness   ,           true, "USE_FE_THICKNESS"   );
  FFA_FIELD_INIT( zPos             ,              0, "HEIGHT"             );
  FFA_FIELD_INIT( FEThickness      ,              0, "FE_THICKNESS"       );

  FFA_FIELD_INIT( angle            ,              0, "ANGLE"              );
  FFA_FIELD_INIT( angleOrigin      ,         LINK_X, "ANGLE_ORIGIN"       );
  FFA_FIELD_INIT( angleOriginVector,  FaVec3(1,0,0), "ANGLE_ORIGIN_VECTOR");

  FFA_FIELD_INIT( useFEMaterial    ,           true, "USE_FE_MATERIAL"    );
  FFA_FIELD_INIT( EMod             ,             -1, "E_MODULE"           );
  FFA_FIELD_INIT( EModFE           ,             -1, "FE_E_MODULE"        );
  FFA_FIELD_INIT( nu               ,             -1, "POISSONS_RATIO"     );
  FFA_FIELD_INIT( nuFE             ,             -1, "FE_POISSONS_RATIO"  );

  FFA_FIELD_INIT(removeStartStrains, true, "SET_START_STRAINS_TO_ZERO");

  tmpZDirection = NULL;
#ifdef USE_INVENTOR
  itsDisplayPt = new FdStrainRosette(this);
#endif
}


FmStrainRosette::~FmStrainRosette()
{
  if (tmpZDirection)
    delete tmpZDirection;
  this->disconnect();
}


void FmStrainRosette::getEntities(std::vector<FmSensorChoice>& choicesToFill, int)
{
  choicesToFill.clear();
  choicesToFill.reserve(2);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::STRAIN]);
  choicesToFill.push_back(itsEntityTable[FmIsMeasuredBase::STRESS]);
}


void FmStrainRosette::getDofs(std::vector<FmSensorChoice>& choicesToFill)
{
  choicesToFill.clear();
  choicesToFill.reserve(7);
  choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::MAX_PR]);
  choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::MIN_PR]);
  choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::SA_MAX]);
  choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::VMISES]);
  if (this->rosetteType.getValue() >= SINGLE_GAGE)
    choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::GAGE_1]);
  if (this->rosetteType.getValue() >= DOUBLE_GAGE_90)
    choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::GAGE_2]);
  if (this->rosetteType.getValue() >= TRIPLE_GAGE_60)
    choicesToFill.push_back(itsDofTable[FmIsMeasuredBase::GAGE_3]);
}


/*!
  Returns the exact position of the strain gage based
  on node position, thickness of underlying element
  Will convert some data from old representation if neccesary.
*/

FaMat34 FmStrainRosette::getSymbolPosMx(bool& matrixIsOk)
{
  bool EzOK = true;
  matrixIsOk = true;

  // Orientation

  FaVec3 v1, v2, Exg, Ex1, Ey1;
  double a = this->angle.getValue();

  // First find the normal

  if (this->numNodes.getValue() == 3) {
    v1 = this->nodePos2.getValue() - this->nodePos1.getValue();
    v2 = this->nodePos3.getValue() - this->nodePos1.getValue();
  }
  else if (this->numNodes.getValue() == 4) {
    v1 = this->nodePos3.getValue() - this->nodePos1.getValue();
    v2 = this->nodePos4.getValue() - this->nodePos2.getValue();
  }
  else
    matrixIsOk = false;

  FaMat34 result;
  FaVec3& Ex = result[0];
  FaVec3& Ey = result[1];
  FaVec3& Ez = result[2];

  if (matrixIsOk)
  {
    Ez = v1 ^ v2;
    if (Ez.length() > 1.0e-10)
      Ez.normalize();
    else
    {
      EzOK = false;
      Ez = FaVec3(0.0,0.0,1.0);
      ListUI <<"  -> Error : Could not find a plane normal for "
             << this->getIdString() <<".\n";
    }
  }

  if (this->tmpZDirection && Ez*(*this->tmpZDirection) < 0.0) {
    this->flipFaceNormal();
    Ez = -Ez;
  }

  // Find Ex1 and Ey1, the unit x and y vectors of the strain gage
  // origin system without applying the user angle.

  // First find the vector that the user wated as origin for
  // the angular rotation Exg :

  switch (this->angleOrigin.getValue())
    {
    case LINK_X:
      Exg.x(1.0);
      break;
    case LINK_Y:
      Exg.y(1.0);
      break;
    case LINK_VECTOR:
      Exg = this->angleOriginVector.getValue();
      Exg.normalize();
      break;
    default:
      Exg.x(1.0);
      break;
    }

  // Ok, then the Ex1 :

  Ex1 = Exg - (Exg*Ez)*Ez;
  if (Ex1.length() > 1.0e-10)
    Ex1.normalize();
  else
  {
    // Ez and the angle origin vector is paralell, use something else
    switch (this->angleOrigin.getValue())
      {
      case LINK_X:
        Exg = FaVec3(0,1,0);
        break;
      case LINK_Y:
        Exg = FaVec3(1,0,0);
        break;
      case LINK_VECTOR:
        Exg = FaVec3(1,0,0);
        if (Exg.isParallell(this->angleOriginVector.getValue()))
          Exg = FaVec3(0,1,0);
        break;
      default:
        Exg = FaVec3(0,1,0);
        break;
      }

    ListUI <<"  -> Error : The direction reference for the rotation of "
           << this->getIdString()
           <<"\n             is parallel to the plane normal. Using ["
           << Exg <<"] instead.\n";
    Ex1 = Exg - (Exg*Ez)*Ez;
    Ex1.normalize();
  }

  if (EzOK)
  {
    // Then Ey1 :

    Ey1 = Ez ^ Ex1;
    Ey1.normalize();

    // And finally apply the angle to the unity vectors to get
    // the actual CS directions of the strain gage

    Ex = cos(a)*Ex1 + sin(a)*Ey1;
    Ey = Ez ^ Ex;
  }

  // Position : "Midpoint" of element:
  result[3] = this->getCalculationPoint();

  // Adding the height to show the actual thickness position
  double height = this->useFEThickness.getValue() ? (this->FEThickness.getValue()*0.5) : this->zPos.getValue();
  result[3] += height*Ez;

  return result;
}


/*!
  Returns the point where the strains are calculated excluding thickness/zPos.
*/

FaVec3 FmStrainRosette::getCalculationPoint() const
{
  if (this->numNodes.getValue() == 3)
    {
      // The midpoint of a triangle is given by the point 1/3*h from
      // each edge. This formula is from Irgens - Formelsamling i Mekanikk
      // Tabell 2. Arealsenter :

      // Xc = (2b - c)/3;  Yc = h/3;
      // Where
      // b  - Length of bottom edge
      // c  - Length of right edge projected onto bottom edge.
      // h  - Triangle height from bottom edge
      // Xc - Length to area center along bottom edge
      // Yc - Length to area center perpend. to bottom edge.
      // In the following : S - Side, U - Unit direction vector.

      FaVec3 P1  = nodePos1.getValue();
      FaVec3 S12 = nodePos2.getValue() - P1;
      FaVec3 S23 = nodePos3.getValue() - nodePos2.getValue();
      FaVec3 U12 = S12; U12.normalize();

      double c = -U12*S23;

      FaVec3 H  = S23 + c * U12;
      double Xc = (2.0*S12.length() - c)/3.0;

      return P1 + Xc * U12 + H/3.0;
    }
  else if (this->numNodes.getValue() == 4)
    {
      // the mid point of a four sides is the point where the lines from the
      // midpoints of two adjencant sides intersect
      FaVec3 P1 = nodePos1.getValue();
      FaVec3 P3 = nodePos3.getValue();

      FaVec3 P12 = P1 + 0.5*(nodePos2.getValue() - P1);
      FaVec3 P34 = P3 + 0.5*(nodePos4.getValue() - P3);

      return P12 + 0.5*(P34-P12);
    }

  return nodePos1.getValue();
}


/*!
  Returns the global position matrix where the strains are calculated.
*/

FaMat34 FmStrainRosette::getGlobSymbolPosMx(bool& matrixIsOk)
{
  FaMat34 point = this->getSymbolPosMx(matrixIsOk);
#ifdef USE_INVENTOR
  if (matrixIsOk && !rosetteLink.isNull() && rosetteLink->getFdPointer())
    return ((FdPart*)rosetteLink->getFdPointer())->getActiveTransform()*point;
#endif
  return point;
}


/*!
  Returns a width of the element to use for scaling the visualization symbol.
*/

double FmStrainRosette::getElmWidth() const
{
  double candidate;
  double width = (nodePos2.getValue() - nodePos1.getValue()).length();

  if (this->numNodes.getValue() > 2)
  {
    candidate = (nodePos3.getValue() - nodePos2.getValue()).length();
    if (candidate < width)
      width = candidate;
  }

  if (this->numNodes.getValue() == 3)
  {
    candidate = (nodePos1.getValue() - nodePos3.getValue()).length();
    if (candidate < width)
      width = candidate;
  }
  else if (this->numNodes.getValue() == 4)
  {
    candidate = (nodePos4.getValue() - nodePos3.getValue()).length();
    if (candidate < width)
      width = candidate;

    candidate = (nodePos1.getValue() - nodePos4.getValue()).length();
    if (candidate < width)
      width = candidate;
  }

  return 0.5*width;
}


bool FmStrainRosette::setTopology(FmPart* part, int n1, int n2, int n3, int n4)
{
  if (!part) return false;

  FFlNode* nod1 = part->getNode(n1);
  if (!nod1) return false;
  FFlNode* nod2 = part->getNode(n2);
  if (!nod2) return false;
  FFlNode* nod3 = part->getNode(n3);
  if (!nod3) return false;
  FFlNode* nod4 = part->getNode(n4);

  node1.setValue(n1);
  nodePos1.setValue(nod1->getPos());

  node2.setValue(n2);
  nodePos2.setValue(nod2->getPos());

  node3.setValue(n3);
  nodePos3.setValue(nod3->getPos());

  if (nod4) {
    node4.setValue(n4);
    nodePos4.setValue(nod4->getPos());
    numNodes.setValue(4);
  }
  else
    numNodes.setValue(3);

  rosetteLink.setRef(part);
  return true;
}


bool FmStrainRosette::setNode(int ID, int indx)
{
  if (rosetteLink.isNull())
    return false;

  FFlNode* node = rosetteLink->getNode(ID);
  if (!node) return false;

  switch (indx) {
  case 1:
    node1.setValue(ID);
    nodePos1.setValue(node->getPos());
    break;
  case 2:
    node2.setValue(ID);
    nodePos2.setValue(node->getPos());
    break;
  case 3:
    node3.setValue(ID);
    nodePos3.setValue(node->getPos());
    break;
  case 4:
    node4.setValue(ID);
    nodePos4.setValue(node->getPos());
    break;
  default:
    return false;
  }

  return true;
}


/*!
  Set the vector used as reference for the rotation angle as a global vector,
  as opposed to the angleOriginVector which is defined in local part directions.
*/

void FmStrainRosette::setGlobalAngleOriginVector(const FaVec3& dir)
{
#ifdef USE_INVENTOR
  if (!rosetteLink.isNull() && rosetteLink->getFdPointer()) {
    FaMat34 partMx = ((FdPart*)rosetteLink->getFdPointer())->getActiveTransform();
    angleOriginVector = partMx.direction().inverse() * dir;
  }
#else
  angleOriginVector = dir;
#endif
}


enum FmSyncErrorTypes
{
  NOT_VERIFIED,
  NODE_ERROR,
  ELEMENT_ERROR,
  NODE_IDS_CHANGED,
  NODE_POS_CHANGED,
  ELEMENT_MATERIAL_CHANGED,
  ELEMENT_THICKNESS_CHANGED,
  FATAL_ERROR,
  NUM_ERROR_CODES
};


/*!
  Syncronizes this strain rosette with the FE model.
  Try to use stored node positions to find matching nodes first.
  Will get the nodes closest to the stored positions.
  If the node positions are invalid (some are coincident) the ID's will be used.
*/

BoolVec FmStrainRosette::syncWithFEModel()
{
  BoolVec errorFlags(NUM_ERROR_CODES, false);
  if (this->numNodes.getValue() < 3) {
    errorFlags[NODE_ERROR] = errorFlags[FATAL_ERROR] = true;
    return errorFlags;
  }

  // Putting all our data into vectors for easier handling

  IntVec nodeNums;
  FaVec3Vec nodePos;
  nodeNums.reserve(4);
  nodePos.reserve(4);

  nodeNums.push_back(this->node1.getValue());
  nodeNums.push_back(this->node2.getValue());
  nodeNums.push_back(this->node3.getValue());
  if (this->numNodes.getValue() > 3)
    nodeNums.push_back(this->node4.getValue());
  IntVec nodeNumsBackup(nodeNums);

  nodePos.push_back(this->nodePos1.getValue());
  nodePos.push_back(this->nodePos2.getValue());
  nodePos.push_back(this->nodePos3.getValue());
  if (this->numNodes.getValue() > 3)
    nodePos.push_back(this->nodePos4.getValue());
  FaVec3Vec nodePosBackup(nodePos);

  // Check whether we have defined all nodes

  size_t n;
  bool nodeNumsIsOK = true;
  for (int node : nodeNums)
    nodeNumsIsOK &= node > 0;

  if (this->rosetteLink.isNull() || !this->rosetteLink->isFEPart(true))
  {
    // Unable to sync because no FE data is loaded
    if (nodeNumsIsOK)
      errorFlags[NOT_VERIFIED] = true;
    else
      errorFlags[NODE_ERROR] = errorFlags[FATAL_ERROR] = true;
    return errorFlags;
  }

  // Check whether the node positions can be considered invalid
  // by checking for coincident nodes within the element

  double tolerance = FmDB::getPositionTolerance();
  bool nodePosIsOK = true;
  for (n = 1; n < nodePos.size() && nodePosIsOK; n++)
    for (size_t i = 0; i < n && nodePosIsOK; i++)
      if (nodePos[i].equals(nodePos[n],tolerance))
        nodePosIsOK = false;

  FFlNode* node;
  if (nodePosIsOK)
  {
    // The positions are OK. Go find the nodes closest to them.
    // TODO : Use node ID as preference on what nodes to select.
    // Also check the resulting nodes whether they are OK to use.
    for (n = 0; n < nodePos.size(); n++)
      if ((node = rosetteLink->getClosestNode(nodePos[n])))
      {
        nodePos[n]  = node->getPos();
        nodeNums[n] = node->getID();
      }
    nodeNumsIsOK = true;
  }
  else if (nodeNumsIsOK)
  {
    // Positions were not OK. Go find them based on nodeIDs instead.
    for (n = 0; n < nodeNums.size(); n++)
      if ((node = rosetteLink->getNode(nodeNums[n])))
        nodePos[n] = node->getPos();
      else
        nodeNumsIsOK = false;
  }

  if (!nodeNumsIsOK)
  {
    errorFlags[NODE_ERROR] = errorFlags[FATAL_ERROR] = true;
    return errorFlags;
  }

  // Todo : Check that the nodes found is OK !

  // Check what has changed :

  bool posHasChanged = false;
  for (n = 0; n < nodePos.size() && !posHasChanged; n++)
    posHasChanged = !nodePos[n].equals(nodePosBackup[n],tolerance);

  bool nodeIdsHasChanged = false;
  for (n = 0; n < nodeNums.size() && !nodeIdsHasChanged; n++)
    nodeIdsHasChanged = nodeNums[n] != nodeNumsBackup[n];

  errorFlags[NODE_IDS_CHANGED] = nodeNumsIsOK && nodeIdsHasChanged;
  errorFlags[NODE_POS_CHANGED] = nodePosIsOK  && posHasChanged;

  this->node1 = nodeNums[0];
  this->node2 = nodeNums[1];
  this->node3 = nodeNums[2];
  if (nodeNums.size() > 3)
    this->node4 = nodeNums[3];

  this->nodePos1 = nodePos[0];
  this->nodePos2 = nodePos[1];
  this->nodePos3 = nodePos[2];
  if (nodePos.size() > 3)
    this->nodePos4 = nodePos[3];

  if (this->useFEMaterial.getValue() || this->useFEThickness.getValue())
  {
    CathegoryVec elmBrands({ FFlTypeInfoSpec::SOLID_ELM, FFlTypeInfoSpec::SHELL_ELM });
    FFlLinkHandler* lh = this->rosetteLink->getLinkHandler();
    FFlElementBase* el = lh->findClosestElement(this->getCalculationPoint(), elmBrands);
    if (el)
    {
      if (this->useFEThickness.getValue() && el->getCathegory() == FFlTypeInfoSpec::SHELL_ELM)
      {
        FFlAttributeBase* pThick = el->getAttribute("PTHICK");
        if (pThick)
        {
          double newFEThickness = ((FFlPTHICK*)pThick)->thickness.getValue();
          if (fabs(this->FEThickness.getValue() - newFEThickness) > 1e-10)
            errorFlags[ELEMENT_THICKNESS_CHANGED] = true;
          this->FEThickness = newFEThickness;
        }
      }
      if (this->useFEMaterial.getValue())
      {
        FFlAttributeBase* pMat = el->getAttribute("PMAT");
        if (pMat)
        {
          double newEmod = ((FFlPMAT*)pMat)->youngsModule.getValue();
          double newNu = ((FFlPMAT*)pMat)->poissonsRatio.getValue();
          if (fabs(this->EModFE.getValue()-newEmod) > 1.0e-10 ||
              fabs(this->nuFE.getValue()-newNu) > 1.0e-10)
            errorFlags[ELEMENT_MATERIAL_CHANGED] = true;
          this->EModFE = newEmod;
          this->nuFE = newNu;
        }
      }
    }
    else
      errorFlags[ELEMENT_ERROR] = true;
  }

  return errorFlags;
}


void FmStrainRosette::flipFaceNormal()
{
  if (this->numNodes.getValue() == 3)
  {
    std::swap(node2.getValue(),node3.getValue());
    std::swap(nodePos2.getValue(),nodePos3.getValue());
  }
  else if (this->numNodes.getValue() == 4)
  {
    std::swap(node2.getValue(),node4.getValue());
    std::swap(nodePos2.getValue(),nodePos4.getValue());
  }
}


/*!
  Syncronize all strain rosettes on part, or all strain rosettes if part = 0
*/

void FmStrainRosette::syncStrainRosettes(FmPart* part)
{
  std::vector<FmModelMemberBase*> rosettes;

  if (part)
    part->getReferringObjs(rosettes,"rosetteLink");
  else
    FmDB::getAllOfType(rosettes,FmStrainRosette::getClassTypeID());

  if (!rosettes.empty())
    ListUI <<"===> Syncronizing Strain Rosettes with FE data:\n";

  for (FmModelMemberBase* ros : rosettes)
  {
    BoolVec errorFlags = static_cast<FmStrainRosette*>(ros)->syncWithFEModel();

    // Errors found by syncronisation

    if (errorFlags[FATAL_ERROR])
      ListUI <<"  -> Error : "<< ros->getIdString(true)
             <<" is not properly defined.\n"
             <<"     It could not be syncronized with the FE mesh.\n";

    if (errorFlags[FATAL_ERROR] && errorFlags[NODE_ERROR])
      ListUI <<"     Something seems to be wrong with the node data.";

    if (errorFlags[NOT_VERIFIED])
      ListUI <<"  -> Note : "<< ros->getIdString(true)
             <<" was not syncronized. (FE data not loaded).\n";

    if (errorFlags[ELEMENT_ERROR])
      ListUI <<"  -> Error : An underlying Finite Element could not be found\n"
             <<"     for "<< ros->getIdString(true)
             <<". The material and/or thickness properties\n"
             <<"     could therefore not be syncronized with the FE mesh.\n";

    // Notes and warnings on changes done by syncronization

    if (errorFlags[ELEMENT_THICKNESS_CHANGED])
      ListUI <<"  -> Note : "<< ros->getIdString(true)
             <<" got a new thickness from the FE model.\n";

    if (errorFlags[ELEMENT_MATERIAL_CHANGED])
      ListUI <<"  -> Note : "<< ros->getIdString(true)
             <<" got a new material data from the FE model.\n";

    if (errorFlags[NODE_IDS_CHANGED] && errorFlags[NODE_POS_CHANGED])
      ListUI <<"  -> Warning : "<< ros->getIdString(true)
             <<" was repositioned to new nodes.\n";
    else if (errorFlags[NODE_POS_CHANGED])
      ListUI <<"  -> Warning : "<< ros->getIdString(true)
             <<" was repositioned.\n";
    else if (errorFlags[NODE_IDS_CHANGED])
      ListUI <<"  -> Note : "<< ros->getIdString(true)
             <<" was connected to new nodes.\n";

    if (errorFlags[FATAL_ERROR]   || errorFlags[NODE_ERROR] ||
        errorFlags[ELEMENT_ERROR] || errorFlags[NOT_VERIFIED])
      return;

    // Concluding message
    ListUI <<"  -> "<< ros->getIdString(true) <<" was syncronized OK.\n";
    static_cast<FmIsRenderedBase*>(ros)->draw();
  }
}


std::ostream& FmStrainRosette::writeFMF(std::ostream &os)
{
  os <<"STRAIN_ROSETTE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


bool FmStrainRosette::readAndConnect(std::istream& is, std::ostream&)
{
  FmStrainRosette* obj = new FmStrainRosette();

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


bool FmStrainRosette::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmStrainRosette::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmStrainRosette::getClassTypeID());
}


bool FmStrainRosette::writeSolverFile(const std::string& fsiFile,
				      const FmPart* part)
{
  std::vector<FmModelMemberBase*> rosettes;
  FmDB::getAllOfType(rosettes,FmStrainRosette::getClassTypeID());
  if (rosettes.empty()) return false;

  FILE* fd = fopen(fsiFile.c_str(),"w");
  if (!fd)
  {
    ListUI <<"===> Could not open gage solver input file: "<< fsiFile <<"\n";
    return false;
  }

  int okStrainGages = 0;
  for (FmModelMemberBase* ros : rosettes)
  {
    FmStrainRosette* rosette = static_cast<FmStrainRosette*>(ros);
    if (!part || rosette->rosetteLink.getPointer() == part)
      okStrainGages += ros->printSolverEntry(fd);
  }

  fclose(fd);
  return okStrainGages > 0;
}


int FmStrainRosette::printSolverEntry(FILE* fd)
{
  bool rosetteOk = true;
  FaMat34 rPos(this->getSymbolPosMx(rosetteOk));
  if (!rosetteOk)
  {
    ListUI <<"Warning : "<< this->getIdString() <<" is not properly defined (ignored).\n"
	   <<"          Please check the nodes and the angle origin for possible singularities.\n";
    return 0;
  }

  fprintf(fd,"&STRAIN_ROSETTE\n");
  this->printID(fd);
  if (!this->rosetteLink.isNull())
    fprintf(fd,"  linkId = %d\n", this->rosetteLink->getBaseID());

  fprintf(fd,"  type = '%s'\n", this->rosetteType.getValue().getText());
  fprintf(fd,"  zeroInit = %d\n", (int)this->removeStartStrains.getValue());
  fprintf(fd,"  numnod = %d\n", this->numNodes.getValue());
  if (this->numNodes.getValue() > 0)
  {
    fprintf(fd,"  nodes = %d", this->node1.getValue());
    if (this->numNodes.getValue() > 1)
    {
      fprintf(fd," %d", this->node2.getValue());
      if (this->numNodes.getValue() > 2)
      {
	fprintf(fd," %d", this->node3.getValue());
	if (this->numNodes.getValue() > 3)
	  fprintf(fd," %d", this->node4.getValue());
      }
    }
    fprintf(fd,"\n");
  }

  fprintf(fd,"  rPos =%17.9e %17.9e %17.9e %17.9e\n",
	  rPos[0][0],rPos[1][0],rPos[2][0],rPos[3][0]);
  fprintf(fd,"        %17.9e %17.9e %17.9e %17.9e\n",
	  rPos[0][1],rPos[1][1],rPos[2][1],rPos[3][1]);
  fprintf(fd,"        %17.9e %17.9e %17.9e %17.9e\n",
	  rPos[0][2],rPos[1][2],rPos[2][2],rPos[3][2]);

  fprintf(fd,"  zPos =%17.9e\n", this->getZPos());
  fprintf(fd,"  Emod =%17.9e\n", this->getEMod());
  fprintf(fd,"  nu   =%17.9e\n", this->getNu());

#ifndef FT_NO_FATIGUE
  // Check if fatigue calculation is enabled for this strain rosette
  std::vector<FmCurveSet*> curves;
  this->getCurveSets(curves);
  for (FmCurveSet* curve : curves)
    if (curve->isFatigueCurve())
    {
      // Use S-N curve parameters from the first curve plotting this rosette
      int stdIdx = curve->getFatigueSNStd();
      int curveIdx = curve->getFatigueSNCurve();
      FFpSNCurve* snC = FFpSNCurveLib::instance()->getCurve(stdIdx,curveIdx);
      if (snC && snC->getStdId() == FFpSNCurve::NORSOK) // NorSok curves only
      {
	double gate = curve->getFatigueGateValue() * 1.0e-6;
	FmMechanism* mech = FmDB::getMechanismObject();
	mech->modelDatabaseUnits.getValue().convert(gate,"FORCE/AREA");
	fprintf(fd,"  gateVal =%17.9e\n", gate);
	fprintf(fd,"  snCurve =%17.9e%17.9e", snC->loga[0], snC->loga[1]);
	fprintf(fd,"%17.9e%17.9e\n", snC->m[0], snC->m[1]);
	break;
      }
    }
#endif

  fprintf(fd,"/\n\n");
  return 1;
}


bool FmStrainRosette::writeRosetteInputFile(const std::string& rosFile,
					    const FmPart* part)
{
  std::vector<FmModelMemberBase*> rosettes;
  FmDB::getAllOfType(rosettes,FmStrainRosette::getClassTypeID());
  if (rosettes.empty()) return true;

  std::ofstream rosStream(rosFile.c_str(),std::ios::out);
  if (!rosStream)
  {
    ListUI <<"===> Could not open strain rosette definition file: "<< rosFile <<"\n";
    return false;
  }

  for (FmModelMemberBase* ros : rosettes)
  {
    FmStrainRosette* rosette = static_cast<FmStrainRosette*>(ros);
    if (!part || rosette->rosetteLink.getPointer() == part)
      rosette->writeToRosetteInputFile(rosStream);
  }

  rosStream <<"end\n";
  rosStream.close();
  return true;
}


bool FmStrainRosette::writeToRosetteInputFile(std::ostream& os)
{
  bool rosetteOk = true;
  FaMat34 rosettePosition = this->getSymbolPosMx(rosetteOk);

  if (!rosetteOk)
    ListUI <<"Warning : "<< this->getIdString() <<" is not properly defined.\n"
	   <<"          Please check the nodes and the angle origin for possible singularities.\n";

  os << this->getID();
  switch (this->rosetteType.getValue()) {
  case SINGLE_GAGE   : os <<" 1"; break;
  case DOUBLE_GAGE_90: os <<" 2"; break;
  case TRIPLE_GAGE_60: os <<" 3"; break;
  case TRIPLE_GAGE_45: os <<" 4"; break;
  default: os <<" 2"; break;
  }

  if (this->rosetteLink.isNull())
    os <<" -1 ";
  else
    os <<" "<< this->rosetteLink->getID() <<" ";

  os << this->numNodes.getValue() <<" ";
  if (this->numNodes.getValue() > 0)
    os << node1.getValue() <<" ";
  if (this->numNodes.getValue() > 1)
    os << node2.getValue() <<" ";
  if (this->numNodes.getValue() > 2)
    os << node3.getValue() <<" ";
  if (this->numNodes.getValue() > 3)
    os << node4.getValue() <<" ";

  os << this->getZPos() <<" "
     << rosettePosition[0] <<" "<< rosettePosition[2] <<" "
     << this->getEMod() <<" "<< this->getNu() << std::endl;

  return rosetteOk;
}


/*!
  This static method is supposed to be used when you need to read an old
  strain gage input file and convert it to strain rosette objects in the model.
  The supplied file name can be relative to the model file or absolute.
  After this method is invoked, all strain rosettes from the file are read,
  and strain rosette objects are created. They are however not resolved
  (regarding pointer to part) nor syncronized with underlying FE mesh.
*/

bool FmStrainRosette::createRosettesFromOldFile(const std::string& fileName,
						bool defaultResetStartStrainValue)
{
  if (fileName.empty()) return false;

  std::string rosFile = fileName;
  FFaFilePath::makeItAbsolute(rosFile,FmDB::getMechanismObject()->getAbsModelFilePath());

  std::ifstream is(rosFile.c_str());
  if (!is)
  {
    ListUI <<" --> Error : Could not open strain rosette input file: "
           << rosFile <<"\n";
    return false;
  }

  ListUI <<" --> Reading strain rosettes from old definition file: "
         << rosFile <<"\n";

  int id, type, partid, numNodes, n1, n2, n3, n4;
  double zHeight, Xx, Xy, Xz, Zx, Zy, Zz, Emod, nu;
  int nReadStrainRosettes = 0;
  bool isEndReached = false;

  for (int lineNumber = 1; !is.eof() && !isEndReached; lineNumber++)
  {
    std::string line;
    std::string::iterator it;
    std::getline(is,line);

    bool errorEncountered = false;
    bool foundComment = false;
    bool foundText = false;
    for (it = line.begin(); it != line.end() && !foundComment; ++it)
      if (*it == '#')
        foundComment = true;
      else if (!isspace(*it))
        foundText = true;

    if (foundComment)
    {
      ListUI <<"     "<< std::string(it,line.end()) <<"\n";
      line.erase(it,line.end());
    }

    if (foundText)
    {
      if (sscanf(line.c_str(),"%d%d%d%d",&id,&type,&partid,&numNodes) == 4)
      {
        if (numNodes == 3)
        {
          n4 = -1;
          if (sscanf(line.c_str(),"%d%d%d%d%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                     &id, &type, &partid, &numNodes, &n1, &n2, &n3,
                     &zHeight, &Xx, &Xy, &Xz, &Zx, &Zy, &Zz, &Emod, &nu) != 16)
            errorEncountered = true;
        }
        else if (numNodes == 4)
        {
          if (sscanf(line.c_str(),"%d%d%d%d%d%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                     &id, &type, &partid, &numNodes, &n1, &n2, &n3, &n4,
                     &zHeight, &Xx, &Xy, &Xz, &Zx, &Zy, &Zz, &Emod, &nu) != 17)
            errorEncountered = true;
        }
        else
          errorEncountered = true;
      }
      else if (line.find("end") != std::string::npos)
        isEndReached = true;
      else
        errorEncountered = true;

      if (errorEncountered)
        ListUI <<"     Error: Line "<< lineNumber
               <<" : Could not read strain gage.\n";
      else if (!isEndReached)
      {
        FmStrainRosette* newRosette = new FmStrainRosette;
#ifdef FM_DEBUG
        std::cerr << id <<" "<< type <<" "<< partid <<" "<< numNodes
                  <<'\n'<< n1 <<" "<< n2 <<" "<< n3 <<" "<< n4
                  <<'\n'<< zHeight <<" "<< Xx <<" "<< Xy <<" "<< Xz
                  <<' '<< Zx <<' '<< Zy <<' '<< Zz
                  <<' '<< Emod <<' '<< nu << std::endl;
#endif
        newRosette->setID(id);

        switch (type) {
        case 1: newRosette->rosetteType = SINGLE_GAGE   ; break;
        case 2: newRosette->rosetteType = DOUBLE_GAGE_90; break;
        case 3: newRosette->rosetteType = TRIPLE_GAGE_60; break;
        case 4: newRosette->rosetteType = TRIPLE_GAGE_45; break;
        default: newRosette->rosetteType = DOUBLE_GAGE_90; break;
        }

        newRosette->rosetteLink.setRef(partid, FmPart::getClassTypeID());

        newRosette->numNodes = numNodes;
        newRosette->node1 = n1;
        newRosette->node2 = n2;
        newRosette->node3 = n3;
        newRosette->node4 = n4;

        newRosette->zPos = zHeight;

        newRosette->angleOriginVector = FaVec3(Xx, Xy, Xz);
        newRosette->angle = 0;
        newRosette->angleOrigin = LINK_VECTOR;

        newRosette->tmpZDirection = new FaVec3(Zx, Zy, Zz);
        newRosette->useFEThickness = false;

        newRosette->useFEMaterial = false;
        newRosette->EMod = Emod;
        newRosette->nu = nu;
        newRosette->removeStartStrains = defaultResetStartStrainValue;

        newRosette->connect();
        ++nReadStrainRosettes;
      }
    }
  }

  ListUI <<" --> Done. Read "<< nReadStrainRosettes <<" rosettes.\n";
  return true;
}
