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
#endif

#ifdef FF_NAMESPACE
using namespace FF_NAMESPACE;
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

  FFA_REFERENCE_FIELD_INIT(rosetteLinkField, rosetteLink, "ROSETTE_LINK");

  FFA_FIELD_INIT(rosetteType      ,   SINGLE_GAGE, "ROSETTE_TYPE"       );

  FFA_FIELD_INIT(useFEThickness   ,          true, "USE_FE_THICKNESS"   );
  FFA_FIELD_INIT(zPos             ,           0.0, "HEIGHT"             );
  FFA_FIELD_INIT(FEThickness      ,           0.0, "FE_THICKNESS"       );

  FFA_FIELD_INIT(angle            ,           0.0, "ANGLE"              );
  FFA_FIELD_INIT(angleOrigin      ,   LINK_VECTOR, "ANGLE_ORIGIN"       );
  FFA_FIELD_INIT(angleOriginVector, FaVec3(1,0,0), "ANGLE_ORIGIN_VECTOR");

  FFA_FIELD_INIT(useFEMaterial    ,          true, "USE_FE_MATERIAL"    );
  FFA_FIELD_INIT(EMod             ,          -1.0, "E_MODULE"           );
  FFA_FIELD_INIT(EModFE           ,          -1.0, "FE_E_MODULE"        );
  FFA_FIELD_INIT(nu               ,          -1.0, "POISSONS_RATIO"     );
  FFA_FIELD_INIT(nuFE             ,          -1.0, "FE_POISSONS_RATIO"  );

  std::string identifier("NODE_1");
  for (int i = 0; i < 4; i++, ++identifier.back())
  {
    FFA_FIELD_INIT(node[i], -1, identifier);
    FFA_FIELD_DEFAULT_INIT(nodePos[i], identifier + "_POSITION");
  }

  FFA_FIELD_INIT(removeStartStrains, true, "SET_START_STRAINS_TO_ZERO");

  tmpZDirection = NULL;
#ifdef USE_INVENTOR
  itsDisplayPt = new FdStrainRosette(this);
#endif
}


FmStrainRosette::~FmStrainRosette()
{
  delete tmpZDirection;
  this->disconnect();
}


void FmStrainRosette::getEntities(std::vector<FmSensorChoice>& choices, int)
{
  choices = {
    itsEntityTable[FmIsMeasuredBase::STRAIN],
    itsEntityTable[FmIsMeasuredBase::STRESS]
  };
}


void FmStrainRosette::getDofs(std::vector<FmSensorChoice>& choices)
{
  choices.reserve(7);
  choices = {
    itsDofTable[FmIsMeasuredBase::MAX_PR],
    itsDofTable[FmIsMeasuredBase::MIN_PR],
    itsDofTable[FmIsMeasuredBase::SA_MAX],
    itsDofTable[FmIsMeasuredBase::VMISES]
  };
  if (rosetteType.getValue() >= SINGLE_GAGE)
    choices.push_back(itsDofTable[FmIsMeasuredBase::GAGE_1]);
  if (rosetteType.getValue() >= DOUBLE_GAGE_90)
    choices.push_back(itsDofTable[FmIsMeasuredBase::GAGE_2]);
  if (rosetteType.getValue() >= TRIPLE_GAGE_60)
    choices.push_back(itsDofTable[FmIsMeasuredBase::GAGE_3]);
}


/*!
  Returns the exact position of the strain gage
  based on node position and thickness of underlying element.
  Will convert some data from old representation if neccesary.
  Optionally transform to global coordinate axes.
*/

FaMat34 FmStrainRosette::getSymbolPosMx(bool global) const
{
  // Orientation

  double a = angle.getValue();
  int n3 = node[3].getValue() > 0 ? 3 : 2;

  // First find the normal

  FaVec3 v1 = nodePos[n3-1].getValue() - nodePos[0].getValue();
  FaVec3 v2 = nodePos[n3].getValue() - nodePos[n3-2].getValue();

  FaMat34 result;
  FaVec3& Ex = result[0];
  FaVec3& Ey = result[1];
  FaVec3& Ez = result[2];
  bool EzOK = true;

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

  if (tmpZDirection)
  {
    if (Ez*(*tmpZDirection) < 0.0)
    {
      const_cast<FmStrainRosette*>(this)->flipFaceNormal();
      Ez = -Ez;
    }
    delete tmpZDirection;
    tmpZDirection = NULL;
  }

  // Find Ex1 and Ey1, the unit x and y vectors of the strain gage
  // origin system without applying the user angle.

  // First find the vector that the user wanted as origin for
  // the angular rotation Exg :

  FaVec3 Exg, Ex1, Ey1;
  switch (angleOrigin.getValue())
    {
    case LINK_X:
      Exg.x(1.0);
      break;
    case LINK_Y:
      Exg.y(1.0);
      break;
    case LINK_VECTOR:
      Exg = angleOriginVector.getValue();
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
    // Ez and the angle origin vector is parallel, use something else
    switch (angleOrigin.getValue())
      {
      case LINK_X:
        Exg = FaVec3(0,1,0);
        break;
      case LINK_Y:
        Exg = FaVec3(1,0,0);
        break;
      case LINK_VECTOR:
        Exg = FaVec3(1,0,0);
        if (Exg.isParallell(angleOriginVector.getValue()))
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

  // Position : Midpoint of element + height to show actual thickness position
  result[3] = this->getCalculationPoint() + this->getZPos()*Ez;

  if (global && !rosetteLink.isNull() && global)
    return rosetteLink->getTransform() * result;

  return result;
}


/*!
  Returns the point where the strains are calculated excluding thickness/zPos.
*/

FaVec3 FmStrainRosette::getCalculationPoint() const
{
  if (node[3].getValue() < 0)
  {
    // The midpoint of a triangle is given by the point 1/3*h from each edge.
    // This formula is from Irgens - Formelsamling i Mekanikk
    // Tabell 2. Arealsenter :

    // Xc = (2b - c)/3;  Yc = h/3;
    // Where
    // b  - Length of bottom edge
    // c  - Length of right edge projected onto bottom edge.
    // h  - Triangle height from bottom edge
    // Xc - Length to area center along bottom edge
    // Yc - Length to area center perpend. to bottom edge.
    // In the following : S - Side, U - Unit direction vector.

    FaVec3 P1  = nodePos[0].getValue();
    FaVec3 S12 = nodePos[1].getValue() - P1;
    FaVec3 S23 = nodePos[2].getValue() - nodePos[1].getValue();
    FaVec3 U12 = S12; U12.normalize();

    double c = -U12*S23;

    FaVec3 H  = S23 + c * U12;
    double Xc = (2.0*S12.length() - c)/3.0;

    return P1 + Xc * U12 + H/3.0;
  }
  else
  {
    // The midpoint of a quadrilateral is the point where the lines from the
    // midpoints of two adjacent sides intersect
    FaVec3 P1 = nodePos[0].getValue();
    FaVec3 P3 = nodePos[2].getValue();

    FaVec3 P12 = P1 + 0.5*(nodePos[1].getValue() - P1);
    FaVec3 P34 = P3 + 0.5*(nodePos[3].getValue() - P3);

    return P12 + 0.5*(P34-P12);
  }
}


/*!
  Returns a width of the element to use for scaling the visualization symbol.
*/

double FmStrainRosette::getElmWidth() const
{
  double width     = (nodePos[1].getValue() - nodePos[0].getValue()).length();
  double candidate = (nodePos[2].getValue() - nodePos[1].getValue()).length();
  if (candidate < width)
    width = candidate;

  if (node[3].getValue() < 0)
  {
    candidate = (nodePos[0].getValue() - nodePos[2].getValue()).length();
    if (candidate < width)
      width = candidate;
  }
  else
  {
    candidate = (nodePos[3].getValue() - nodePos[2].getValue()).length();
    if (candidate < width)
      width = candidate;

    candidate = (nodePos[0].getValue() - nodePos[3].getValue()).length();
    if (candidate < width)
      width = candidate;
  }

  return 0.5*width;
}


bool FmStrainRosette::setTopology(FmPart* part, const IntVec& nodes)
{
  if (part)
    rosetteLink.setRef(part);

  for (size_t i = 0; i < 4; i++)
    if (i < nodes.size())
    {
      node[i].setValue(nodes[i]);
      if (part)
      {
        FFlNode* nod = part->getNode(nodes[i]);
        if (!nod) return false;

        nodePos[i].setValue(nod->getPos());
      }
    }
    else
      node[i].setValue(-1);

  return true;
}


FmPart* FmStrainRosette::getTopology(IntVec& nodes) const
{
  nodes.resize(this->getNoNodes());
  for (size_t i = 0; i < nodes.size(); i++)
    nodes[i] = node[i].getValue();

  return rosetteLink.getPointer();
}


bool FmStrainRosette::setNode(int ID, int idx)
{
  if (idx < 0 || idx > 3)
    return false;

  node[idx].setValue(ID);

  if (rosetteLink.isNull())
    return false;

  FFlNode* nod = rosetteLink->getNode(ID);
  if (!nod) return false;

  nodePos[idx].setValue(nod->getPos());

  return true;
}


const FaVec3& FmStrainRosette::getNodePos(int idx) const
{
  if (idx < 0 || idx >= this->getNoNodes())
  {
    static FaVec3 Origin;
    return Origin;
  }

  return nodePos[idx].getValue();
}


/*!
  Set the vector used as reference for the rotation angle as a global vector,
  as opposed to the angleOriginVector which is defined in local part directions.
*/

void FmStrainRosette::setGlobalAngleOriginVector(const FaVec3& dir)
{
  if (rosetteLink.isNull())
    angleOriginVector.setValue(dir);
  else
    angleOriginVector.setValue(dir * rosetteLink->getTransform().direction());
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
  If \a forceUseID is \e true, the stored node positions are ignored.
*/

BoolVec FmStrainRosette::syncWithFEModel(bool forceUseID)
{
  BoolVec errorFlags(NUM_ERROR_CODES, false);

  // Putting our node data into vectors for easier handling

  size_t num_nodes = this->getNoNodes();
  IntVec nodeNums(num_nodes);
  FaVec3Vec nodePoss(num_nodes);
  for (size_t i = 0; i < num_nodes; i++)
  {
    nodeNums[i] = node[i].getValue();
    nodePoss[i] = nodePos[i].getValue();
  }

  // Check whether we have defined all nodes

  bool nodeNumsOK = std::all_of(nodeNums.begin(), nodeNums.end(),
                                [](int n){ return n > 0; });

  if (rosetteLink.isNull() || !rosetteLink->isFEPart(true))
  {
    // Unable to sync because no FE data is loaded
    if (nodeNumsOK)
      errorFlags[NOT_VERIFIED] = true;
    else
      errorFlags[NODE_ERROR] = errorFlags[FATAL_ERROR] = true;
    return errorFlags;
  }

  // Check whether the node positions can be considered invalid
  // by checking for coincident nodes within the element

  double tolerance = FmDB::getPositionTolerance();
  bool nodePosIsOK = !forceUseID;
  if (!forceUseID)
    for (size_t n = 1; n < num_nodes && nodePosIsOK; n++)
      for (size_t i = 0; i < n && nodePosIsOK; i++)
        if (nodePoss[i].equals(nodePoss[n],tolerance))
          nodePosIsOK = false;

  if (nodePosIsOK)
  {
    // The positions are OK. Go find the nodes closest to them.
    // TODO : Use node ID as preference on what nodes to select.
    // Also check the resulting nodes whether they are OK to use.
    for (size_t n = 0; n < num_nodes; n++)
      if (FFlNode* node = rosetteLink->getClosestNode(nodePoss[n]); node)
      {
        nodeNums[n] = node->getID();
        nodePoss[n] = node->getPos();
      }
    nodeNumsOK = true;
  }
  else if (nodeNumsOK)
  {
    // Positions were not OK. Go find them based on nodeIDs instead.
    for (size_t n = 0; n < num_nodes; n++)
      if (FFlNode* node = rosetteLink->getNode(nodeNums[n]); node)
        nodePoss[n] = node->getPos();
      else
        nodeNumsOK = false;
  }

  if (!nodeNumsOK)
  {
    errorFlags[NODE_ERROR] = errorFlags[FATAL_ERROR] = true;
    return errorFlags;
  }

  // Todo : Check that the nodes found is OK !

  // Check what has changed :

  bool idsHasChanged = false;
  for (size_t n = 0; n < num_nodes; n++)
    if (node[n].setValue(nodeNums[n]))
      idsHasChanged = true;

  bool posHasChanged = forceUseID;
  for (size_t n = 0; n < num_nodes && !posHasChanged; n++)
    posHasChanged = !nodePoss[n].equals(nodePos[n].getValue(),tolerance);

  for (size_t n = 0; n < num_nodes; n++)
    nodePos[n].setValue(nodePoss[n]);

  errorFlags[NODE_IDS_CHANGED] = nodeNumsOK  && idsHasChanged;
  errorFlags[NODE_POS_CHANGED] = nodePosIsOK && posHasChanged;

  if (useFEMaterial.getValue() || useFEThickness.getValue())
  {
    FFlLinkHandler* lh = rosetteLink->getLinkHandler();
    FFlElementBase* el = lh->findClosestElement(this->getCalculationPoint(),
                                                { FFlTypeInfoSpec::SOLID_ELM,
                                                  FFlTypeInfoSpec::SHELL_ELM });
    if (el)
    {
      if (useFEThickness.getValue() &&
          el->getCathegory() == FFlTypeInfoSpec::SHELL_ELM)
      {
        FFlAttributeBase* pThick = el->getAttribute("PTHICK");
        if (pThick)
        {
          double newFEThickness = ((FFlPTHICK*)pThick)->thickness.getValue();
          if (fabs(FEThickness.getValue() - newFEThickness) > 1e-10)
            errorFlags[ELEMENT_THICKNESS_CHANGED] = true;
          FEThickness.setValue(newFEThickness);
        }
      }
      if (useFEMaterial.getValue())
      {
        FFlAttributeBase* pMat = el->getAttribute("PMAT");
        if (pMat)
        {
          double newEmod = ((FFlPMAT*)pMat)->youngsModule.getValue();
          double newNu = ((FFlPMAT*)pMat)->poissonsRatio.getValue();
          if (fabs(EModFE.getValue()-newEmod) > 1.0e-10 ||
              fabs(nuFE.getValue()-newNu) > 1.0e-10)
            errorFlags[ELEMENT_MATERIAL_CHANGED] = true;
          EModFE.setValue(newEmod);
          nuFE.setValue(newNu);
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
  int n3 = node[3].getValue() > 0 ? 3 : 2;
  std::swap(node[1].getValue(),node[n3].getValue());
  std::swap(nodePos[1].getValue(),nodePos[n3].getValue());
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
  fprintf(fd,"&STRAIN_ROSETTE\n");
  this->printID(fd);
  if (!this->rosetteLink.isNull())
    fprintf(fd,"  linkId = %d\n", rosetteLink->getBaseID());

  fprintf(fd,"  type = '%s'\n", rosetteType.getValue().getText());
  fprintf(fd,"  zeroInit = %d\n", (int)removeStartStrains.getValue());

  fprintf(fd,"  numnod = %d\n  nodes =", this->getNoNodes());
  for (int i = 0; i < this->getNoNodes(); i++)
    fprintf(fd," %d", node[i].getValue());
  fprintf(fd,"\n");

  FaMat34 rPos(this->getSymbolPosMx());
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


/*!
  This static method is supposed to be used when you need to read an old
  strain gage input file and convert it to strain rosette objects in the model.
  The supplied file name can be relative to the model file or absolute.
  After this method is invoked, all strain rosettes from the file are read,
  and strain rosette objects are created. They are however not resolved
  (regarding pointer to part) nor syncronized with underlying FE mesh.
*/

bool FmStrainRosette::createRosettesFromOldFile(const std::string& fileName,
                                                bool resetStartStrainValue)
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
        FmStrainRosette* newRosette = new FmStrainRosette();
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
        newRosette->setTopology(NULL, { n1, n2, n3, n4 });

        newRosette->useFEThickness = false;
        newRosette->zPos = zHeight;

        newRosette->angleOriginVector = FaVec3(Xx, Xy, Xz);
        newRosette->tmpZDirection = new FaVec3(Zx, Zy, Zz);

        newRosette->useFEMaterial = false;
        newRosette->EMod = Emod;
        newRosette->nu = nu;
        newRosette->removeStartStrains = resetStartStrainValue;

        newRosette->connect();
        ++nReadStrainRosettes;
      }
    }
  }

  ListUI <<" --> Done. Read "<< nReadStrainRosettes <<" rosettes.\n";
  return true;
}
