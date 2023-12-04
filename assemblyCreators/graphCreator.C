// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "turbineConverter.H"
#include "vpmDB/FmGraph.H"
#include "vpmDB/FmCurveSet.H"
#include "FFaLib/FFaDefinitions/FFaResultDescription.H"


bool FWP::createGraphs (FmModelMemberBase* hub,
		        FmModelMemberBase* tower,
		        FmModelMemberBase* generator)
{
  FFaResultDescription curveDesc("Triad",hub->getBaseID());

  // Rotor rotational speed
  curveDesc.varRefType = "ROT3";
  curveDesc.varDescrPath.push_back("Local coordinates");
  curveDesc.varDescrPath.push_back("Angular velocity");
  FmCurveSet* curve = new FmCurveSet();
  FmGraph* graph = new FmGraph();
  graph->setUserDescription("Rotor rotational speed [rpm]");
  graph->connect();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"Z");
  curve->setScaleFactor(1.0,9.549296584);
  curve->onChanged();

  // Rotor torque
  curveDesc.varDescrPath.back() = "Moment";
  curve = new FmCurveSet();
  graph = new FmGraph();
  graph->setUserDescription("Rotor torque");
  graph->connect();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"Z");
  curve->onChanged();

  // Generator speed [rpm]
  curveDesc.OGType = "Revolute joint";
  curveDesc.baseId = generator->getBaseID();
  curveDesc.varRefType = "SCALAR";
  curveDesc.varDescrPath.clear();
  curveDesc.varDescrPath.push_back("Rz joint variables");
  curveDesc.varDescrPath.push_back("Angular velocity");
  curve = new FmCurveSet();
  FmCurveSet* pCurveComb1 = curve; // use later
  graph = new FmGraph();
  graph->setUserDescription("Generator speed [rpm]");
  graph->connect();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"None");
  curve->setScaleFactor(1.0,9.549296584);
  curve->onChanged();

  // Generator moment [kN]
  curveDesc.varDescrPath.back() = "Moment value";
  curve = new FmCurveSet();
  FmCurveSet* pCurveComb2 = curve; // use later
  graph = new FmGraph();
  graph->setUserDescription("Generator moment [kN]");
  graph->connect();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"None");
  curve->setScaleFactor(1.0,0.001);
  curve->onChanged();

  // Generator power [kW]
  curve = new FmCurveSet(FmCurveSet::COMB_CURVES);
  graph = new FmGraph();
  graph->setUserDescription("Generator power [kW]");
  graph->connect();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setCurveComp(pCurveComb1,0);
  curve->setCurveComp(pCurveComb2,1);
  curve->setExpression("-A*B/9.549296584");
  curve->onChanged();

  // Tower top forces and moments
  return FWP::createTriadForceGraph(tower);
}


static void createXYZcurves (FmGraph* graph, const FFaResultDescription& curveDesc)
{
  FmCurveSet* curve = new FmCurveSet();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"X");
  curve->onChanged();

  curve = new FmCurveSet();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"Y");
  curve->onChanged();

  curve = new FmCurveSet();
  graph->addCurveSet(curve);
  curve->setColor(graph->getCurveDefaultColor());
  curve->setResult(FmCurveSet::YAXIS,curveDesc);
  curve->setResultOper(FmCurveSet::YAXIS,"Z");
  curve->onChanged();
}


bool FWP::createTriadForceGraph (FmModelMemberBase* triad)
{
  FFaResultDescription curveDesc("Triad",triad->getBaseID());

  // Triad forces
  curveDesc.varRefType = "VEC3";
  curveDesc.varDescrPath.clear();
  curveDesc.varDescrPath.push_back("Force");

  std::string descr = triad->getUserDescription();
  if (descr.empty()) descr = triad->getIdString();

  FmGraph* graph = new FmGraph();
  graph->setUserDescription(descr + " forces");
  graph->connect();

  createXYZcurves(graph,curveDesc);

  // Triad moments
  curveDesc.varRefType = "ROT3";
  curveDesc.varDescrPath.back() = "Moment";
  graph = new FmGraph();
  graph->setUserDescription(descr + " moments");
  graph->connect();

  createXYZcurves(graph,curveDesc);

  return true;
}


bool FWP::createBeamForceGraph (FmModelMemberBase* beam, int iend)
{
  FFaResultDescription curveDesc("Beam",beam->getBaseID());

  // Beam end forces
  curveDesc.varRefType = "VEC3";
  curveDesc.varDescrPath.clear();
  curveDesc.varDescrPath.push_back("Sectional force, end 1");
  if (iend > 1) *curveDesc.varDescrPath.back().rbegin() = '2';

  std::string descr = beam->getUserDescription();
  if (descr.empty()) descr = beam->getIdString();

  FmGraph* graph = new FmGraph();
  graph->setUserDescription(descr + " forces");
  graph->connect();

  createXYZcurves(graph,curveDesc);

  // Beam end moments
  curveDesc.varRefType = "ROT3";
  curveDesc.varDescrPath.back().replace(10,5,"moment");
  graph = new FmGraph();
  graph->setUserDescription(descr + " moments");
  graph->connect();

  createXYZcurves(graph,curveDesc);

  return true;
}
