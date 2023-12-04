// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmControlAdmin.H"
#include "vpmDB/FmAllControlHeaders.H"
#include "vpmDB/FmEngine.H"
#include "vpmDB/FmSimpleSensor.H"
#include "vpmDB/FmDB.H"
#include "FFaLib/FFaDefinitions/FFaMsg.H"
#include "FFaLib/FFaString/FFaStringExt.H"


FmCtrlElementBase* FmControlAdmin::createElement(int type)
{
  if      (type == Fmc1ordTF::getClassTypeID())
    return new Fmc1ordTF();
  else if (type == Fmc2ordTF::getClassTypeID())
    return new Fmc2ordTF();
  else if (type == FmcAdder::getClassTypeID())
    return new FmcAdder();
  else if (type == FmcAmplifier::getClassTypeID())
    return new FmcAmplifier();
  else if (type == FmcPower::getClassTypeID())
    return new FmcPower();
  else if (type == FmcCompConjPole::getClassTypeID())
    return new FmcCompConjPole();
  else if (type == FmcComparator::getClassTypeID())
    return new FmcComparator();
  else if (type == FmcDeadZone::getClassTypeID())
    return new FmcDeadZone();
  else if (type == FmcHysteresis::getClassTypeID())
    return new FmcHysteresis();
  else if (type == FmcInput::getClassTypeID())
    return new FmcInput();
  else if (type == FmcIntegrator::getClassTypeID())
    return new FmcIntegrator();
  else if (type == FmcLimDerivator::getClassTypeID())
    return new FmcLimDerivator();
  else if (type == FmcLimitation::getClassTypeID())
    return new FmcLimitation();
  else if (type == FmcLogicalSwitch::getClassTypeID())
    return new FmcLogicalSwitch();
  else if (type == FmcMultiplier::getClassTypeID())
    return new FmcMultiplier();
  else if (type == FmcOutput::getClassTypeID())
    return new FmcOutput();
  else if (type == FmcPIlimD::getClassTypeID())
    return new FmcPIlimD();
  else if (type == FmcPd::getClassTypeID())
    return new FmcPd();
  else if (type == FmcPi::getClassTypeID())
    return new FmcPi();
  else if (type == FmcPid::getClassTypeID())
    return new FmcPid();
  else if (type == FmcPlimD::getClassTypeID())
    return new FmcPlimD();
  else if (type == FmcPlimI::getClassTypeID())
    return new FmcPlimI();
  else if (type == FmcPlimIlimD::getClassTypeID())
    return new FmcPlimIlimD();
  else if (type == FmcRealPole::getClassTypeID())
    return new FmcRealPole();
  else if (type == FmcSampleHold::getClassTypeID())
    return new FmcSampleHold();
  else if (type == FmcTimeDelay::getClassTypeID())
    return new FmcTimeDelay();

  return NULL;
}


FmCtrlElementBase* FmControlAdmin::createElement(int type, float x, float y)
{
  FmCtrlElementBase* retElem = FmControlAdmin::createElement(type);
  if (!retElem) return NULL;

  retElem->setPosition(x,y);
  retElem->connect();

  if (retElem->isOfType(FmcOutput::getClassTypeID()))
  {
    FmEngine* e = new FmEngine();
    e->setSensor(retElem->getSimpleSensor(true));
    e->setUserDescription(retElem->getUITypeName() + FFaNumStr(" [%d]",retElem->getID()));
    e->connect();
  }
  else if (retElem->isOfType(FmcInput::getClassTypeID()))
  {
    FmEngine* e = new FmEngine();
    e->setUserDescription(retElem->getUITypeName() + FFaNumStr(" [%d]",retElem->getID()));
    static_cast<FmcInput*>(retElem)->setEngine(e);
    e->connect();
  }
  retElem->draw();

  return retElem;
}


FmCtrlLine* FmControlAdmin::createLine(FmCtrlElementBase* start,
                                       FmCtrlElementBase* end, int portNo)
{
  if (!start || !end)
    return NULL;

  FmCtrlLine* l1 = new FmCtrlLine();
  l1->connect();

  if (!l1->setStartElement(start) || !end->setLine(portNo,l1))
  {
    l1->erase();
    return NULL;
  }

  l1->setInitialLineData(start,end,portNo);
  l1->draw();

  return l1;
}


int FmControlAdmin::checkControl()
{
  int errCount = 0;

  // check all elements
  std::vector<FmCtrlInputElementBase*> allElements;
  FmDB::getAllControlElements(allElements);
  for (FmCtrlInputElementBase* activeElement : allElements)
  {
    // check input element lines
    for (int j = 1; j <= activeElement->getNumInputPorts(); j++)
      if (!activeElement->getLine(j))
      {
        errCount++;
        ListUI <<"\n---> CONTROL SYSTEM ERROR: Port "<< j
               <<" in element "<< activeElement->getID()
               <<" ("<< activeElement->getUITypeName()
               <<") is not connected.\n";
      }

    // check output element lines
    if (!activeElement->hasCtrlLines())
    {
      errCount++;
      ListUI <<"\n---> CONTROL SYSTEM ERROR: Output port in element "
             << activeElement->getID() <<" ("<< activeElement->getUITypeName()
             <<") is not connected.\n";
    }
  }

  // check input elements
  std::vector<FmcInput*> allInputs;
  FmDB::getAllControlInput(allInputs);
  for (FmcInput* activeInput : allInputs)
  {
    // check engine
    if (!activeInput->getEngine())
    {
      errCount++;
      ListUI <<"\n---> CONTROL SYSTEM ERROR: No input specified for element "
             << activeInput->getID() <<".\n";
    }

    // check output element lines
    if (!activeInput->hasCtrlLines())
    {
      errCount++;
      ListUI <<"\n---> CONTROL SYSTEM ERROR: Output port in element "
             << activeInput->getID() <<" is not connected.\n";
    }
  }

  // check output elements
  std::vector<FmcOutput*> allOutputs;
  FmDB::getAllControlOutput(allOutputs);
  for (FmcOutput* activeOutput : allOutputs)
    if (!activeOutput->getLine())
    {
      errCount++;
      ListUI <<"\n---> CONTROL SYSTEM ERROR: Input port in element "
             << activeOutput->getID() <<" is not connected.\n";
    }

  return errCount;
}


int FmControlAdmin::printControl(FILE* fp, int& baseId)
{
  std::vector<FmcInput*>                allInputs;
  std::vector<FmCtrlInputElementBase*>  allElements;
  std::vector<FmCtrlOutputElementBase*> allCtrl;

  // Renumber the control variables
  FmDB::getAllControlInput(allInputs);
  FmDB::getAllControlElements(allElements);
  allCtrl.reserve(allInputs.size()+allElements.size());
  for (FmcInput* inp : allInputs)
    allCtrl.push_back(inp);
  for (FmCtrlInputElementBase* elm : allElements)
    allCtrl.push_back(elm);

  int runner = 1;
  for (FmCtrlOutputElementBase* ctrl : allCtrl)
    runner = ctrl->renumberLocalVariables(runner);

  // Total number of control variables (including the internal ones)
  int nCtrlVar = runner-1;
  if (nCtrlVar < 1) return 0;

  int errCount = 0;
  std::set<int> nonInt;
  std::vector<FmCtrlLine*> lines;

  // Write external control variables

  fprintf(fp,"! Control lines (non-internal control variables)\n");

  for (size_t i = 0; i < allCtrl.size(); i++)
  {
    allCtrl[i]->getLines(lines);
    if (lines.empty())
      errCount++;
    else for (size_t j = 0; j < lines.size(); j++)
    {
      runner = lines[j]->getControlVarNo();
      if (j == 0) nonInt.insert(runner);
      fprintf(fp, "&CONTROL_%s\n", j == 0 ? "VARIABLE" : "LINE");
      lines[j]->printID(fp);
      fprintf(fp,"  iVar = %d\n", runner);
      if (j == 0 && i < allInputs.size())
        fprintf(fp,"  status = 1\n");
      fprintf(fp,"/\n\n");
    }
  }

  if (nonInt.size() < (size_t)nCtrlVar)
  {
    // Also write the internal control variables (needed in restart)
    fprintf(fp,"! Internal control variables\n");

    // The internal variables do not have a corresponding DB object,
    // so we must assign a unique baseID to each of them (and which doesn't
    // conflict with the other objects) so that the solver can write a
    // consistent frs-file to be used in restart runs. The internal variables
    // must all have a zero user ID (to distinguish them from the others).
    std::set<int>::const_iterator vit = nonInt.begin();
    for (runner = 1; runner <= nCtrlVar; runner++)
      if (runner < *vit)
      {
        fprintf(fp, "&CONTROL_VARIABLE\n");
        fprintf(fp,"  id = %d\n", baseId++);
        fprintf(fp,"  iVar = %d\n/\n\n", runner);
      }
      else
        ++vit;
  }

  // Now write the control element definitions
  for (FmCtrlOutputElementBase* ctrl : allCtrl)
    errCount += ctrl->printSolverEntry(fp);

  return errCount;
}
