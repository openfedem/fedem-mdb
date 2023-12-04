// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "FFaLib/FFaString/FFaParse.H"

#ifdef USE_INVENTOR
#include "vpmDisplay/FdCtrlElement.H"
#include "vpmDisplay/FdCtrlLine.H"
#endif

#include "vpmDB/FmcInput.H"
#include "vpmDB/FmcOutput.H"
#include "vpmDB/FmCtrlLine.H"

#include <utility>


/**********************************************************************
 *
 * Class constructor and destructor
 *
 **********************************************************************/

Fmd_DB_SOURCE_INIT(FccLINE, FmCtrlLine, FmIsPlottedBase);


FmCtrlLine::FmCtrlLine()
{
  Fmd_CONSTRUCTOR_INIT(FmCtrlLine);

  FFA_FIELD_INIT(myFirstLineVertical, false, "FIRST_LINE_VERTICAL");
  FFA_FIELD_INIT(myFirstUndefLine,    1,     "FIRST_UNDEF_LINE");
  FFA_FIELD_INIT(mySolverVar,         0,     "CONTROL_VAR_NO");
  FFA_FIELD_DEFAULT_INIT(mySegmentLengths,   "SEGMENT_LENGTHS");

  FFA_REFERENCE_FIELD_INIT(myStartCtrlBlockField, myStartCtrlBlock, "OWNER_START");
  FFA_FIELD_INIT(myEndCtrlBlockField,
                 FmBlockPortReference(&myEndCtrlBlock,this), "OWNER_END");
  FFA_REFERENCE_INIT(myEndCtrlBlock);

#ifdef USE_INVENTOR
  itsDisplayPt = new FdCtrlLine(this);
#endif
}


FmCtrlLine::~FmCtrlLine()
{
  this->disconnect();
}


/**********************************************************************
 *
 * other class methods
 *
 **********************************************************************/

int FmCtrlLine::getEndPort() const
{
  return myEndCtrlBlock.isNull() ? -1 : myEndCtrlBlock->atWhatPort(this);
}


bool FmCtrlLine::setStartElement(FmCtrlElementBase* elm)
{
  if (elm && elm->isOfType(FmcOutput::getClassTypeID()))
    return false;

  myStartCtrlBlock.setRef(dynamic_cast<FmCtrlOutputElementBase*>(elm));
  return true;
}


bool FmCtrlLine::setEndElement(FmCtrlElementBase* elm)
{
  if (elm && elm->isOfType(FmcInput::getClassTypeID()))
    return false;

  myEndCtrlBlock.setRef(elm);
  return true;
}


std::ostream& FmCtrlLine::writeFMF(std::ostream& os)
{
  os <<"CONTROL_LINE\n{\n";
  this->writeFields(os);
  os <<"}\n\n";
  return os;
}


void FmCtrlLine::initAfterResolve()
{
  this->FmIsRenderedBase::initAfterResolve();

  myEndCtrlBlock->setLine(myEndCtrlBlockField.getValue().portNo, this);
}


bool FmCtrlLine::disconnect()
{
  myStartCtrlBlock.setPointerToNull();

  if (!myEndCtrlBlock.isNull())
    myEndCtrlBlock->releaseFromPort(this);

  return this->mainDisconnect();
}


bool FmCtrlLine::clone(FmBase* obj, int depth)
{
  return cloneInherited(obj, depth);
}


bool FmCtrlLine::cloneLocal(FmBase* obj, int)
{
  return obj->isOfType(FmCtrlLine::getClassTypeID());
}


/***********************************************************************
*
* Input and output from stream
*
************************************************************************/

bool FmCtrlLine::readAndConnect(std::istream& is, std::ostream&)
{
  FmCtrlLine* obj = new FmCtrlLine();

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


inline Vec2 operator+(const Vec2& P, const Vec2& Q) { return { P.x+Q.x, P.y+Q.y }; }
inline Vec2 operator-(const Vec2& P) { return { -P.x, -P.y }; }


static Vec2 inPortCorrection(FmCtrlElementBase* elm, int port)
{
  Vec2 newVec({-0.75f,0.0f});

  switch (port + (elm->getNumInputPorts() == 1 ? 1 : 2))
    {
    case 2: // One inport
      break;
    case 3: // First of two inports
      newVec.y =  0.25f;
      break;
    case 4: // Second of two inports
      newVec.y = -0.25f;
      break;
    }

  return elm->isLeftRotated() ? -newVec : newVec;
}


static Vec2 outPortCorrection(FmCtrlElementBase* elm)
{
  return { elm->isLeftRotated() ? -0.75f : 0.75f, 0.0f };
}


//Calculates the line coordinatematrix
/////////////////////////////////////////

void FmCtrlLine::getCoordMatrix(std::vector<Vec2>& coordMat) const
{
  int numLines = this->getNumberOfSegments();
  coordMat.resize(numLines+1);

  //Sets the first coordinate.
  coordMat.front() = myStartCtrlBlock->getPosition() +
    outPortCorrection(myStartCtrlBlock.getPointer());

  //Sets the last coordinate.
  coordMat.back() = myEndCtrlBlock->getPosition() +
    inPortCorrection(myEndCtrlBlock.getPointer(),myEndCtrlBlock->atWhatPort(this));

  //Calculate the breakpoints in front of the to undef. linesegments.
  bool vertical = myFirstLineVertical.getValue();
  for (int j = 1; j < myFirstUndefLine.getValue(); j++)
  {
    coordMat[j] = coordMat[j-1];
    if (vertical)
      coordMat[j].y += mySegmentLengths.getValue()[j-1];
    else
      coordMat[j].x += mySegmentLengths.getValue()[j-1];

    vertical = !vertical; // Make next line segment vertical if the current was horizontal
  }

  //Calculate the breakpoints behind the to undef. linesegments. Starting from the endpoint.
  bool odd = numLines%2; // If odd is false the number of lines is even
  vertical = myFirstLineVertical.getValue() == odd; // The last element is vertical?
  for (int j = numLines-1; j > myFirstUndefLine.getValue(); j--)
  {
    coordMat[j] = coordMat[j+1];
    if (vertical)
      coordMat[j].y -= mySegmentLengths.getValue()[j-2];
    else
      coordMat[j].x -= mySegmentLengths.getValue()[j-2];

    vertical = !vertical; // Make next line segment vertical if the current was horizontal
  }

  //Calculate the undef. breakpoint.
  vertical = myFirstLineVertical.getValue();
  odd = myFirstUndefLine.getValue()%2;
  int k = odd == vertical ? -1 : 1;

  coordMat[myFirstUndefLine.getValue()] = { coordMat[myFirstUndefLine.getValue()+k].x,
                                            coordMat[myFirstUndefLine.getValue()-k].y };
}


//Calculate a length array with all line segments
///////////////////////////////////////////////////

DoubleVec FmCtrlLine::getTotLengthArray() const
{
  std::vector<Vec2> coordMatrix;
  this->getCoordMatrix(coordMatrix);

  int numLines = this->getNumberOfSegments();
  DoubleVec totLengthArray(numLines,0);

  for (int i = 0; i < numLines; i++)
  {
    float x1 = coordMatrix[i  ].x;
    float x2 = coordMatrix[i+1].x;
    float y1 = coordMatrix[i  ].y;
    float y2 = coordMatrix[i+1].y;

    if (x1 != x2)
      totLengthArray[i] = x2-x1;
    else if (y1 != y2)
      totLengthArray[i] = y2-y1;
  }

  return totLengthArray;
}


void FmCtrlLine::setInitialLineData(FmCtrlElementBase* start,
                                    FmCtrlElementBase* end, int portNr)
{
  // Rules for how the line going to be drawn,
  // depending of the element rotation and location
  bool startLeftRot = start->isLeftRotated();
  bool endLeftRot = end->isLeftRotated();
  bool is1stVertical = false;
  int  firstUndefSegment = 0;

  // Sets start and end points for the line.
  Vec2 startVec = start->getPosition() + outPortCorrection(start);
  Vec2 endVec   = end->getPosition() + inPortCorrection(end,portNr);
  DoubleVec lineArray;

  if (!startLeftRot && !endLeftRot)
  {
    firstUndefSegment = 2;
    is1stVertical = startVec.x >= endVec.x;
    lineArray = { 0.5*(is1stVertical ? endVec.y-startVec.y : endVec.x-startVec.x) };
  }
  else if (startLeftRot && endLeftRot)
  {
    firstUndefSegment = 2;
    is1stVertical = startVec.x <= endVec.x;
    lineArray = { 0.5*(is1stVertical ? endVec.y-startVec.y : endVec.x-startVec.x) };
  }
  else if (!startLeftRot && endLeftRot)
  {
    firstUndefSegment = 1;
    is1stVertical = startVec.x > endVec.x;
  }
  else if (startLeftRot && !endLeftRot)
  {
    firstUndefSegment = 1;
    is1stVertical = startVec.x <= endVec.x;
  }

  myFirstLineVertical.setValue(is1stVertical);
  myFirstUndefLine.setValue(firstUndefSegment);
  mySegmentLengths.setValue(lineArray);
}
