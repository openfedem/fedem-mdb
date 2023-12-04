// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CTRL_LINE_H
#define FM_CTRL_LINE_H

#include "vpmDB/FmIsPlottedBase.H"
#include "vpmDB/FmCtrlOutputElementBase.H"
#include "FFaLib/FFaContainers/FFaReference.H"

class FmCtrlLine;


/*!
  This struct is making output of a FFaReference together with
  the port number possible. Made to keep the compatibility in
  the model file when converting all references to FFaReferences.
*/

struct FmBlockPortReference
{
  FFaReferenceBase* block; //!< Pointer to the real FFaReference of the control element
  int portNo;              //!< The port number, used only when reading from file and initAfterResolve()
  FmCtrlLine* line;        //!< The ctrline this is embedded in. To make it possible to get the endPort
  FmBlockPortReference(FFaReferenceBase* b = NULL, FmCtrlLine* l = NULL)
  : block(b), portNo(0), line(l) {}
};

inline bool operator==(const FmBlockPortReference&, const FmBlockPortReference&)
{ return false; }


class FmCtrlLine : public FmIsPlottedBase
{
  Fmd_DB_HEADER_INIT();

public:
  FmCtrlLine();

  virtual bool disconnect();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual bool clone(FmBase* obj, int depth);

  virtual const char* getUITypeName() const { return "CtrlLine"; }

  void setLengthArray(const DoubleVec& array) { mySegmentLengths.setValue(array); }
  const DoubleVec& getLengthArray() const { return mySegmentLengths.getValue(); }

  int getEndPort() const;

  bool setStartElement(FmCtrlElementBase* elm);
  bool setEndElement(FmCtrlElementBase* elm);

  FmCtrlOutputElementBase* getStartElement() const { return myStartCtrlBlock.getPointer(); }
  FmCtrlElementBase*       getEndElement() const   { return myEndCtrlBlock.getPointer(); }

  void setControlVarNo(int varNo) { mySolverVar.setValue(varNo); }
  int  getControlVarNo() const { return mySolverVar.getValue(); }

  bool isFirstLineVertical() const { return myFirstLineVertical.getValue(); }

  void setFirstUndefSegment(int lineNr) { myFirstUndefLine.setValue(lineNr); }
  int  getFirstUndefSegment() const { return myFirstUndefLine.getValue(); }
  int  getNumberOfSegments() const { return mySegmentLengths.getValue().size()+2; }

  void getCoordMatrix(std::vector<Vec2>& coordMatrix) const;
  DoubleVec getTotLengthArray() const;

  void setInitialLineData(FmCtrlElementBase* start,
                          FmCtrlElementBase* end, int portNr);

protected:
  virtual ~FmCtrlLine();

  virtual void initAfterResolve();
  virtual bool cloneLocal(FmBase* obj, int depth);

  FFaReference<FmCtrlOutputElementBase> myStartCtrlBlock;
  FFaField<FFaReferenceBase*>           myStartCtrlBlockField;
  FFaReference<FmCtrlElementBase>       myEndCtrlBlock;
  FFaNoCopyField<FmBlockPortReference>  myEndCtrlBlockField;

  FFaField<int>       mySolverVar;
  FFaField<DoubleVec> mySegmentLengths;
  FFaField<bool>      myFirstLineVertical;
  FFaField<int>       myFirstUndefLine;
};


/*!
  Specialized IO methods for FFaField.
*/

template<> inline void
FFaField<FmBlockPortReference>::write(std::ostream& os) const
{
  myData.block->write(os);
  os <<" "<< myData.line->getEndPort();
}

template<> inline void
FFaField<FmBlockPortReference>::read(std::istream& is, FmBlockPortReference& v) const
{
  v.block->read(is);
  is >> v.portNo;
}

#endif
