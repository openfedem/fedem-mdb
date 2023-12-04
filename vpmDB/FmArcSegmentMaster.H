// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ARC_SEGMENT_MASTER_H
#define FM_ARC_SEGMENT_MASTER_H

#include "vpmDB/Fm1DMaster.H"
#include "FFaLib/FFaAlgebra/FFaMat34.H"
#include <array>

typedef std::array<FaVec3,4> CurveSegment;


class FmArcSegmentMaster : public Fm1DMaster
{
  Fmd_DB_HEADER_INIT();

  struct CurveSection
  {
    double  curvature;
    double  slideValue[2];
    FaMat34 ur[2];
    FaVec3  radVec;
    CurveSection() { curvature = slideValue[0] = slideValue[1] = 0.0; }
  };

public:
  FmArcSegmentMaster();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);
  virtual int printSolverEntry(FILE* fd);
  virtual bool clone(FmBase* obj, int depth);
  virtual void initAfterResolve();
  virtual const char* getUITypeName() const { return "Arc segments"; }

  virtual bool addTriadOnPoint(const FaVec3& globalPoint);
  virtual bool addTriad(FmTriad* triad, bool asFront = false);
  virtual int  releaseTriad(FmTriad* triad, FmTriad* replacement = NULL);
  virtual void releaseTriads(bool eraseUnused = false);

  bool isLooping() const { return myLoopFlag.getValue(); }
  void setLooping(bool loop = true) { myLoopFlag.setValue(loop); }

  void getCurveSegments(std::vector<CurveSegment>& seg) const;
  void setDefaultOrientation(FmTriad* follower);

  virtual double getSliderPosition(FaMat34& ur, const FaVec3& pos) const;

private:
  bool isAddable(FmTriad* triad) const;
  void getCurveSections(std::vector<CurveSection>& sec) const;

protected:
  virtual ~FmArcSegmentMaster() {}

  virtual bool cloneLocal(FmBase* obj, int depth);

private:
  FFaField<bool> myLoopFlag;
};

#endif
