// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_1D_MASTER_H
#define FM_1D_MASTER_H

#include "vpmDB/FmIsRenderedBase.H"
#include "vpmDB/FmTriad.H"
#include "FFaLib/FFaContainers/FFaReferenceList.H"


class Fm1DMaster : public FmIsRenderedBase
{
  Fmd_DB_HEADER_INIT();

public:
  virtual bool detach();

  virtual void initAfterResolve();

  virtual bool isListable() const { return false; }

  virtual bool addTriadOnPoint(const FaVec3& globalPoint) = 0;
  virtual bool addTriad(FmTriad* triad, bool asFront = false);
  virtual bool insertTriad(FmTriad* triad, size_t pos = 0);

  virtual int  releaseTriad(FmTriad* triad, FmTriad* replacement = NULL);
  virtual void releaseTriads(bool eraseUnused = false);

  void setTriads(const std::vector<int>& triadIDs);
  bool setTriads(const std::vector<FmTriad*>& triads);
  void getTriads(std::vector<FmTriad*>& trs) const { myTriads.getPtrs(trs); }
  FmTriad* getFirstTriad() const;
  FmTriad* getLastTriad() const;

  size_t size() const { return myTriads.size(); }
  bool hasTriad(const FmTriad* triad) const { return myTriads.hasPtr(triad); }

  void reverseTriads() { myTriads.reverse(); }

  FmLink* getOwnerLink() const;
  FmPart* getOwnerPart() const;
  bool isAttached(bool allowMultipleLinks) const;

  virtual double getSliderPosition(FaMat34& ur, const FaVec3& pos) const = 0;

protected:
  Fm1DMaster();
  virtual ~Fm1DMaster();

  void printForSolver(FILE* fd, const std::vector<FmTriad*>& triads,
                      double loopLength = 0.0);

  void reshuffleTriads(size_t newFirst) { myTriads.shuffle(newFirst); }
  FmTriad* getTriad(size_t pos) const { return myTriads.getPtr(pos); }

  virtual bool cloneLocal(FmBase* obj, int depth);
  static  bool localParse(const char* keyWord, std::istream& activeStatement,
			  Fm1DMaster* obj);

  virtual bool eraseOptions();

private:
  friend bool operator==(const Fm1DMaster& lhs, const Fm1DMaster& rhs);

  FFaReferenceList<FmTriad>       myTriads;
  FFaField<FFaReferenceListBase*> myTriadsField;
};

#endif
