// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_MODEL_MEMBER_CONNECTOR_H
#define FM_MODEL_MEMBER_CONNECTOR_H

#include "vpmDB/FmModelMemberBase.H"
#include "FFaLib/FFaDynCalls/FFaSwitchBoard.H"
#include "FFaLib/FFaPatterns/FFaSingelton.H"


class FmSignalConnector : public FFaSingelton<FFaSwitchBoardConnector>
{
protected:
  FmSignalConnector() {}
  virtual ~FmSignalConnector() {}
};


template <class T>
class FmModelMemberChangedReceiver : public FFaSwitchBoardConnector
{
  T* myOwner;

public:
  FmModelMemberChangedReceiver(T* owner) : myOwner(owner)
  {
    FFaSwitchBoard::connect(FmSignalConnector::instance(),
                            FmModelMemberBase::MODEL_MEMBER_CHANGED,
                            new FFaSlot1<FmModelMemberChangedReceiver<T>,FmModelMemberBase*>
                            (this,&FmModelMemberChangedReceiver<T>::onChanged));
  }
  virtual ~FmModelMemberChangedReceiver() {}

private:
  void onChanged(FmModelMemberBase* obj) { myOwner->onModelMemberChanged(obj); }
};

#endif
