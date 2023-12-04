// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_CAM_FRICTION_H
#define FM_CAM_FRICTION_H

#include "vpmDB/FmTransFriction.H"


class FmCamFriction : public FmTransFriction
{
  Fmd_DB_HEADER_INIT();

public:
  FmCamFriction();

  static bool readAndConnect(std::istream& is, std::ostream& os = std::cout);
  virtual std::ostream& writeFMF(std::ostream& os = std::cout);

  virtual const char* getUITypeName() const { return "Cam friction"; }

protected:
  virtual ~FmCamFriction() {}

  virtual const char* getFrictionFsiName() const { return "CAM_FRICTION"; }
};

#endif
