// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_THRESHOLD_H
#define FM_THRESHOLD_H

#include "FFaLib/FFaString/FFaEnum.H"

#include <iostream>
#include <string>


/*!
  \brief Class holding threshold settings for digital twin output functions.
  \details The parameters here are not used by the Fedem simulation, they are
  only passed to the digital twin cloud framework through the app.json file
  of the exported app.
*/

class FmThreshold
{
public:
  FmThreshold() : active(UNDEF), threshold(0.0), severity(LOW) { min=skip = 0; }

  bool isDefined() const { return active > UNDEF; }
  bool isActive() const { return active == ACTIVE; }

  void writeAppJson(std::ostream& os,
                    const std::string& tag, const std::string src,
                    size_t indent = 4) const;

  bool operator==(const FmThreshold& a) const;

  enum Active { UNDEF = -1, INACTIVE = 0, ACTIVE = 1 };
  enum Severity { LOW = 1, MEDIUM = 2, HIGH = 3 };

private:
  FFaEnumMapping(Active) {
    FFaEnumEntry(UNDEF, "UNDEFINED");
    FFaEnumEntry(INACTIVE, "INACTIVE");
    FFaEnumEntry(ACTIVE, "ACTIVE");
    FFaEnumEntryEnd;
  };

  FFaEnumMapping(Severity) {
    FFaEnumEntry(LOW, "LOW");
    FFaEnumEntry(MEDIUM, "MEDIUM");
    FFaEnumEntry(HIGH, "HIGH");
    FFaEnumEntryEnd;
  };

public:
  ActiveEnum   active;
  double       threshold;
  int          min, skip;
  SeverityEnum severity;
  std::string  description;
};

std::ostream& operator<<(std::ostream& os, const FmThreshold& fld);
std::istream& operator>>(std::istream& is, FmThreshold& fld);

#endif
