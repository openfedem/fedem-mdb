// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FMVTFTYPE_H
#define FMVTFTYPE_H

#include "FFaLib/FFaString/FFaEnum.H"

enum VTFFileType {
  VTF_EXPRESS,
  VTF_BINARY,
  VTF_ASCII
};

FFaEnumMapping(VTFFileType) {
  FFaEnumEntry(VTF_EXPRESS,"EXPRESS");
  FFaEnumEntry(VTF_BINARY, "BINARY");
  FFaEnumEntry(VTF_ASCII,  "ASCII");
  FFaEnumEntryEnd;
};

#endif
