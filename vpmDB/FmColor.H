// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_COLOR_H
#define FM_COLOR_H

#include <array>
#include <iostream>

#define I3 std::array<int,3>
#define F3 std::array<float,3>
#define FF 255.0f


/*!
  \brief Representation of a RGB color by floats in range [0.0,1.0].
*/

class FmColor : public F3
{
public:
  //! \brief Default constructor.
  FmColor(float r = 0.0f, float g = 0.0f, float b = 0.0f) : F3({r,g,b}) {}
  //! \brief Copy constructor.
  FmColor(const F3& c) : F3(c) {}
  //! \brief Conversion from integer representation in range [0,255]
  FmColor(const I3& c) : F3({c[0]/FF,c[1]/FF,c[2]/FF}) {}
};


inline std::ostream& operator<<(std::ostream& os, const FmColor& color)
{
  std::streamsize oldPrec = os.precision(7);
  os << color[0] << ' ' << color[1] << ' ' << color[2];
  os.precision(oldPrec);
  return os;
}

inline std::istream& operator>>(std::istream& is, FmColor& color)
{
  FmColor tmp;
  is >> tmp[0] >> tmp[1] >> tmp[2];
  if (is) color = tmp;
  return is;
}

#undef I3
#undef F3
#undef FF
#endif
