// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_ARRAY_H
#define FM_ARRAY_H

#include <array>
#include <iostream>


template<size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<int,N>& arr)
{
  os << arr.front();
  for (size_t i = 1; i < N; i++) os <<" "<< arr[i];
  return os;
}

template<size_t N>
std::istream& operator>>(std::istream& is, std::array<int,N>& arr)
{
  std::array<int,N> tmp;
  for (int& i : tmp) is >> i;
  if (is) arr = tmp;
  return is;
}

#endif
