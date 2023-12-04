// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#include "vpmDB/FmSolverConvergence.H"


FmSolverConvergence& FmSolverConvergence::operator=(const FmSolverConvergence& ref)
{
  if (this != &ref)
  {
    value  = ref.value;
    policy = ref.policy;
  }
  return *this;
}


bool FmSolverConvergence::operator==(const FmSolverConvergence& ref) const
{
  if (this == &ref)
    return true;
  else
    return value == ref.value && policy == ref.policy;
}


std::ostream& operator<<(std::ostream& s, const FmSolverConvergence& obj)
{
  return s << obj.value <<" "<< obj.policy;
}


std::istream& operator>>(std::istream& s, FmSolverConvergence& obj)
{
  s >> obj.value >> obj.policy;
  return s;
}
