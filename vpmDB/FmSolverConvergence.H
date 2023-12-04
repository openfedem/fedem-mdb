// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef FM_SOLVER_CONVERGENCE_H
#define FM_SOLVER_CONVERGENCE_H

#include "FFaLib/FFaString/FFaEnum.H"


class FmSolverConvergence
{
public:
  enum FmSolverConvPolicy {
    CONV_IGNORE = 0,
    CONV_ONE_OF = 1,
    CONV_ALL_OF = 2
  };

  FFaEnumMapping(FmSolverConvPolicy) {
    FFaEnumEntry(CONV_IGNORE,"IGNORE");
    FFaEnumEntry(CONV_ONE_OF,"ONE_OF");
    FFaEnumEntry(CONV_ALL_OF,"ALL_OF");
    FFaEnumEntryEnd;
  };

  FmSolverConvPolicyEnum policy;
  double                 value;

  FmSolverConvergence(double v = 0.0, FmSolverConvPolicy p = CONV_IGNORE) : policy(p), value(v) {}
  FmSolverConvergence(double val, int pol) : value(val) { policy = pol; }

  FmSolverConvergence& operator=(const FmSolverConvergence& ref);
  bool operator==(const FmSolverConvergence& ref) const;
};

std::ostream& operator<<(std::ostream& s, const FmSolverConvergence& obj);
std::istream& operator>>(std::istream& s, FmSolverConvergence& obj);

#endif
