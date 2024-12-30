// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef ASSEMBLY_CREATORS_H
#define ASSEMBLY_CREATORS_H

#include <string>

#ifdef FF_NAMESPACE
namespace FF_NAMESPACE { class FFlLinkHandler; }
using FFlLinkHandler = FF_NAMESPACE::FFlLinkHandler;
#else
class FFlLinkHandler;
#endif


namespace FWP
{
  //! \brief Expands an FE part with beam elements into a spaceframe assembly.
  void createJacket(const FFlLinkHandler* jl, const std::string& name,
		    const double* Morison, int IDoffset);
}

#endif
