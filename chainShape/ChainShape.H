// SPDX-FileCopyrightText: 2023 SAP SE
//
// SPDX-License-Identifier: Apache-2.0
//
// This file is part of FEDEM - https://openfedem.org
////////////////////////////////////////////////////////////////////////////////

#ifndef CHAIN_SHAPE_H
#define CHAIN_SHAPE_H

#include "FFaLib/FFaOS/FFaFortran.H"
#include <vector>


INTEGER_FUNCTION (chain_shape,CHAIN_SHAPE) (const int& n,
                                            const double* M,
                                            const double* L,
                                            const double& dX,
                                            const double& dY,
                                            double* X, double* Y);

INTEGER_FUNCTION (cable_shape,CABLE_SHAPE) (const int& n,
                                            const double& L,
                                            const double& dX,
                                            const double& dY,
                                            double* X, double* Y);


int getChainShape (const std::vector<double>& M,
                   const std::vector<double>& L, double dX, double dY,
                   std::vector<double>& X, std::vector<double>& Y)
{
#ifdef FT_HAVE_CHAINSHAPE
  return F90_NAME(chain_shape,CHAIN_SHAPE) (M.size(),M.data(),L.data(),
                                            dX,dY,X.data(),Y.data());
#else
  std::cerr <<" *** ChainShape module is not available "
            << M.size() <<" "<< L.size() <<" "<< dX <<" "<< dY
            << X.size() <<" "<< Y.size() << std::endl;
  return -99;
#endif
}


int getCableShape (int n, double L, double dX, double dY,
                   std::vector<double>& X, std::vector<double>& Y)
{
#ifdef FT_HAVE_CHAINSHAPE
  return F90_NAME(cable_shape,CABLE_SHAPE) (n,L,dX,dY,X.data(),Y.data());
#else
  std::cerr <<" *** ChainShape module is not available "
            << n <<" "<< L <<" "<< dX <<" "<< dY <<" "
            << X.size() <<" "<< Y.size() << std::endl;
  return -99;
#endif
}

#endif
