!! SPDX-FileCopyrightText: 2023 SAP SE
!!
!! SPDX-License-Identifier: Apache-2.0
!!
!! This file is part of FEDEM - https://openfedem.org
!!==============================================================================

function chain_shape (N,M,L,dX,dY,X,Y) result(ierr)

  !DEC$ ATTRIBUTES DLLEXPORT :: chain_shape
  !!============================================================================
  !! Global scope function for calculation of chain shapes.
  !! To be used in C++ and/or C# programs.
  !!
  !! Programmed by: Knut Morten Okstad             date/rev: July 04, 2014 / 1.0
  !!============================================================================

  use ChainModel, only : ChainType, calcChainShape

  implicit none

  !! Arguments
  integer, intent(in)  :: N      !< Number of mass points
  real*8 , intent(in)  :: M(N)   !< Point masses
  real*8 , intent(in)  :: L(N+1) !< Lengths between the mass points
  real*8 , intent(in)  :: dX     !< Horizontal distance between the end points
  real*8 , intent(in)  :: dY     !< Vertical distance between the end points
  real*8 , intent(out) :: X(N+2) !< X-coordinates of the mass points, incl. ends
  real*8 , intent(out) :: Y(N+2) !< Y-coordinates of the mass points, incl. ends

  !! Local variables
  integer         :: ierr, np
  real*8, pointer :: XY(:,:)
  type(ChainType) :: chain

  !! --- Logic section ---

  X = 0.0d0
  Y = 0.0d0
  allocate(chain%L(N+1),chain%M(N),stat=ierr)
  if (ierr /= 0) return

  chain%M  = M
  chain%L  = L
  chain%dX = dX
  chain%dY = dY
  call calcChainShape (chain,XY,0,0,ierr)
  if (ierr == 0) then
     np = size(XY,2)
     X(2:1+np) = XY(1,:)
     Y(2:1+np) = XY(2,:)
  end if

  deallocate(chain%L,chain%M,XY)

end function chain_shape


function cable_shape (N,L,dX,dY,X,Y) result(ierr)

  !DEC$ ATTRIBUTES DLLEXPORT :: cable_shape
  !!============================================================================
  !! Global scope function for calculation of uniform cable shapes.
  !! To be used in C++ and/or C# programs.
  !!
  !! Programmed by: Knut Morten Okstad          date/rev: October 09, 2019 / 1.0
  !!============================================================================

  use ChainModel, only : ChainType, calcChainShape

  implicit none

  !! Arguments
  integer, intent(in)  :: N      !< Number of cable segments
  real*8 , intent(in)  :: L      !< Total cable length
  real*8 , intent(in)  :: dX     !< Horizontal distance between the end points
  real*8 , intent(in)  :: dY     !< Vertical distance between the end points
  real*8 , intent(out) :: X(N+1) !< X-coordinates of the mass points, incl. ends
  real*8 , intent(out) :: Y(N+1) !< Y-coordinates of the mass points, incl. ends

  !! Local variables
  integer         :: ierr, np
  real*8, pointer :: XY(:,:)
  type(ChainType) :: chain

  !! --- Logic section ---

  X = 0.0d0
  Y = 0.0d0
  allocate(chain%L(N),chain%M(N-1),stat=ierr)
  if (ierr /= 0) return

  chain%M  = 1.0d0
  chain%L  = L/dble(N)
  chain%dX = dX
  chain%dY = dY
  call calcChainShape (chain,XY,0,0,ierr)
  if (ierr == 0) then
     np = size(XY,2)
     X(2:1+np) = XY(1,:)
     Y(2:1+np) = XY(2,:)
  end if

  deallocate(chain%L,chain%M,XY)

end function cable_shape
