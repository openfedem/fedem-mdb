!! SPDX-FileCopyrightText: 2023 SAP SE
!!
!! SPDX-License-Identifier: Apache-2.0
!!
!! This file is part of FEDEM - https://openfedem.org
!!==============================================================================

subroutine FCN (n, x, Fvec, Fjac, ldFjac, iflag)

  !!============================================================================
  !! Evaluates the current residual and Jacobian of the nonlinear function F(x),
  !! defining the equilibrium of a set of string-connected point masses.
  !!
  !! Call-back invoked by HYBRJ.
  !!============================================================================

  use ChainModel, only : myChain, lpUnit

  implicit none

  !! Arguments
  integer, intent(in)  :: n, ldFjac, iflag
  real*8 , intent(in)  :: x(n)
  real*8 , intent(out) :: Fvec(n), Fjac(ldFjac,n)

  !! Local variables
  integer :: i
  real*8  :: d, ld, ld3

  !! --- Logic section ---

  select case (iflag)
  case (0) ! Print for current iteration

     write(lpUnit,600) 'X',x
     write(lpUnit,600) 'F',FVec
600  format (A1,':',1P6E12.4 / (2X,1P6E12.4))

  case (1) ! Evaluate the residual

     do i = 1, n-2
        Fvec(i) = x(i) - x(i+1) - myChain%m(i)
     end do
     Fvec(n-1) = -myChain%dY
     Fvec(n)   = -myChain%dX
     do i = 1, n-1
        ld = myChain%l(i)/sqrt(x(i)*x(i)+x(n)*x(n))
        Fvec(n-1) = Fvec(n-1) + x(i)*ld
        Fvec(n)   = Fvec(n)   + x(n)*ld
     end do

  case (2) ! Evaluate the Jacobian

     Fjac = 0.0D0
     do i = 1, n-2
        Fjac(i,i)   =  1.0D0
        Fjac(i,i+1) = -1.0D0
     end do
     do i = 1, n-1
        d = sqrt(x(i)*x(i)+x(n)*x(n))
        ld = myChain%l(i)/d
        ld3 = ld/(d*d)
        Fjac(n-1,i) = -x(i)*x(i)*ld3 + ld
        Fjac(n  ,i) = -x(i)*x(n)*ld3
        Fjac(n-1,n) = Fjac(n-1,n) + Fjac(n,i)
        Fjac(n  ,n) = Fjac(n  ,n) - x(n)*x(n)*ld3 + ld
     end do

  end select

end subroutine FCN
