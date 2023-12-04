!! SPDX-FileCopyrightText: 2023 SAP SE
!!
!! SPDX-License-Identifier: Apache-2.0
!!
!! This file is part of FEDEM - https://openfedem.org
!!==============================================================================

module ChainModel

  !!============================================================================
  !! This module involves calculation of the shape of a chain consisting of a
  !! given number of point masses connected by rigid bars of arbitrary length.
  !!
  !! Programmed by: Knut Morten Okstad             date/rev: July 04, 2014 / 1.0
  !!============================================================================

  implicit none

  type ChainType
     real*8          :: dX   ! Horizontal distance between end points
     real*8          :: dY   ! Vertical distance between end points
     real*8, pointer :: l(:) ! Lengths between each point mass (N+1)
     real*8, pointer :: m(:) ! The point masses (N)
  end type ChainType

  !! Global variables to be defined by application
  integer        , save          :: lpUnit            ! Logical print unit
  type(ChainType), save, pointer :: myChain => null() ! Chain data container


contains

  subroutine calcChainShape (chain, XY, nprint, lpu, ierr)

    !DEC$ ATTRIBUTES DLLEXPORT :: calcChainShape
    !!==========================================================================
    !! Calculates the shape of a chain of point-masses
    !! by solving the nonlinear set of equations F(x) = 0
    !! where the function F is defined by the subroutine FCN.
    !! The Minpack subroutine HYBRJ from Netlib is used.
    !!
    !! References:
    !! http://www.netlib.org/minpack/
    !! http://www.mcs.anl.gov/~more/ANL8074a.pdf
    !!==========================================================================

    !! Arguments
    type(ChainType), target , intent(in)  :: chain   ! Data defining the chain
    real*8         , pointer, intent(out) :: XY(:,:) ! Positions of the masses
    integer                 , intent(in)  :: nprint  ! Print switch for HYBRJ
    integer                 , intent(in)  :: lpu     ! Logical print unit
    integer                 , intent(out) :: ierr    ! Error flag

    !! Local variables
    integer             :: i, j, n, lr, qf, a1, a2, a3, a4, nw
    integer             :: maxfev, mode, nfev, njev
    real*8              :: xtol, factor, d
    real*8, allocatable :: x(:), Fvec(:), Fjac(:,:), diag(:), w(:)

    !! Objective function
    external :: FCN

    !! --- Logic section ---

    !! Check input variables
    if (associated(chain%m) .and. associated(chain%l)) then
       n = min(size(chain%m)+2,size(chain%l)+1)
       do i = 1, n-1
          if (chain%l(i) <= 0.0D0) then
             n = -i
             exit
          end if
       end do
    else
       n = 0
    end if
    if (n < 3) then
       ierr = -1
       if (lpu > 0) write(lpu,*) ' *** Error: Invalid chain definition.'
       return
    end if

    !! Allocate work arrays for HYBRJ
    lr = n*(n+1)/2
    qf = lr + 1
    a1 = qf + n
    a2 = a1 + n
    a3 = a2 + n
    a4 = a3 + n
    nw = a4 + n - 1
    allocate(x(n),Fvec(n),Fjac(n,n),diag(n),w(nw),stat=ierr)
    if (ierr /= 0) then
       ierr = -99
       if (lpu > 0) write(lpu,*) '*** Allocation error 1'
       return
    end if

    !! Initial guess
    x(1:n-2) = chain%m(1:n-2)
    x(n-1) = chain%m(n-2)
    x(n) = 0.0D0

    !! Set some input parameters for HYBRJ
    xtol    = 1.0D-8
    maxfev  = 1000
    mode    = 1
    factor  = 100.0D0
    lpUnit  = max(6,lpu)
    myChain => chain

    !! Solve the nonlinear problem using Minpack::HYBRJ
    call HYBRJ (FCN,n,x(1),fvec(1),fjac(1,1),n,xtol,maxfev, &
         &      diag(1),mode,factor,nprint,ierr,nfev,njev, &
         &      w(1),lr,w(qf),w(a1),w(a2),w(a3),w(a4))
    select case (ierr)
    case (0)
       if (lpu > 0) write(lpu,*) '*** Improper input arguments to HYBRJ.'
       ierr = -1
       return
    case (1)
       if (lpu > 0) write(lpu,*) '  * HYBRJ has converged.'
       ierr = 0
    case (2)
       if (lpu > 0) write(lpu,*) ' ** HYBRJ reached',maxfev,' evaluations.'
    case (3)
       if (lpu > 0) write(lpu,*) ' ** HYBRJ says xtol is too small.'
    case (4)
       if (lpu > 0) write(lpu,*) ' ** Slow convergence in last 5 Jacobians.'
    case (5)
       if (lpu > 0) write(lpu,*) ' ** Slow convergence in last 10 iterations.'
    end select

    !! Calculate point positions from the solution
    deallocate(Fvec,Fjac,diag,w)
    allocate(XY(2,n-1),stat=ierr)
    if (ierr /= 0) then
       ierr = -99
       if (lpu > 0) write(lpu,*) '*** Allocation error 2'
       return
    end if

    do i = 1, n-1
       XY(:,i) = 0.0D0
       do j = 1, i
          d = sqrt(x(j)*x(j)+x(n)*x(n))
          XY(1,i) = XY(1,i) + chain%l(j)*x(n)/d
          XY(2,i) = XY(2,i) - chain%l(j)*x(j)/d
       end do
    end do
    deallocate(x)

  end subroutine calcChainShape

end module ChainModel

