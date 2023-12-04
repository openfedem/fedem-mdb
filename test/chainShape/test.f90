!! SPDX-FileCopyrightText: 2023 SAP SE
!!
!! SPDX-License-Identifier: Apache-2.0
!!
!! This file is part of FEDEM - https://openfedem.org
!!==============================================================================

program test

  use ChainModel, only : ChainType, calcChainShape

  implicit none

  integer, parameter :: nprint = 0 ! Print av iterasjonsdata
  integer, parameter :: nPt  = 20  ! Antall massepunkt
  real*8 , parameter :: Ltot = 2.0 ! Total kjedelengde

  integer         :: ierr
  real*8, pointer :: X(:,:)
  type(ChainType) :: kjede

  allocate(kjede%l(nPt+1),kjede%m(nPt),stat=ierr)
  if (ierr /= 0) stop 'Allocation error'

  kjede%dX = 1.0 ! Horisontal avstand mellom start- og ende-punkt
  kjede%dY = 0.1 ! Hoydeforskjell mellom start- og ende-punkt

  kjede%l = Ltot/real(nPt+1) ! Lengde pr. element (konstant)
  kjede%m = 1.0; kjede%m(6) = -3.0; kjede%m(13) = 7.0 ! Her er punktmassene

  call calcChainShape (kjede,X,nprint,6,ierr)
  if (ierr < 0) stop ' *** Dessverre.'

  print '(A)','# Her er posisjonen til punkt-massene inkl. start- og ende-punkt'
  print 1,0.0,0.0
  print 1,(X(:,ierr),ierr=1,size(X,2))
1 format(2F8.4)
  deallocate(kjede%l,kjede%m,X)
  stop '# Ferdig!'

end program test
