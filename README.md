<!---
  SPDX-FileCopyrightText: 2023 SAP SE

  SPDX-License-Identifier: Apache-2.0

  This file is part of FEDEM - https://openfedem.org
--->

[![REUSE status](https://api.reuse.software/badge/github.com/SAP/fedem-mdb)](https://api.reuse.software/info/github.com/SAP/fedem-mdb)

# FEDEM model database

![Fedem Logo](https://github.com/SAP/fedem-foundation/blob/main/cfg/FedemLogo.png)

## About this project

This project contains the source code of the C++ library that implements
the model database of FEDEM, with an API for accessing the model from
a shared object library (or DLL on windows). It is consumed as a submodule in
[fedem-gui](https://github.com/SAP/fedem-gui).

## Source code organization

The main bulk of the code is located in the [vpmDB](vpmDB) sub-folder,
which consists of an elaborate class hierarhcy representing the various
object types of a FEDEM model. See the file [README.md](vpmDB/README.md)
for more detailed information on this code.

The classes in `vpmDB` use some
[fedem-foundation](https://github.com/SAP/fedem-foundation) classes
as building blocks, especially some `FFaLib` classes for handling the
object fields and inter-object topology, and `FFlLib` for accessing the
FE part data. Therefore, this repository is consumed as submodule here.

In addition, we have the following sub-folders in this repository:

* [assemblyCreators](assemblyCreators)
  A library for generation of some parameterized higher-level
  mechanism components as sub-assemblies.
* [chainShape](chainShape)
  A Fortran library for calculation of nodal points along a beamstring
  with the shape of a free-hanging chain between two fixed points.
  Used for generation of start configurations of mooring lines, etc.
* [test](test)
  Top-level folder for the unit- and regression test files.

The file [FedemDB.C](FedemDB.C) defines the API, through which the
model database library can be accessed from python scripts.

## Requirements and Setup

A static build of the libraries `vpmDB`, `assemblyCreators` and `chainShape`
for use in the FEDEM GUI is governed by the build system of the supermodule
[fedem-gui](https://github.com/SAP/fedem-gui). However, it is also possible
to build them separately from this project, as described below.

### Build instructions for FedemDB

The build of the shared object library `FedemDB` is handled by the top-level
`CMakeLists.txt` file of this repository, which then also will build the
`vpmDB` library (and `chainShape` if Fortran is activated),
but not the `assemblyCreators` library. This also includes some unit-
and regression tests which may be executed locally.

We use the packages [googletest](https://github.com/google/googletest) and
[pFUnit](https://github.com/Goddard-Fortran-Ecosystem/pFUnit) to implement
some unit tests for the C++ and Fortran code, respectively. Therefore,
you need to set the environment variables `GTEST_ROOT` and `PFUNIT` to point to
the root path of valid installations of the googletest and pFUnit frameworks,
respectively, before executing the cmake command shown below.

- From a bash shell or command prompt, clone the sources of this repository:

      mkdir ~/Fedem-src
      cd ~/Fedem-src
      git clone --recurse-submodules git@github.com:SAP/fedem-mdb.git

- On Linux, the FEDEM model database library and tests can be built
  and executed by:

      cd ~/Fedem-src/fedem-mdb
      mkdir Release; cd Release
      cmake .. -DCMAKE_BUILD_TYPE=Release
      make check

  If you also want to build and test a shared object library for
  [chainShape](chainShape), you need to include `-DUSE_FORTRAN=ON`
  as command-line option to the cmake command above.

- On Windows, use the following bat script to configure the build with
  Visual Studio 2019 and (optionally) the Intel&reg; Fortran Compilers.

      @echo off
      title Open FEDEM DB configuration for Windows
      call "C:\Program Files (x86)\Intel\oneAPI\setvars.bat" intel64 vs2019
      set /p VERSION=<%USERPROFILE%\Fedem-OpenSrc\fedem-mdb\cfg\VERSION
      "%VS2019INSTALLDIR%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" ^
      -G "Visual Studio 16 2019" ^
      -S %USERPROFILE%\Fedem-src\fedem-mdb ^
      -B %USERPROFILE%\Fedem-build\fedemDB ^
      -DCMAKE_INSTALL_PREFIX=%USERPROFILE%\Fedem-install\%VERSION% ^
      -DUSE_QT=ON -DUSE_FORTRAN=ON
      pause

  If you don't have the Intel&reg; Fortran Compilers installed,
  you have to replace the call statement above by

      call "%VS2019INSTALLDIR%\VC\Auxiliary\Build\vcvarsall.bat" x64

  and remove the option `-DUSE_FORTRAN=ON`.
  It will then configure without Fortran support
  and not build the `chainShape` library.

- Then (still on Windows), open the generated solution file
  `%USERPROFILE%\Fedem-build\fedemDB\fedemDB.sln`
  in Visual Studio and build the `FedemDB` target for `Release`
  configuration to compile this library. Build the `check` target
  if you also want to execute the tests.

  Build the `INSTALL` target to install the binaries
  (the `.dll` files) in the folder `${CMAKE_INSTALL_PREFIX}\bin`
  where `CMAKE_INSTALL_PREFIX` is specified on the `cmake` command (see above).
  The binaries will then be installed in a subfolder named after the
  current version stored in the `cfg\VERSION` file.

### Regression testing

The regression tests invoked by building the `check` target are defined
in the subfolder [test/fedempy](test/fedempy). Since they rely on the
`fedempy` module, they will be active only if the repository
[fedem-solvers](https://github.com/SAP/fedem-solvers),
which contains the sources of `fedempy` has been cloned into a folder
named `fedem-solvers`, in parallel to the clone of this repository.

Most of the regression tests will also try to invoke the FEDEM solvers,
to verify the consistency of the generated models. This requires that
the following environment variables are set before running `make check`:

* `FEDEM_REDUCER` = Path to the FEDEM part reducer shared object library
* `FEDEM_SOLVER`  = Path to the FEDEM dynamics solver shared object library

See also the [README.md](test/fedempy/README.md) file for `fedempy`.

## Contributing

This project is open to feature requests/suggestions, bug reports etc. via [GitHub issues](https://github.com/SAP/fedem-mdb/issues). Contribution and feedback are encouraged and always welcome. For more information about how to contribute, the project structure, as well as additional contribution information, see our [Contribution Guidelines](CONTRIBUTING.md).

## Security / Disclosure

If you find any bug that may be a security problem, please follow our instructions at [in our security policy](https://github.com/SAP/fedem-mdb/security/policy) on how to report it. Please do not create GitHub issues for security-related doubts or problems.

## Code of Conduct

We as members, contributors, and leaders pledge to make participation in our community a harassment-free experience for everyone. By participating in this project, you agree to abide by its [Code of Conduct](https://github.com/SAP/.github/blob/main/CODE_OF_CONDUCT.md) at all times.

## Licensing

Copyright 2023 SAP SE or an SAP affiliate company and fedem-mdb contributors. Please see our [LICENSE](LICENSE) for copyright and license information. Detailed information including third-party components and their licensing/copyright information is available [via the REUSE tool](https://api.reuse.software/info/github.com/SAP/fedem-mdb).
