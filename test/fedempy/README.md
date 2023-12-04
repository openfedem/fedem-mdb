<!---
  SPDX-FileCopyrightText: 2023 SAP SE

  SPDX-License-Identifier: Apache-2.0

  This file is part of FEDEM - https://openfedem.org
--->

# Regression tests for the FEDEM model database

This folder contains a set of python test drivers for conducting the
regression testing of the capabilities in the FEDEM model database library.

## Using fedempy

The regression tests use the `fedempy` python module of the
[fedem-solvers](https://github.com/SAP/fedem-solvers) repository.
The latest version of `fedempy` therefore needs to be installed
before executing the tests. Alternatively,
the variable `PYTHON_DIR` can be defined and point to the local path of the
[fedempy root folder](https://github.com/SAP/fedem-solvers/tree/main/PythonAPI/src)
within the local clone of `fedem-solvers.git`.
By this, the tests in here also serve as regression tests for `fedempy` itself.

## Test execution

The execution of the regression tests are managed by the ctest package
using the setup in the [CMakeLists.txt](CMakeLists.txt) file.
The `test_(name).py` drivers can also be executed manually
if you define the environment variables `PYTHONPATH` and
`SRC_DIR`=(local path of this directory) as indicated in `CMakeLists.txt`.

Notice that most of the tests also will try to invoke the FEDEM solvers,
to verify the consistency of the generated models. This requires that
the following environment variables are set:

* `FEDEM_REDUCER` = Path to the FEDEM part reducer shared object library
* `FEDEM_SOLVER`  = Path to the FEDEM dynamics solver shared object library

If these variables are not set, the tests will only complete the modelling tasks.
