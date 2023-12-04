# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This Python test executes the solver using a fmm-file as input.
It uses input files from the folder `$TEST_DIR/CarSuspension`.

This test uses three environment variables:
    FEDEM_REDUCER = Full path to the reducer shared object library
    FEDEM_SOLVER = Full path to the solver shared object library
    FEDEM_MDB = Full path to the Fedem model database shared object library
    TEST_DIR = Full path to the solver tests source folder
"""

from os import environ, getcwd
from shutil import copyfile
from fedempy.fmm_solver import FmmSolver


if "FEDEM_SOLVER" not in environ:
    exit(0)  # Nothing to do here without solvers

################################################################################
# Start the fedem solver on the model file:
# $TEST_DIR/CarSuspension/SLA.fmm
# But first, copy the input files to current working directory,
# such that the results database will be created here.
# No result comparison in this test, only check that it runs without errors.
solver = FmmSolver()
fmfile = "SLA.fmm"
wrkdir = getcwd() + "/"
srcdir = environ["TEST_DIR"] + "/CarSuspension/"
copyfile(srcdir + fmfile, wrkdir + fmfile)
copyfile(srcdir + "lca.nas", wrkdir + "lca.nas")
copyfile(srcdir + "uca.nas", wrkdir + "uca.nas")
copyfile(srcdir + "knuckle.nas", wrkdir + "knuckle.nas")
copyfile(srcdir + "FZ.asc", wrkdir + "FZ.asc")
# Open the model file, run FE part reductions, and then start the solver
ierr = solver.start(fmfile, False, False, True)
if ierr < 0:
    exit(-ierr)

print("\n#### Running dynamics solver on", fmfile)
while solver.solve_next():
    pass

ierr = solver.ierr.value
if ierr >= 0:
    ierr = solver.solver_done(False)
    print("Simulation of", fmfile, "OK, solver closed")

# Update the model file (unless the simulation failed) and close it
solver.close_model(ierr == 0)
# Release the heap-allocated FFl-singeltons in the solver library.
# Note: This would normally be done by the solver_done() call above,
# but it appears that this also affects similar singeltons in the
# modeler library such that the subsequent model save operation fails.
# Consider this as a workaround.
if ierr == 0:
    solver.solver_close()
