# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module tests modeling with FE parts using fedempy.

The test uses one environment variable:
    FEDEM_MDB = Full path to the Fedem model database shared object library
    SRC_DIR = Full path to the source directory of this test
"""

from fedempy.fmm_solver import FmmSolver
from fedempy.modeler import FedemModeler
from fedempy.enums import FmDof, FmLoadType, FmType, FmVar
from os import environ

# Create a new, empty model
FM_FILE = "CantileveredShell.fmm"
myModel = FedemModeler(FM_FILE, True)

# Create a FE part by loading specified file
p1 = myModel.make_fe_part(environ["SRC_DIR"] + "/models/CQUAD04_.nas")
print("Created FE part", p1)
if p1 <= 0:  # Failed to load specified FE data file
    myModel.close()
    exit(1)

# Reorient the part
if not myModel.edit_part(p1, Tx=1.0, Rx=90, Rz=90):
    myModel.close()
    exit(2)

# Create two triads attached to the part.
# The coordinates need to match, or else...
t1 = myModel.make_triad("Support", (0, 0, 0), on_part=p1)
t2 = myModel.make_triad("Tip", (1, 0, 0), on_part=p1)
print("Created triads", [t1, t2])
if t1 <= 0 or t2 <= 0:
    myModel.close()
    exit(3)

# Create a rigid joint attaching the first triad to ground
jnt = myModel.make_joint("Support", FmType.RIGID_JOINT, t1)
print("Created Rigid joint", jnt)
if jnt <= 0:
    myModel.close()
    exit(4)

# Create a sinusoidal load at the tip triad
ldir = (0, 0, 1)  # Positive Y-direction
load = myModel.make_load("Last", FmLoadType.FORCE, t2, ldir, "1E6*sin(10*x)")
print("Created load", load)
if load <= 0:
    myModel.close()
    exit(5)

# Create a sensor measuring the tip triad displacement
s1 = myModel.make_sensor("Forskyvning", t2, FmVar.POS, FmDof.TZ)
print("Created sensor", s1)
if s1 <= 0:
    myModel.close()
    exit(6)

# Define some solver setup, quasi-static simulation for t=0,0.1,0.2,...5.0
myModel.fm_solver_setup(t_inc=0.1, t_quasi=5)

# Save the model with its given name and clean up
if not myModel.close(True):
    exit(7)

if "FEDEM_SOLVER" in environ:
    # Now try to solve it
    mySolver = FmmSolver()
    print("\n### Running dynamics solver on", FM_FILE)
    ierr = mySolver.start(FM_FILE, False, False, True)
    if ierr < 0:
        exit(-ierr)
    while mySolver.solve_next():
        print(f"     d({mySolver.get_current_time()}) =", mySolver.get_function(s1))

    if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
        print("Time step loop OK, solver closed")
        mySolver.close_model(True)
    else:
        mySolver.close_model(False)
        print(" *** Solver failed", mySolver.ierr.value)
        exit(mySolver.ierr.value)
