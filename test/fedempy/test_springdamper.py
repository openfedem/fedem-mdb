# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module tests some basic modeling functionalities in fedempy.

The test uses one environment variable:
    FEDEM_MDB = Full path to the Fedem model database shared object library
"""

from fedempy.fmm_solver import FmmSolver
from fedempy.modeler import FedemModeler, FmDof, FmDofStat, FmVar
from os import environ

# Create a new, empty model
FM_FILE = "springdamper.fmm"
myModel = FedemModeler(FM_FILE, True)

# Create some triads
t1 = myModel.make_triad("T1", (0, 0, 0))
t2 = myModel.make_triad("T2", (2, 0, 0))
t3 = myModel.make_triad("T3", (3, 1, 0.5))
print("Created triads", [t1, t2, t3])
if t1 <= 0 or t2 <= 0 or t3 <= 0:
    exit(1)

# Totally fix the first Triad
if not myModel.edit_triad(t1, constraints={"All" : FmDofStat.FIXED}):
    exit(2)

# Constrain rotations in the other two triads and add some masses
if not myModel.edit_triad(t2, mass=1500.0, constraints={
    "Rx" : FmDofStat.FIXED,
    "Ry" : FmDofStat.FIXED,
    "Rz" : FmDofStat.FIXED,
    }):
    exit(3)

if not myModel.edit_triad(t3, mass=1000.0, constraints={
    "Rx" : FmDofStat.FIXED,
    "Ry" : FmDofStat.FIXED,
    "Rz" : FmDofStat.FIXED,
    }):
    exit(4)

# Create two spring elements (polyline)
springs = myModel.make_spring(
    "MySpring",
    [(t1, t2), (t2, t3)],
    xy = [ [0., 0.], [0.01, 1000.], [0.02, 3000.], [0.2, 3500.], [0.4, 1800.],
           [0.5, 750.], [2.0, 1277.] ],
    extrapol_type = "FLAT",
    spring_characteristics = "SPR_TRA_FORCE"
)
print("Created spring elements", springs)
if springs is None:
    exit(5)

# Create a damper element (polyline)
damper1 = myModel.make_damper(
    "MyDamper",
    (t1, t2),
    xy = [ [0., 0.], [0.01, 0.1], [0.02, 0.3], [0.2, 0.1], [0.4, 0.18],
           [0.5, 0.05], [2.0, 0.075] ],
    extrapol_type = "FLAT",
    damp_characteristics = "DA_TRA_COEFF"
)
print("Created damper element", damper1)
if damper1 is None:
    exit(6)

# Create second damper element with constant damping coefficient
damper2 = myModel.make_damper("Damper2", (t2, t3), init_Damp_Coeff = 0.0143)
print("Created damper element", damper2)
if damper2 is None:
    exit(7)

# Create a sensor measuring the tip displacement
s1 = myModel.make_sensor("Forskyvning", t3, FmVar.POS, FmDof.TZ)
print("Created sensor", s1)
if s1 <= 0:
    exit(10)

# Save the model with its given name and clean up
if not myModel.close(True):
    exit(11)

if "FEDEM_SOLVER" in environ:
    # Try to solve it
    print("\n### Running dynamics solver on", FM_FILE)
    mySolver = FmmSolver(FM_FILE)
    while mySolver.solve_next():
        time = mySolver.get_current_time()
        disp = mySolver.get_function(s1)
        print(f"     d({time}) =", disp)

    if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
        print("Time step loop OK, solver closed")
        mySolver.close_model(True)
    else:
        mySolver.close_model(False)
        print(" *** Solver failed", mySolver.ierr.value)
        exit(mySolver.ierr.value)
