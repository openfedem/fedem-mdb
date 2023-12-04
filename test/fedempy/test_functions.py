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
from fedempy.modeler import FedemModeler
from fedempy.enums import FmDof, FmVar
from numpy import savetxt, c_
from os import environ, path

# Create a new, empty model
FM_FILE = "functions.fmm"
myModel = FedemModeler(FM_FILE, True)
# Create a free-falling triad with a mass,
# just a dummy model such that the solver can be run
t1 = myModel.make_triad("T1", (0, 0, 0))
if not myModel.edit_triad(t1, mass=(10, 1, 1, 1)):
    print(" *** Failed to add mass to Triad", t1)

# Create polyline function "Poly"
# xy must be specified, i.e., xy=[ [x0, y0], [x1, y1], ...., [xn, yn] ]
# optional: extrapol_type ("NONE", "FLAT" or "LINEAR")
f_extId = myModel.make_function(
    "Poly",
    xy = [ [0., 0.], [0.01, 1.], [0.02, 3.], [0.2, -1.], [0.4, 1.8], [0.5, 0.75], [2.0, -0.75] ],
    extrapol_type = "FLAT",
)
print("Created polyline function", f_extId)
if f_extId <= 0:
    exit(1)

# Create sine function "Sine"
# frequency must be specified, e.g., frequency=3.0
# optional: delay, amplitude, mean_value, end
# e.g., delay=0.0, amplitude=3.0, mean_value=-0.1, end=0.0
fs_extId = myModel.make_function("Sine", frequency=3.0, amplitude=2.25)
print("Created sine function", fs_extId)
if fs_extId <= 0:
    exit(2)

# Create device file, if file doesn't exists
if not path.exists("poly_file.txt"):
    # write data to file, e.g. 2 channels T5 and T35
    x = [0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0]
    y = [0.0, 0.25, 1.0, 4.0, 9.0, 25.0, 30.0, 36.0]
    z = [0.0, -0.25, -1.0, -4.0, -9.0, -25.0, -30.0, -36.0]
    savetxt("poly_file.txt", c_[x,y,z], header="#DESCRIPTION T5 T35", comments="")

# Create polyline-from-file function "Device"
# filename must be specified, e.g., filename="An_ascii_file"
# optional: ch_name, sc_factor, z_adjust, v_shift
# e.g. ch_name="channel1", v_shift = 0.1, sc_factor = 1.2, z_adjust = False
dev_extId = myModel.make_function(
    "Device",
    filename = "poly_file.txt",
    ch_name = "T35",
    sc_factor = 1.25,
    z_adjust = False,
)
print("Created device function", dev_extId)
if dev_extId <= 0:
    exit(3)

# Create math expression function "Math_Expr"
# expression must be specified, e.g., expression="1+sin(x)"
me_extId = myModel.make_function("Math_Expr", expression="2.65*sin(10*x)")
print("Created mathematical expression function", me_extId)
if me_extId <= 0:
    exit(4)

# Create external function
# No additional arguments
ex_extId = myModel.make_function("External")
print("Created external function", ex_extId)
if ex_extId <= 0:
    exit(5)

# Create some linear functions
f1 = myModel.make_function("Constant", value=1.23)
f2 = myModel.make_function("Linear", slope=4.56)
f3 = myModel.make_function("Ramp", start_ramp=0.43, slope=3.2, start_val=0.8)
f4 = myModel.make_function("Ramp limited", start_ramp=0.43, end_ramp=1.62,
                                           slope=4.12, start_val=0.6)
print("Created linear functions", f1, f2, f3, f4)
if f1 <= 0 or f2 <= 0 or f3 <= 0 or f4 <= 0:
    exit(6)
if not myModel.edit_function(f2, t1, FmVar.POS, FmDof.TZ):
    print(" *** Failed to change argument of Function", f2, "to Tz in Triad", t1)
    exit(7)

# Create some input and output functions
i1 = myModel.make_function("First input", tag="I1")
i2 = myModel.make_function("Second input", tag="I2")
print("Created input functions", i1, i2)
o1 = myModel.make_sensor("Position", t1, FmVar.POS, FmDof.TZ, tag="Pos")
o2 = myModel.make_sensor("Velocity", t1, FmVar.VEL, FmDof.TZ, tag="Vel")
o3 = myModel.make_sensor("Acceleration", t1, FmVar.ACC, FmDof.TZ, tag="Acc")
print("Created output sensors", o1, o2, o3)
# Define some solver setup, dynamic simulation for t=0,0.1,0.2,...5.0
# without initial static equilibrium, therefore t_quasi=-1
myModel.fm_solver_setup(t_quasi=-1, t_inc=0.1, t_end=5)

# Save the model with its given name and clean up
if not myModel.close(True):
    exit(9)

if "FEDEM_SOLVER" in environ:
    # Try to solve it
    print("\n### Running dynamics solver on", FM_FILE)
    mySolver = FmmSolver(FM_FILE)
    while mySolver.solve_next():
        print(
            mySolver.get_current_time(),
            mySolver.get_functions([f_extId, fs_extId, dev_extId, me_extId]),
            mySolver.get_functions([f1, f2, f3, f4]),
            mySolver.get_functions(["I1", "I2", "Pos", "Vel", "Acc"]),
        )
        ti = mySolver.get_current_time()
        mySolver.set_input("I1", 10.0*ti)
        mySolver.set_input("I2", 20.0*ti)

    if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
        print("Time step loop OK, solver closed")
        mySolver.close_model(True)
    else:
        mySolver.close_model(False)
        print(" *** Solver failed", mySolver.ierr.value)
