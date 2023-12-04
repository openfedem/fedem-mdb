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
from fedempy.enums import FmDof, FmDofStat, FmLoadType, FmType, FmVar
from os import environ

# Create a new, empty model
fmmfile = "jalla.fmm"
myModel = FedemModeler(fmmfile, True)

def error_exit(stat):
    """
    Convenience function to print error message on exit.
    Also close the model, otherwise the log-file is not saved.
    """
    myModel.close()
    print("**** Model generation failed, please check the log-file",
          fmmfile.replace(".fmm", ".log"))
    exit(stat)

# Create some triads
t1 = myModel.make_triad("T1", (0, 0, 0))
t2 = myModel.make_triad("T2", (2, 0, 0))
t3 = myModel.make_triad("T3", (3, 1, 0.5))
print("Created triads", [t1, t2, t3])
if t1 <= 0 or t2 <= 0 or t3 <= 0:
    error_exit(1)

# Tag some triads
myModel.fm_tag_object([t1, t3], "Ends")
ends = myModel.fm_get_objects(tag="Ends")
print("Tagged triads:", ends)

# Create a material property               Rho   E       nu
mat = myModel.make_beam_material("Steel", (7850, 2.1e11, 0.3), "B1")
print("Created material property", mat)
if mat <= 0:
    error_exit(2)

# Create a Pipe cross section                  Do   Di
pipe = myModel.make_beam_section("Pipe", mat, (0.5, 0.45), "B1")
print("Created beam property", pipe)
if pipe <= 0:
    error_exit(3)

# Create two beam elements connecting the three triads
beams = myModel.make_beam("Bjelken", (t1, t2, t3), pipe, "B2")
print("Created beams", beams)
if beams is None:
    error_exit(4)

# Check object retrieval using regular expression
beam_objs = myModel.fm_get_objects(tag="B.")
print("Here are the tagged beam objects:", beam_objs)
if len(beam_objs) < 4:
    error_exit(4)

# Extract global position of triad
print(f"Position of Triad [{t3}]:", myModel.fm_get_pos(t3))

# Create a rigid joint attaching the first triad to ground
jnt = myModel.make_joint("Support", FmType.RIGID_JOINT, t1)
print("Created Rigid joint", jnt)
if jnt <= 0:
    error_exit(5)

# Create a sinusoidal load at the tip triad
ldir = (0, 0, 1)  # Positive Z-direction
loa1 = myModel.make_load("Last 1", FmLoadType.FORCE, t3, ldir, "1E6*sin(10*x)")
# Create a constant load at the middle triad
ldir = (0, 1, 0)  # Positive Y-direction
loa2 = myModel.make_load("Last 2", FmLoadType.FORCE, t2, ldir, "1234.56")
# Create a torque at the tip triad
loa3 = myModel.make_load("Last 3", FmLoadType.TORQUE, t3, ldir,
                         fn=myModel.make_function("Sinus", frequency=0.1))
print("Created loads", [loa1, loa2, loa3])
if loa1 <= 0 or loa2 <= 0 or loa3 <= 0:
    error_exit(6)

# Create a sensor measuring the tip Z-position
s1 = myModel.make_sensor("Forskyvning", t3, FmVar.POS, FmDof.TZ)
# Create a relative sensor in the Y-position
s2 = myModel.make_sensor("Forskyvning", (t3, t2), FmVar.POS, FmDof.TY)
print("Created sensors", [s1, s2])
if s1 <= 0 or s2 <= 0:
    error_exit(8)

# Relocate the tip triad (t3) 0.23 length units in negative Y-direction
# and rotate it triad 30 degrees about Z-axis
# Then fix it in X-direction
if not myModel.edit_triad(t3, Ty=-0.23, Rz=30.0,
                          constraints={"Tx" : FmDofStat.FIXED}):
    error_exit(11)

# Create two more triads and a beam connecting them
t4 = myModel.make_triad("T4", (2, 0, 0), tag="t4")
t5 = myModel.make_triad("T5", (3, 3, 0), tag="t5")
print("Created triads", [t4, t5])
b2 = myModel.make_beam("Bjelke2", (t4, t5), pipe)
print("Created beam", b2)
if b2 is None:
    error_exit(12)

# Try referring triads via their tag
if not myModel.edit_triad("t.", mass=12.34):
    error_exit(15)

# Connect the two beams using a ball joint
ball = myModel.make_joint("Kuleledd", FmType.BALL_JOINT, t4, t2)
print("Created Rigid joint", ball)
if ball <= 0:
    error_exit(13)

# Assign some spring/damper properties,
# but fix the joint during initial equilibrium (SPRING_DYN)
if not myModel.edit_joint(ball,
                          spring={"Rx":1000, "Ry":1200, "Rz":1230},
                          damper={"Rx":10.0, "Ry":12.3, "Rz":2.34},
                          constraints={
                              "Rx" : FmDofStat.SPRING_DYN,
                              "Ry" : FmDofStat.SPRING_DYN,
                              "Rz" : FmDofStat.SPRING_DYN,
                          }):
    error_exit(14)

# Create two more triads and a beam connecting them
t6 = myModel.make_triad("T6", (2.5, 1.5, 0))
t7 = myModel.make_triad("T7", (3.5, 4.5, 0))
print("Created triads", [t6, t7])
b3 = myModel.make_beam("Bjelke3", (t6, t7), pipe)
print("Created beam", b3)
if b3 is None:
    error_exit(15)

# Connect the two beams using a revolute joint with free Z-translation
cyl = myModel.make_joint("Cylinder", FmType.REVOLUTE_JOINT, t6, t5)
print("Created Rigid joint", ball)
if cyl <= 0:
    error_exit(16)
if not myModel.edit_joint(cyl, Ry=90, tra_ref=b2[0], rot_ref=b2[0],
                          spring={"Tz":100000}, damper={"Tz":10},
                          constraints={"Tz" : FmDofStat.SPRING_DYN,
                                       "Rz" : FmDofStat.FIXED}):
    error_exit(17)

# Define some solver setup, dynamics simulation for t=0,0.1,0.2,...4.0
myModel.fm_solver_setup(t_inc=0.1, t_end=4)

# Save the model with its given name and clean up
if not myModel.close(True):
    print("     Please check the log-file", fmmfile.replace(".fmm", ".log"))
    exit(99)

if "FEDEM_SOLVER" in environ:
    # Try to solve it
    print("\n### Running dynamics solver on", fmmfile)
    mySolver = FmmSolver(fmmfile)
    while mySolver.solve_next():
        print(f"     d({mySolver.get_current_time()}) =", mySolver.get_functions([s1, s2]))

    if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
        print("Time step loop OK, solver closed")
        mySolver.close_model(True)
    else:
        mySolver.close_model(False)
        print(" *** Solver failed", mySolver.ierr.value)
        exit(mySolver.ierr.value)

# Create a new simple model consisting of just a free-falling Triad.
# This is the simplest solvable model one can have in Fedem.
fmmfile = "triad.fmm"
myModel.fm_new(fmmfile)
myModel.edit_triad(myModel.make_triad("T1", (0, 0, 0)), mass=(10, 1, 1, 1))
myModel.close(True)

if "FEDEM_SOLVER" in environ:
    ierr = mySolver.solve_all(fmmfile)
    if ierr < 0:
        exit(-ierr)
