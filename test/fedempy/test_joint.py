# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module tests the prismatic joint in fedempy
with 2 additional nodes along the glider (4 independent triads in total)

The test uses one environment variable:
    FEDEM_MDB = Full path to the Fedem model database shared object library
"""

from fedempy.fmm_solver import FmmSolver
from fedempy.modeler import FedemModeler
from fedempy.enums import FmDof, FmLoadType, FmType, FmVar
from os import environ


# Create a new, empty model
fmmfile = "prismaticJoint.fmm"
myModel = FedemModeler(fmmfile, True)


def create_model():
    # Create some triads
    t1 = myModel.make_triad("tx_start", (0, 0, 0))
    t2 = myModel.make_triad("tx_end", (3, 0, 0))
    t3 = myModel.make_triad("crossing", (1.5, 0, 0))
    t4 = myModel.make_triad("tz_middle", (1.5, 0, 1))
    t5 = myModel.make_triad("tz_end", (1.5, 0, 2))
    t6 = myModel.make_triad("m1", (1, 0, 0))
    t7 = myModel.make_triad("m2", (2, 0, 0))
    t8 = myModel.make_triad("m3", (1.25, 0, 0))  # additional glider node
    t0 = myModel.make_triad("m4", (1.75, 0, 0))  # additional glider node
    t9 = myModel.make_triad("support", (1.25, -1.0, 0))
    print("Created triads", [t1, t2, t3, t4, t5, t6, t7, t8, t9, t0])
    if any(i <= 0 for i in (t1, t2, t3, t4, t5, t6, t7, t8, t9, t0)):
        return -1

    # Create a material property               Rho   E       nu
    mat = myModel.make_beam_material("steel", (7850, 2.1e11, 0.3))
    print("Created material property", mat)
    if mat <= 0:
        return -2

    # Create a Pipe cross section                  Do   Di
    pipe = myModel.make_beam_section("pipe", mat, (0.2, 0.0))
    print("Created beam property", pipe)
    if pipe <= 0:
        return -3

    # Create a prismatic joint with 4 glider triads.
    # Start/end triads must be the first 2 elements of the list,
    # additional triads can be in random order.
    jt_id = myModel.make_joint("Prismatic", FmType.PRISMATIC_JOINT, t3, [t6, t7, t8, t0])
    print("Created prismatic joint", jt_id)
    if jt_id <= 0:
        return -4

    # build up longitudinal beams
    beams_l = myModel.make_beam("bx1", (t1, t6), pipe)
    beams_l.extend(myModel.make_beam("bx2", (t7, t2), pipe))
    print("Created longitudinal beams (x dir)", beams_l)
    if any(el is None for el in beams_l):
        return -5

    # build up transversal beams
    beams_t = myModel.make_beam("bz1", (t3, t4, t5), pipe)
    print("Created transversal beams (z dir)", beams_t)
    if any(el is None for el in beams_t):
        return -6

    # build vertical beam
    beams_v = myModel.make_beam("bv1", (t9, t8), pipe)
    print("Created vertical beam (y dir)", beams_v)
    if beams_v is None:
        return -7

    # build cross beam
    beams_c = myModel.make_beam("bc1", (t9, t0), pipe)
    print("Created cross beam (cross)", beams_c)
    if beams_c is None:
        return -8

    # Create a rigid joint attaching the first triad to ground (fixed boundary conditions)
    jnt1 = myModel.make_joint("Support", FmType.RIGID_JOINT, t1)
    jnt2 = myModel.make_joint("Support", FmType.RIGID_JOINT, t2)
    jnt3 = myModel.make_joint("Support", FmType.RIGID_JOINT, t5)
    jnt4 = myModel.make_joint("Support", FmType.RIGID_JOINT, t9)
    print("Created Rigid joint", [jnt1, jnt2, jnt3, jnt4])
    if jnt1 <= 0 or jnt2 <= 0 or jnt3 <= 0 or jnt4 <= 0:
        return -9

    # Create a sinusoidal load at the middle triad for transversal beams
    ldir = (1, 0, 0)  # Positive x-direction
    f1 = myModel.make_load("Fx_sine", FmLoadType.FORCE, t4, ldir, "1E6*sin(2.*3.14159*x)")
    print("Created load", f1)
    if f1 <= 0:
        return -10

    # Create a sensor measuring the tip Z-position
    s1 = myModel.make_sensor("displacement", t4, FmVar.POS, FmDof.TX)
    print("Created sensor", s1)
    if s1 <= 0:
        return -11

    # Define some solver setup, dynamics simulation for t=0,0.01,0.02,...5.0
    myModel.fm_solver_setup(t_inc = 0.01, t_end = 5.0)

    # Save the model with its given name
    if not myModel.save():
        return -12

    # Clean up
    myModel.close()

    return s1  # Return the output function Id


def run_model(s1):

    print("\n### Running dynamics solver on", fmmfile)
    mySolver = FmmSolver(fmmfile)

    while mySolver.solve_next():
        print(f"     d({mySolver.get_current_time()}) =", mySolver.get_functions([s1]))

    if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
        print("Time step loop OK, solver closed")
        mySolver.close_model(True)
    else:
        mySolver.close_model(False)
        print(" *** Solver failed", mySolver.ierr.value)
        exit(mySolver.ierr.value)


stat = create_model()
if stat < 0:
    myModel.close()
    print(f"**** Model generation failed {stat}, please check the log-file",
          fmmfile.replace(".fmm", ".log"))
    exit(-stat)

if "FEDEM_SOLVER" in environ:
    run_model(stat)
