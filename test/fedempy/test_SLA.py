# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module demonstrates modeling and solving using fedempy.
It creates the same CarSuspension model (SLA.fmm) which is
included in the solver regression test suite.

The following environment variables need to be defined:
    FEDEM_REDUCER = Full path to the Fedem reducer shared object library
    FEDEM_SOLVER = Full path to the Fedem solver shared object library
    FEDEM_MDB = Full path to the Fedem model database shared object library
    TEST_DIR = Full path to the solver tests source folder
"""

from fedempy.fmm_solver import FmmSolver
from fedempy.modeler import FedemModeler
from fedempy.enums import FmDof, FmDofStat, FmLoadType, FmType, FmVar
from test_utils import compare_lists
from os import environ, path
from sys import argv


def make_model (fmm_name, src_dir):
    """
    This function generates the Fedem model.
    """
    # Create a new, empty model
    myModel = FedemModeler(fmm_name, True)
    refplan = 2  # Base Id of the reference plane, always 2 for new models

    # Load the FE parts
    p1 = myModel.make_fe_part(src_dir + "lca.nas")
    p2 = myModel.make_fe_part(src_dir + "knuckle.nas")
    p3 = myModel.make_fe_part(src_dir + "uca.nas")
    print("Created FE parts", [p1, p2, p3])

    # Create triads attached to the FE parts.
    # The coordinates need to match nodal points in the FE models.
    t1 = myModel.make_triad("T1", (-0.05811678, 0.03882856, 0.07981566), on_part=p1)
    t2 = myModel.make_triad("T2", ( 0.17168449, 0.03882856, 0.07981566), on_part=p1)
    t3 = myModel.make_triad("T3", ( 0.0, -0.16,  0.087), on_part=p1)
    t4 = myModel.make_triad("T4", ( 0.0, -0.16,  0.087), on_part=p2)
    t5 = myModel.make_triad("T5", ( 0.0, -0.145, 0.298), on_part=p2)
    t6 = myModel.make_triad("T6", ( 0.0, -0.145, 0.298), on_part=p3)
    t7 = myModel.make_triad("T7", ( 0.08428750, -0.02, 0.28652272), on_part=p3)
    t8 = myModel.make_triad("T8", (-0.08312982, -0.02, 0.31604290), on_part=p3)
    t9 = myModel.make_triad("T3", ( 0.00603329, -0.07978369, 0.08261359), on_part=p1)
    frame = myModel.make_triad("Frame", (0.0, 0.0, 0.3), on_part=refplan)
    wheel = myModel.make_triad("Wheel", (0.00312002, -0.215, 0.2), on_part=p2)
    steer = myModel.make_triad("Steer", (0.08525102, -0.201314, 0.1854661), on_part=p2)
    print("Created triads", [t1, t2, t3, t4, t5, t6, t7, t8, t9, frame, wheel, steer])
    myModel.edit_triad(steer, constraints={"Tx" : FmDofStat.FIXED})

    # Create ball joints coupling the FE parts together and to ground
    b1 = myModel.make_joint("B1", FmType.BALL_JOINT, t1)
    b2 = myModel.make_joint("B2", FmType.BALL_JOINT, t2)
    b3 = myModel.make_joint("B3", FmType.BALL_JOINT, t4, t3)
    b4 = myModel.make_joint("B4", FmType.BALL_JOINT, t6, t5)
    b5 = myModel.make_joint("B5", FmType.BALL_JOINT, t7)
    b6 = myModel.make_joint("B6", FmType.BALL_JOINT, t8)
    print("Created Ball joints", [b1, b2, b3, b4, b5, b6])

    # Create the axial spring/damper
    spr = myModel.make_spring("Spring", (frame, t9), init_Stiff_Coeff=750000.0)
    dmp = myModel.make_damper("Damper", (frame, t9), init_Damp_Coeff=3000.0)

    # Create the load on the wheel triad
    f1 = myModel.make_function("Load function", filename=src_dir + "FZ.asc")
    print("Created Function", f1)
    l1 = myModel.make_load("Load", FmLoadType.FORCE, wheel, (0, 0, -1), fn=f1)
    print("Created Load", l1)

    # Create some sensors measuring response quantities
    s2 = myModel.make_sensor("Spring defl", spr, FmVar.DEFLECTION)
    s3 = myModel.make_sensor("Spring force", spr, FmVar.FORCE)
    s4 = myModel.make_sensor("Damper force", dmp, FmVar.FORCE)
    s5 = myModel.make_sensor("B4 rot angle", b4, FmVar.POS, FmDof.RX)
    s6 = myModel.make_sensor("B1 angle vel", b1, FmVar.VEL, FmDof.RX)
    s7 = myModel.make_sensor("Steer position", steer, FmVar.POS, FmDof.TZ)
    print("Created Sensors", [s2, s3, s4, s5, s6, s7])

    # Define some solver setup, dynamics simulation up to t=2.5
    myModel.fm_solver_setup(t_inc=0.005, t_end=2.5, t_quasi=-1.0)
    # Set convergence tolerances
    myModel.fm_solver_tol(tol_ene=1.0e-6, tol_dis=1.0e-6, tol_vel=1.0e-6)

    # Save the model with its given name and clean up
    return myModel.close(True)


# Create the model, unless it already exists
src_dir = environ["TEST_DIR"] + "/CarSuspension/"
FM_FILE = "SLApy.fmm"
iarg = 1
if len(argv) > iarg and argv[iarg] == "--run-in-src":
    FM_FILE = src_dir + FM_FILE
    iarg = 2
if len(argv) > iarg and argv[iarg] == "--force-new":
    force_new = True
    iarg += 1
else:
    force_new = False
if force_new or not path.isfile(FM_FILE):
    if not make_model(FM_FILE, src_dir):
        exit(1)

if "FEDEM_SOLVER" not in environ:
    exit(0)

# Now try to solve it
mySolver = FmmSolver()
if len(argv) > iarg and argv[iarg] == "--no-compare":
    exit(mySolver.solve_all(FM_FILE, True, True))

# Solve it, while checking the response variables against reference data
print("\n#### Running dynamics solver on", FM_FILE)
ierr = mySolver.start(FM_FILE, False, False, True)
if ierr < 0:
    exit(-ierr)

# Read reference values to compare with from file
print("Comparing with response variables in exported_curves.asc")
rfile = open(src_dir + "exported_curves.asc", "r", encoding="latin-1")
for line in rfile:
    # Need the encode-decode stuff to eliminate non-ascii chars giving error
    print(line.rstrip().encode("ascii", "ignore").decode("ascii"))
    if line[0:5] == "#DESC":
        next(rfile)  # skip the first data line (t=0.0)
        break

print("  ## Starting time integration")
while mySolver.solve_next():
    # Extract results from the seven sensors defined
    vals = mySolver.get_functions([1, 2, 3, 4, 0, 0, 0, 0, 5, 6, 7])
    time = float(mySolver.get_current_time())
    vals.insert(0, time)  # Insert time as first column
    vals[-1] -= 0.1854661  # Subtract initial Z-coordinate to get displacement
    # Read reference data from file
    reference = [float(x) for x in next(rfile).split()]
    vals[5:8] = reference[5:8]  # Insert reference values when no sensor data
    # Compare the response values using a relatively loose tolerance
    # since the Triad coordinates contain some minor discrepancies
    ierr += compare_lists(time, vals, reference, 0.002)

if ierr > 0:
    print(" *** Detected", ierr, "discrepancies, test not passed")
else:
    print("   * All response values match :-)")

if mySolver.solver_done() == 0 and mySolver.ierr.value == 0:
    print("  ## Time step loop OK, solver closed")
    mySolver.close_model(True)
    if ierr > 0:
        exit(ierr)
else:
    mySolver.close_model(False)
    print(" *** Solver failed", mySolver.ierr.value)
    exit(mySolver.ierr.value)
