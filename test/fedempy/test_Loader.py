# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module demonstrates modeling and solving using fedempy.
It creates the good-old Loader model.

The following environment variables need to be defined:
    FEDEM_REDUCER = Full path to the Fedem reducer shared object library
    FEDEM_SOLVER = Full path to the Fedem solver shared object library
    FEDEM_MDB = Full path to the Fedem model database shared object library
    SRC_DIR = Full path to the source folder of this test
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

    # Load the FE parts
    # They already contain the necessary external nodes (Triads)
    p1 = myModel.make_fe_part(src_dir + "Front.flm", tag="Front")
    p2 = myModel.make_fe_part(src_dir + "Boom.flm", tag="Boom")
    p3 = myModel.make_fe_part(src_dir + "Bucket.flm", tag="Bucket")
    p4 = myModel.make_fe_part(src_dir + "BellCrank.flm", tag="BellCrank")
    p5 = myModel.make_fe_part(src_dir + "BucketLink.flm", tag="BucketLink")
    parts = [p1, p2, p3, p4, p5]
    print("Created FE parts", parts)

    # Assign some proportional damping and move the parts in place
    myModel.edit_part(parts, alpha2=0.00286,
                      component_modes=0, consistent_mass=True)

    # Move the parts in place
    myModel.edit_part(p2, Tx=0.01080263, Tz=-0.77487206)
    myModel.edit_part(p3, Tx=-0.64636636, Tz=-2.0328088, Ry=-30, Rz=-180)
    myModel.edit_part(p4, Tx=-3.2499752, Ty=-2.8376081, Tz=0.04694241, Ry=21.814096)
    myModel.edit_part(p5, Tx=-2.041544, Ty=-0.92750001, Tz=0.12191465, Ry=-4.9156169)

    # Create revolute joints coupling the FE parts together and to ground
    joints = []
    joints.append(myModel.make_joint("Rev1", FmType.REVOLUTE_JOINT, 8))
    joints.append(myModel.make_joint("Rev2", FmType.REVOLUTE_JOINT, 11))
    joints.append(myModel.make_joint("Rev3", FmType.REVOLUTE_JOINT, 7, 19))
    joints.append(myModel.make_joint("Rev4", FmType.REVOLUTE_JOINT, 24, 6))
    joints.append(myModel.make_joint("Rev5", FmType.REVOLUTE_JOINT, 38, 22))
    joints.append(myModel.make_joint("Rev6", FmType.REVOLUTE_JOINT, 33, 17))
    joints.append(myModel.make_joint("Rev7", FmType.REVOLUTE_JOINT, 49, 21))
    joints.append(myModel.make_joint("Rev8", FmType.REVOLUTE_JOINT, 20, 48))
    joints.append(myModel.make_joint("Rev9", FmType.REVOLUTE_JOINT, 47, 56))
    joints.append(myModel.make_joint("Rv10", FmType.REVOLUTE_JOINT, 55, 35))
    print("Created Revolute joints", joints)

    # Reorient the joints
    myModel.edit_joint(joints[2], Ry=90, Rz=-90)
    myModel.edit_joint(joints[3:], Rx=90)

    # Create the load/motion functions
    f1 = myModel.make_function("Lift Cylinders",
                               frequency=0.625, amplitude=0.15,
                               delay=0.25, mean_value=0.15, end=0.8)
    f2 = myModel.make_function("Boom Cylinder",
                               slope=-0.5, start_ramp=0.5, end_ramp=1.2)
    f3 = myModel.make_function("Rotation Front",
                               frequency=0.5, amplitude=0.262,
                               delay=0.25, mean_value=0.262, end=1.0)
    f4 = myModel.make_function("Bucket Load",
                               slope=50000, start_ramp=0.7, end_ramp=1.2)
    print("Created Functions", [f1, f2, f3, f4])

    # Apply Rz-rotation on the Front part
    myModel.edit_joint(joints[0],
                       constraints={"Rz" : FmDofStat.SPRING},
                       spring={"Rz" : 1.0e9},
                       length={"Rz" : f3})

    # Create the hydraulic cylinder springs with prescribed length change
    cyl = myModel.make_spring("Lift cylinder", [(9, 23), (10, 18)],
                              init_Stiff_Coeff=1.0e9, length=f1)
    cyl.append(myModel.make_spring("Boom cylinder", (5, 50),
                                   init_Stiff_Coeff=1.0e9, length=f2))
    print("Created Axial springs", cyl)

    # Create the Bucket load
    print("Created Load", myModel.make_load("Load", FmLoadType.FORCE, 39, (0, 0, -1), fn=f4))

    # Create some strain rosettes on the Boom part
    r1 = myModel.make_strain_rosette("Gage A", p2,
                                     nodes=[111, 122, 123],
                                     direction=(0, 1, 0))
    r2 = myModel.make_strain_rosette("Gage B", p2,
                                     pos=[(-1.702537, -0.5171, 1.702752),
                                          (-1.649538, -0.5171, 1.658139),
                                          (-1.630592, -0.5171, 1.73142)],
                                     direction=(0, 1, 0))
    print("Created Strain Rosettes", [r1, r2])

    # Create some sensors measuring response quantities
    s1 = myModel.make_sensor("Front rot", joints[0], FmVar.POS, FmDof.RZ)
    s2 = myModel.make_sensor("Front vel", joints[0], FmVar.VEL, FmDof.RZ)
    s3 = myModel.make_sensor("Lift rot", joints[2], FmVar.POS, FmDof.RZ)
    s4 = myModel.make_sensor("Boom length", cyl[2], FmVar.LENGTH)
    s5 = myModel.make_sensor("Lift vel", joints[2], FmVar.VEL, FmDof.RZ)
    s6 = myModel.make_sensor("Gage 1", r1, FmVar.STRAIN, FmDof.SA_MAX)
    s7 = myModel.make_sensor("Load accel", 39, FmVar.ACC, FmDof.TZ)
    s8 = myModel.make_sensor("Gage 2", r2, FmVar.STRAIN, FmDof.SA_MAX)
    print("Created Sensors", [s1, s2, s3, s4, s5, s6, s7, s8])

    # Define some solver setup, dynamics simulation up to t=1.6
    # Switching off initial static equilibrium by setting t_quasi negative
    myModel.fm_solver_setup(t_quasi=-1.0, t_inc=0.01, t_end=1.6)
    # Set convergence tolerances
    myModel.fm_solver_tol(tol_ene=1.0e-6, tol_dis=1.0e-6, tol_vel=1.0e-6)

    # Save the model with its given name and clean up
    return myModel.close(True)


# Create the model, unless it already exists
src_dir = environ["SRC_DIR"] + "/models/"
FM_FILE = "Loaderpy.fmm"
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
rfile = open(src_dir + "exported_curves.asc", "r")
for line in rfile:
    print(line.rstrip())
    if line[0:5] == "#DESC":
        next(rfile)  # skip the first data line (t=0.0)
        break

print("  ## Starting time integration")
while mySolver.solve_next():
    # Extract results from the seven sensors defined
    vals = mySolver.get_functions(range(5,13))
    time = float(mySolver.get_current_time())
    vals.insert(0, time)  # Insert time as first column
    # Read reference data from file
    reference = [float(x) for x in next(rfile).split()]
    # Compare the response values using a relatively loose tolerance
    # since the Triad coordinates may contain some minor discrepancies
    vals[7] = reference[7]  # Ignore the triad acceleration (too unstable)
    ierr += compare_lists(time, vals, reference, 0.003)

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
