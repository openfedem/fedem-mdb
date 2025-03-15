# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This Python test executes the solver using a fmm-file as input.
It uses input files from the folder `$TEST_DIR/CarSuspension`.

This test uses three environment variables:
    FEDEM_SOLVER = Full path to the solver shared object library
    FEDEM_MDB = Full path to the Fedem model database shared object library
    TEST_DIR = Full path to the solver tests source folder
"""

from os import environ, path
from fedempy.modeler import FedemModeler
from fedempy.fmm_solver import FmmSolver
from fedempy.enums import FmDof, FmVar, FmType
from test_utils import compare_lists


if "FEDEM_SOLVER" not in environ:
    exit(0)  # Nothing to do here without solvers

################################################################################
# Start the fedem solver on the model file SLA.fmm, which now is assumed to be
# present in the build tree due to the successful execution of an earlier test.
# We're testing the solve_window() method only, but need to add sensors first.

fmfile = "SLA.fmm"
srcdir = environ["TEST_DIR"] + "/CarSuspension/"
if not path.exists(fmfile):
    print(f"\n  ** File {fmfile} does not exist - test skipped.")
    exit(0)

model = FedemModeler(fmfile)
nfunc = model.fm_count(FmType.FUNCTION)
print("   * Model file", fmfile, "successfully opened")
print("     Number of Functions:", nfunc)
if nfunc < 7:
    # Create some sensors measuring the response quantities to compare with
    s2 = model.make_sensor("Spring defl", 61, FmVar.DEFLECTION)
    s3 = model.make_sensor("Spring force", 61, FmVar.FORCE)
    s4 = model.make_sensor("Damper force", 62, FmVar.FORCE)
    s5 = model.make_sensor("B4 rot angle", 47, FmVar.POS, FmDof.RX)
    s6 = model.make_sensor("B1 angle vel", 24, FmVar.VEL, FmDof.RX)
    s7 = model.make_sensor("Steer position", 39, FmVar.POS, FmDof.TZ)
    print("Created Sensors", [s2, s3, s4, s5, s6, s7])
    fId = [ 1, s2, s3, s4, 0, 0, 0, 0, s5, s6, s7]
    model.close(True)
else:
    # Assume sensors have been created in an earlier execution
    fId = [ 1, 2, 3, 4, 0, 0, 0, 0, 5, 6, 7]
    model.close(False)

solver = FmmSolver(fmfile)
if solver.ierr.value < 0:
    exit(-solver.ierr.value)

print("\n#### Running dynamics solver on", fmfile)
# Read reference values to compare with from file
print("Comparing with response variables in exported_curves.asc")
references = []
rfile = open(srcdir + "exported_curves.asc", "r", encoding="latin-1")
for line in rfile:
    # Need the encode-decode stuff to eliminate non-ascii chars giving error
    line.rstrip().encode("ascii", "ignore").decode("ascii")
    if line[0:5] == "#DESC":
        for count, head in enumerate(line.split()):
            if count == 0:
                print("#Reference quantities:")
            else:
                print(count, head)
        next(rfile)  # skip the first data line (t=0.0)
    elif line[0] == "#":
        print(line.strip())  # comment line
    else:
        references.append([float(x) for x in line.split()])

# Run only the first time window here
ierr = 0
n_out = len(fId)
n_step = 100
# Caution: We here assume the time step size in the model is constant
t = solver.get_current_time()
dt = solver.get_next_time() - t
# Invoke the dynamics solver for this time window
outputs, do_continue = solver.solve_window(n_step, None, fId)
if do_continue and solver.ierr.value == 0:
    # Compare the responses
    for i in range(n_step):
        output = [float(outputs[j]) for j in range(n_out * i, n_out * (i + 1))]
        output.insert(0, float(t + dt * (i + 1)))
        output[5:8] = references[i][5:8]  # Replace non-existing sensor data
        ierr += compare_lists(output[0], output, references[i], 1.0e-4)
    if ierr > 0:
        print(f" *** Detected {ierr} discrepancies, test not passed")
    else:
        print(f"   * All response values match, checked {n_step} steps")

# Time window finished, terminate by closing down the result database, etc.
ierr += abs(solver.solver_done())
if ierr == 0:
    print("Time window OK, solver closed")
else:
    exit(ierr)

# Write updated model file
solver.close_model(True)
