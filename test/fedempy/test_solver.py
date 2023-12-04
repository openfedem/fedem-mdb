# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This Python test executes the solver using a fmm-file as input.
It uses input files from the folder `$TEST_DIR/Cantilever-extFunc`.

This test uses three environment variables:
    FEDEM_REDUCER = Full path to the reducer shared object library
    FEDEM_SOLVER = Full path to the solver shared object library
    FEDEM_MDB = Full path to the Fedem model database shared object library
    TEST_DIR = Full path to the solver tests source folder
"""

from os import environ, getcwd
from shutil import copyfile
from fedempy.modeler import FedemModeler
from fedempy.fmm_solver import FmmSolver
from fedempy.enums import FmDof, FmVar, FmType
from test_utils import compare_lists


if "FEDEM_SOLVER" not in environ:
    exit(0)  # Nothing to do here without solvers

################################################################################
# Start the fedem solver on the model file:
# $TEST_DIR/Cantilever-extFunc/Cantilever-internal.fmm
# But first, copy it to current working directory,
# such that the results database will be created here.
wrkdir = getcwd() + "/"
srcdir = environ["TEST_DIR"] + "/Cantilever-extFunc/"
fmfile = "Cantilever-internal.fmm"
copyfile(srcdir + fmfile, wrkdir + fmfile)
solver = FmmSolver(fmfile)

print("\n#### Running dynamics solver on", fmfile)
print("Comparing with response variables in TipPosition.asc")

# Read reference values to compare with from file
rfile = open(srcdir + "TipPosition.asc", "r")
for line in rfile:
    print(line.strip())
    if line[0:5] == "#DESC":
        next(rfile)  # skip the first data line (t=0.0)
        break

# List of function IDs to extract results for
fId = [3, 4, 5]
n_out = len(fId)

# Time step loop
ierr = 0
do_continue = True
references = []
while do_continue and solver.ierr.value == 0:
    # Solve for next time step
    time = solver.get_next_time()
    do_continue = solver.solve_next()
    # Extract the results
    outputs = [float(solver.get_function(fId[i])) for i in range(n_out)]
    outputs.insert(0, float(time))
    # Read reference data from file
    reference = [float(x) for x in next(rfile).split()]
    # Compare response values
    ierr += compare_lists(time, outputs, reference, 1.0e-8)
    references.append(reference)

if ierr > 0:
    print(f" *** Detected {ierr} discrepancies, test not passed")
else:
    print(f"   * All response values match, checked {len(references)} steps")

# Simulation finished, terminate by closing down the result database, etc.
rfile.close()
ierr += abs(solver.solver_done())
if ierr == 0 and solver.ierr.value == 0:
    print("Time step loop OK, solver closed")
else:
    exit(ierr + abs(solver.ierr.value))

# Write updated model file
solver.close_model(True)

################################################################################
# Start over on the same model, to verify that the model file
# is updated correctly when we already have some results.
# No result comparison in this run (should be identical).

ierr = solver.start(fmfile)
if ierr < 0:
    exit(-ierr)

print("\n#### Running dynamics solver on", fmfile)
while solver.solve_next() and solver.ierr.value == 0:
    pass

# Simulation finished, terminate by closing down the result database, etc.
ierr = abs(solver.solver_done())
if ierr == 0 and solver.ierr.value == 0:
    print("Time step loop OK, solver closed")
else:
    exit(ierr + abs(solver.ierr.value))

# Write updated model file
solver.close_model(True)

################################################################################
# Start the fedem solver on the model file SLA.fmm, which here is assumed to be
# present in the build tree due to the successful execution of an earlier test.
# We're testing the solve_window() method only, but need to add sensors first.

fmfile = "SLA.fmm"
srcdir = environ["TEST_DIR"] + "/CarSuspension/"

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

ierr = solver.start(fmfile)
if ierr < 0:
    exit(-ierr)

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
