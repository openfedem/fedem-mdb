# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
Running the inverse solver based on model file input. The model used is
Cantilever4_twin.fmm from the InversePy/beam_transversal folder of the
solver_tests repository. Further explanations are given in the README
file in that folder.
"""

from os import environ
from numpy import loadtxt
from shutil import copyfile

from fedempy.fmm_solver import FmmInverse
from test_utils import compare_lists


def _internal_eq(int_eq):
    """
    Transforms the internal equations to dictionary layout
    """
    def_keys = {
        'unknown_f' :  [ 'triadID', 'dof', 'dataSrc' ],
        'known_intF':  [ 'beamID', 'triadID', 'dof', 'dataSrc' ],
    }

    nn = {}
    for key, value in int_eq.items():
        nn[key] = []
        for j in value:  # loop over all tuples
            if isinstance(j, tuple):
                m = list(j)  # transform tuple to list
            else:
                m = [j]  # insert int to list
            nn[key].append(dict(zip(def_keys[key], m)))
    return {'internal_equations' : nn}


if "FEDEM_SOLVER" not in environ:
    exit(0)  # Nothing to do here without solvers

srcdir = environ["TEST_DIR"] + "/beam_transversal/"
srcdir = srcdir.replace("TimeDomain", "InversePy")
fmfile = "Cantilever4_twin.fmm"
copyfile(srcdir + fmfile, "./" + fmfile)
print("\n#### Running inverse solver on", fmfile)

# Set up the inverse input
inp = _internal_eq(
    {
        "unknown_f": [ ("Tip", "ty"), ("Tip", "rz") ],
        "known_intF": [ ("B2", "T2", "rz"), ("B4", "T4", "ty") ],
    }
)

# Create the inverse solver object
solver = FmmInverse(fmfile, inp)

# Read input data
ref = loadtxt(open(srcdir + "refData.asc", "rb"), delimiter=",")

# List of function IDs to extract results for
funcId = ["Mz_17", "Qy_113", "Disp_114"]

ierr = 0
nstp = 999
for n in range(nstp):
    print("\n+++++++++++++++++++++++++++++++++++++++++++++")
    print("Solving time increment", n, "with inputs", ref[n, 0:2])
    res = solver.run_inverse(ref[n, 0:2], out_def=funcId)
    if res is None:
        break  # end of simulation
    time = float(solver.get_current_time())
    print("results:", time, ref[n][2], res[2])
    ierr += compare_lists(time, res, ref[n], [0.02, 0.01, 0.003])

if ierr > 0:
    print(" *** Detected", ierr, "discrepancies, test not passed")
else:
    print("   * All response values match :-)")

if solver.solver_done() == 0 and solver.ierr.value == 0:
    print("  ## Time step loop OK, solver closed")
    solver.close_model(True)
    if ierr > 0:
        exit(ierr)
else:
    solver.close_model(False)
    print(" *** Solver failed", solver.ierr.value)
    exit(solver.ierr.value)
