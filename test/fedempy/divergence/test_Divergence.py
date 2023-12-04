# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
Running the loader model while checking solver behaviour
in case of divergence. Divergences are provoked by some
artifical peaks in the input data.
"""

from os import environ, getcwd
from numpy import array
from pandas import DataFrame, concat, read_csv
from fedempy.dts_operators.window import run


def swap_columns(df, col1, col2):
    """
    Swap two columns in pandas dataframe.
    """
    col_list = list(df.columns)
    x, y = col_list.index(col1), col_list.index(col2)
    col_list[y], col_list[x] = col_list[x], col_list[y]
    df = df[col_list]
    return df


def micro_to_sec(fname):
    """
    Shift time stamp from microsec to sec.
    """
    df = read_csv(fname, sep=",", index_col="_t")
    ts = array(df.index.tolist())
    ts = ts / 1000000
    df.set_index(ts, inplace=True)
    df.index.name = "_t"
    return df


def modify_row_in_data(dfr, start, end, factor=1.0, op="MULT"):
    """
    Modify rows in a dataframe to provoke divergence in Fedem.
    Selecting rows by start:end and modifying them with given factor.
    2 modifications are available: adding or multiplication.
    """
    print(f"Modifying data [{int(start)}:{int(end)}]\tfactor: {factor}")

    if op == "MULT":
        dfr.iloc[int(start) : int(end)] *= factor
    else:
        dfr.iloc[int(start) : int(end)] += factor

    return dfr


class DTScontext:
    """
    Dummy DTS container class.
    """

    def __init__(self):
        self._window = None
        self._state = None

    @property
    def window(self):
        return self._window

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value


if "FEDEM_SOLVER" not in environ:
    exit(0)  # Nothing to do here without solvers

# Define the state container
dts = DTScontext()

# Start the fedem solver
fmm_file = "loader_01.fmm"
print("\n#### Running dynamics solver on", fmm_file)

# Load input data, and shift the time to sec from microsec
df = micro_to_sec(environ["SRC_DIR"] + "/divergence/loader_01_input.csv")

# swap "Loader_cylinder_left" <-> "Loader_cylinder_top"
# order in csv file is not matching the order in modelfile loader_01.fmm
df = swap_columns(df, "Loader_cylinder_left", "Loader_cylinder_top")
print("\nShape of the input data frame: ", df.shape)

# List of output function tags
out_tag = [
    "Loader_bucket_load_mid_X",
    "Loader_bucket_load_mid_Y",
    "Loader_bucket_load_mid_Z",
    "Loader_cylinder_right_F",
    "Loader_cylinder_top_F",
    "Loader_cylinder_left_F",
]

"""
Define arguments
maxit:
    Max number of Newton iterations
    Other convergence parameters are tolDistNorm, tolVelNorm, tolEnerSum
use_state_n_steps_behind:
    In case of divergence, step this many steps back on restart
nr_steps:
    Number of quasi-static steps ahead (large number - to window's end)
sequence:
    The order for convergence handling
"""
kwargs = {
    "fmm_file": fmm_file,
    "lib_dir": getcwd(),
    "output_ids": out_tag,  # function ids [18:23]
    "keep_old_res": True,  # avoid that res-files are overwritten on restart
    "use_state": True,
    "use_times": True,
    "crash_options": {
        "maxit": 15,
        "use_ramping": True,
        "use_state_n_steps_behind": 10,
        "nr_steps": 1000,
        "sequence": ["TOL", "STATE_STATIC", "STATE_DYNAMIC", "JUMP_OVER"],
    },
}

n_step = df.shape[0]
w = 200  # set window size to 200 steps
wc0 = 100  # crash location 1
wc1 = 300  # crash location 2
h_w = 5  # half bandwidth for manipulated data

# Dataset modification (first 2 windows) around center +/-h_w rows
# Use factor 30. for increasing input data
dfr = df.copy(deep=True)
dfr = modify_row_in_data(dfr, wc0 * (0.5) - h_w, wc0 * (0.5) + h_w, 30.0)
dfr = modify_row_in_data(dfr, wc1 * (1.0) - h_w, wc1 * (1.0) + h_w, 30.0)

# Loop index for simulation windows
i0 = 0

# Result data frame
res = DataFrame()

# Run simulation windows (here 3 windows, each 200 steps)
while i0 < n_step:
    # Run the simulation over the next 200 steps
    i1 = i0 + w
    if i0 == w:
        kwargs["crash_options"]["sequence"].insert(1, "JUMP_OVER")
    print(f"\nRun window from {i0} - {i1} steps:")
    dfw = dfr.iloc[i0 : i1, :].copy()  # make copy to avoid warning
    res = concat([res, run(dfw, dts, **kwargs)])
    i0 += w
