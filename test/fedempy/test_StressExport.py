# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This Python test runs the model created by `test_Loader.py`
now using the stress_visualization operator as the simulation driver.

The following environment variables need to be defined:
    FEDEM_SOLVER = Full path to the Fedem solver shared object library
    VIS_EXPORTER = Full path to the visualization export shared object library
"""

from os import environ, getcwd
from pandas import DataFrame
from fedempy.modeler import FedemModeler
from fedempy.dts_operators.stress_visualization import run

if "FEDEM_SOLVER" not in environ or "VIS_EXPORTER" not in environ:
    print("  ** No visualization exporter library, test skipped")
    exit(0)  # Nothing to do here if no visualization exporter library

FM_FILE = "Loaderpy.fmm"

# Add solver options for performing stress recovery without frs-output
myModel = FedemModeler(FM_FILE)
myModel.fm_solver_setup(t_quasi=-1.0, t_inc=0.01, t_end=1.6,
                        add_opt="-partDeformation=0 -partVMStress=2 -allPrimaryVars-")
myModel.edit_part("Boom", recovery=3)  # activate stress- and gage recovery
myModel.edit_part("Bucket", recovery=1)  # activate stress recovery
myModel.close(True)

# Create dummy input dataframe (this model does not have external functions)
nsteps = 100
inputs = DataFrame({ "F0": [None] * nsteps }, index=range(1, 1+nsteps))

# Define the Exporter input
fe_parts = [{
    "path": "Loaderpy_RDB/link_DB/Front.ftl",
    "name": "Front",
    "base_id": 4,
    "recovery": False
}, {
    "path": "Loaderpy_RDB/link_DB/Boom.ftl",
    "name": "Boom",
    "base_id": 16,
    "recovery": True
}, {
    "path": "Loaderpy_RDB/link_DB/Bucket.ftl",
    "name": "Bucket",
    "base_id": 27,
    "recovery": True
}, {
    "path": "Loaderpy_RDB/link_DB/BellCrank.ftl",
    "name": "BellCrank",
    "base_id": 46,
    "recovery": False
}, {
    "path": "Loaderpy_RDB/link_DB/BucketLink.ftl",
    "name": "BucketLink",
    "base_id": 54,
    "recovery": False
}]

# Run the simulation over the first nsteps steps
print("\n#### Running dynamics solver with stress recovery on", FM_FILE)
run(inputs.copy(), fmm_file=FM_FILE, lib_dir=getcwd(), fe_parts=fe_parts)
