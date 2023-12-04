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

from argparse import ArgumentParser
from fedempy.fmm_solver import FmmSolver
from fedempy.modeler import FedemModeler, FmDof, FmDofStat, FmVar


def make_model (fmm_name, plugin=None):
    """
    This function generates a Fedem model with user-defined (Bar) elements.
    """
    # Create a new, empty model
    myModel = FedemModeler(fmm_name, True, plugin)

    # Create some triads
    t1 = myModel.make_triad("Joint", (1, 0, 0))
    t2 = myModel.make_triad("Fixed", (0, 0, 0))
    t3 = myModel.make_triad("Tip"  , (1, 1, 0))
    print("Created triads", [t1, t2, t3])

    # Fix the second triad
    myModel.edit_triad(t2, constraints={"All" : FmDofStat.FIXED})

    # Create two user-defined bar elements
    elms = myModel.make_udelm("#Property 1000 10", [t2, t1, t3], alpha1=0.03, alpha2=0.05)
    print("Created user-defined elements", elms)

    # Create a sensor measuring the displacement in t2
    s1 = myModel.make_sensor("Forskyvning", t2, FmVar.POS, FmDof.TY)
    print("Created sensor", s1)

    # Save the model with its given name and clean up
    return myModel.close(True, True)


def main(fmm_file, plugin, solve=False):
    """Main driver."""
    if not make_model(fmm_file, plugin):
        exit(1)
    elif solve:
        mySolver = FmmSolver()
        exit(mySolver.solve_all(fmm_file, True))


if __name__ == "__main__":
    parser = ArgumentParser(description="PythonAPI User-defined element test")
    parser.add_argument("-f", "--fmm-file", default="ubar.fmm", help="Fedem model file")
    parser.add_argument("-p", "--plugin", required=True, help="Plugin library")
    parser.add_argument("-s", "--solve", action="store_true", help="Solves the model")
    main(**vars(parser.parse_args()))
