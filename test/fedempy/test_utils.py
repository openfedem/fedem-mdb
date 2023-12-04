# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
Utility functions to facilitate regression testing of the Python wrapper.
"""

def diff_values(v1, v2, eps):
    """
    Compares two float values with some tolerance.
    """
    diff = abs(v1 - v2)
    refv = abs(v1 + v2) * eps * 0.5
    return diff if diff > refv and diff > eps else 0.0


def compare_lists(t, l1, l2, eps=1.0e-12):
    """
    Compares two lists of float values with some tolerance.
    """
    ndiff = 0
    for i in range(len(l1)):
        if isinstance(eps, list):
            epsc = eps[i] if i < len(eps) else eps[-1]
            diff = diff_values(l1[i], l2[i], epsc)
        else:
            diff = diff_values(l1[i], l2[i], eps)
        if diff > 0.0:
            print(f"t={t}: Value {i+1} does not match {l1[i]} {l2[i]} diff={diff}")
            ndiff += 1

    return ndiff
