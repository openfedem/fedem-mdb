# SPDX-FileCopyrightText: 2023 SAP SE
#
# SPDX-License-Identifier: Apache-2.0
#
# This file is part of FEDEM - https://openfedem.org

"""
This module creates a simple beam model of a excavator using fedempy.
The hydraulic cylinders are represented by two beams each connected
by a revolute joint with prescribed Tz-motion.

The test uses one environment variable:
    FEDEM_MDB = Full path to the Fedem model database shared object library
"""

from fedempy.modeler import FedemModeler
from fedempy.enums import FmDofStat, FmType


# Create a new, empty model
fmmfile = "Gravemaskin.fmm"
myModel = FedemModeler(fmmfile, True)


def create_model():

    # Create triads defining the model hard-points
    triads = [myModel.make_triad("Origin", (0, 0, 0))] # 0
    triads.append(myModel.make_triad("T0", (0, 0, 0)))
    triads.append(myModel.make_triad("T1", (1, 3, 0)))
    triads.append(myModel.make_triad("T1", (1, 3, 0)))
    triads.append(myModel.make_triad("T1", (1, 3, 0)))
    triads.append(myModel.make_triad("T2", (2, 6, 0))) # 5
    triads.append(myModel.make_triad("T2", (2, 6, 0)))
    triads.append(myModel.make_triad("T7", (2, 4, 0)))
    triads.append(myModel.make_triad("T7", (2, 4, 0)))
    triads.append(myModel.make_triad("T3", (2, 7, 0))) # 9
    triads.append(myModel.make_triad("T3", (2, 7, 0)))
    triads.append(myModel.make_triad("T4", (2, 2, 0))) # 11
    triads.append(myModel.make_triad("T4", (2, 2, 0)))
    triads.append(myModel.make_triad("T5", (2.5, 2, 0))) # 13
    triads.append(myModel.make_triad("T5", (2.5, 2, 0)))
    triads.append(myModel.make_triad("Tip", (1.3, 2, 0))) # 15
    triads.append(myModel.make_triad("T6", (0, 1, 0))) # 16
    triads.append(myModel.make_triad("T6", (0, 1, 0)))
    triads.append(myModel.make_triad("Cyl1", (0.9, 2.8, 0.0))) # 18
    triads.append(myModel.make_triad("Rod1", (0.1, 1.2, 0.0)))
    triads.append(myModel.make_triad("Cyl2", (1.6, 5.4, 0.0)))
    triads.append(myModel.make_triad("Rod2", (1.4, 4.6, 0.0)))
    triads.append(myModel.make_triad("Cyl3", (2.35, 2.6, 0.0)))
    triads.append(myModel.make_triad("Rod3", (2.15, 3.4, 0.0))) # 23
    triads.append(myModel.make_triad("Base", (-0.1, 0.0, 0.0)))
    print("Created triads", triads)
    if any(i <= 0 for i in triads):
        return -1

    # Create a material property               Rho   E       nu
    mat = myModel.make_beam_material("steel", (7850, 2.1e11, 0.3))
    print("Created material property", mat)
    if mat <= 0:
        return -2

    # Create a pipe cross section                  Do   Di
    pipe = myModel.make_beam_section("pipe", mat, (0.2, 0.0))
    print("Created beam property", pipe)
    if pipe <= 0:
        return -3

    # Create the three excavator booms
    beams = myModel.make_beam("B1", [triads[1], triads[2], triads[5]], pipe)
    beams.extend(myModel.make_beam("B2", [triads[9], triads[6], triads[7], triads[11]], pipe))
    beams.extend(myModel.make_beam("B3", [triads[13], triads[12], triads[15]], pipe))
    # Create the three cylinders
    beams.extend(myModel.make_beam("Cyl1", (triads[17], triads[18]), pipe))
    beams.extend(myModel.make_beam("Rod1", (triads[3], triads[19]), pipe))
    beams.extend(myModel.make_beam("Cyl2", (triads[4], triads[20]), pipe))
    beams.extend(myModel.make_beam("Rod2", (triads[10], triads[21]), pipe))
    beams.extend(myModel.make_beam("Cyl3", (triads[8], triads[22]), pipe))
    beams.extend(myModel.make_beam("Rod3", (triads[14], triads[23]), pipe))
    beams.extend(myModel.make_beam("Base", (triads[24], triads[0], triads[16]), pipe))
    print("Created beams", beams)
    if any(i <= 0 for i in beams):
        return -4

    # Assign stiffness-proportional damping
    myModel.edit_part(beams, alpha2=0.05)

    # Create joints connecting the beams
    joints = [myModel.make_joint("Rot", FmType.REVOLUTE_JOINT, triads[24])]
    joints.append(myModel.make_joint("J0", FmType.REVOLUTE_JOINT, triads[1], triads[0]))
    joints.append(myModel.make_joint("J1", FmType.REVOLUTE_JOINT, triads[6], triads[5]))
    joints.append(myModel.make_joint("J2", FmType.REVOLUTE_JOINT, triads[11], triads[12]))
    joints.append(myModel.make_joint("J3", FmType.REVOLUTE_JOINT, triads[17], triads[16]))
    joints.append(myModel.make_joint("J4", FmType.REVOLUTE_JOINT, triads[3], triads[2]))
    joints.append(myModel.make_joint("J5", FmType.REVOLUTE_JOINT, triads[4], triads[2]))
    joints.append(myModel.make_joint("J6", FmType.REVOLUTE_JOINT, triads[10], triads[9]))
    joints.append(myModel.make_joint("J7", FmType.REVOLUTE_JOINT, triads[14], triads[13]))
    joints.append(myModel.make_joint("J8", FmType.REVOLUTE_JOINT, triads[8], triads[7]))
    joints.append(myModel.make_joint("Cyl1", FmType.REVOLUTE_JOINT, triads[18], triads[19]))
    joints.append(myModel.make_joint("Cyl2", FmType.REVOLUTE_JOINT, triads[20], triads[21]))
    joints.append(myModel.make_joint("Cyl3", FmType.REVOLUTE_JOINT, triads[22], triads[23]))
    print("Created joints", joints)

    # Create some motion functions
    func = [myModel.make_function("Y-rotation", start_ramp=4.0, end_ramp=5.0, slope=1.6)]
    func.append(myModel.make_function("C1", xy = [[0.2, 0.0], [2.0, -1.0], [3.0, -1.0], [4.0, 0.0]], extrapol_type = "FLAT"))
    func.append(myModel.make_function("C2", xy = [[0.1, 0.0], [2.0, -1.5], [3.0,  0.0], [5.0, 0.0], [6.0, -1.0]], extrapol_type = "FLAT"))
    func.append(myModel.make_function("C3", xy = [[0.3, 0.0], [2.0,  0.5], [3.0, -0.2], [5.0,-0.2], [6.0,  0.3]], extrapol_type = "FLAT"))
    print("Created functions", func)
    if any(i <= 0 for i in func):
        return -5

    # Assign joint properties,
    # aligning the cylinder joints with the beams they are attached to, etc.
    myModel.edit_joint(joints[0], Rx=90,
                       constraints={"Rz" : FmDofStat.PRESCRIBED},
                       motion={"Rz" : func[0]})
    myModel.edit_joint(joints[10], Ry=90, tra_ref=beams[7], rot_ref=beams[7],
                       constraints={"Tz" : FmDofStat.PRESCRIBED},
                       motion={"Tz" : func[1]})
    myModel.edit_joint(joints[11], Ry=90, tra_ref=beams[9], rot_ref=beams[10],
                       constraints={"Tz" : FmDofStat.PRESCRIBED},
                       motion={"Tz" : func[2]})
    myModel.edit_joint(joints[12], Ry=90, tra_ref=beams[11], rot_ref=beams[11],
                       constraints={"Tz" : FmDofStat.PRESCRIBED},
                       motion={"Tz" : func[3]})

    # Define some solver setup, dynamics simulation for t=0,0.01,0.02,...7.0
    myModel.fm_solver_setup(t_inc = 0.01, t_end = 7.0)

    # Save the model with its given name
    if not myModel.save():
        return -6

    # Clean up
    myModel.close()

    return 0


stat = create_model()
if stat < 0:
    myModel.close()
    print("**** Model generation failed, please check the log-file",
          fmmfile.replace(".fmm", ".log"))
    exit(-stat)
