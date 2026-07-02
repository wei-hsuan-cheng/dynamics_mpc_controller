#!/usr/bin/env python3

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import ee_motion_tracking_target as ee_motion_target


# This is a contact-task example for inverse dynamics MPC.
# Required controller settings:
#   model_settings.wrenchInRNEA: true
#   model_settings.trackZeroWrench: false
#   eeContactFrictionConeSoftConstraint.activate: true

ee_motion_target.DEFAULT_COMMAND_TYPE = "ee_motion_pose"
ee_motion_target.DEFAULT_PUBLISH_EE_WRENCH = True
ee_motion_target.DEFAULT_EE_WRENCH_FRAME = "ee"  # "world" | "base" | "ee"

# Start with a reachable floor-contact pose template. Tune z/orientation for the
# actual tool geometry and contact point used in the MuJoCo scene.
ee_motion_target.DEFAULT_TRANSLATION_CENTER = np.array([0.45, 0.0, 0.0])
ee_motion_target.DEFAULT_TRANSLATION_AMPLITUDE = np.array([0.0, 0.0, 0.0])
ee_motion_target.DEFAULT_TRANSLATION_PHASE = np.array([0.0, 0.0, 0.0])

ee_motion_target.DEFAULT_ORIENTATION_RPY_CENTER = np.array([-np.pi / 1.0, 0.0, -np.pi / 2.0])
ee_motion_target.DEFAULT_ORIENTATION_RPY_AMPLITUDE = np.array([0.0, 0.0, 0.0])
ee_motion_target.DEFAULT_ORIENTATION_RPY_PHASE = np.array([0.0, 0.0, 0.0])

# EE wrench is represented in the world/base-aligned frame used by the ID RNEA
# wrench Jacobian. For floor contact, +Z is an upward normal force on the robot.
ee_motion_target.DEFAULT_EE_WRENCH_CENTER = np.array([0.0, 0.0, -20.0, 0.0, 0.0, 0.0])
ee_motion_target.DEFAULT_EE_WRENCH_AMPLITUDE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
ee_motion_target.DEFAULT_EE_WRENCH_PHASE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    ee_motion_target.main()
