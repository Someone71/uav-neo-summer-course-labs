"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Seek the Gate
Yaw to center the largest cyan gate, then fly forward toward it.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
MIN_AREA   = 400
COL_CENTER = 320
MAX_YAW    = 0.3        # yaw authority for centering
APPROACH_PITCH = 0.2    # forward speed once centered
CENTER_TOL = 90         # px error to count as centered
SEARCH_YAW = 0.2        # spin slowly when no gate is seen
TARGET_WIDTH = 170      # gate this wide (px) => close enough

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done
    _done = False


def update(drone):
    global _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: spin until a cyan gate is found, yaw to center it, then fly forward until
    # it fills the view (bounding-box width >= TARGET_WIDTH).
    #
    # Tools: drone.camera.get_color_image(); neo_lab.largest_cyan_gate(image, MIN_AREA);
    #        cv2.boundingRect(contour) -> (x, y, w, h); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle).
    #
    # No gate in view -> turn slowly (SEARCH_YAW) to find one. With a gate, the box
    # center column vs. COL_CENTER gives a yaw error; only add forward pitch once it is
    # roughly centered (within CENTER_TOL) so you turn toward it before chasing. The box
    # grows as you approach; stop when w reaches TARGET_WIDTH.

    image = drone.camera.get_color_image()
    gate = neo_lab.largest_cyan_gate(image, MIN_AREA)

    if gate is None:
        drone.flight.send_pcmd(0, 0, SEARCH_YAW, 0)
    else:
        x, __, w, __ = cv2.boundingRect(gate)
        center = x + w/2
        err = center - COL_CENTER

        if err >= CENTER_TOL:
            yaw = uav_utils.clamp(err / COL_CENTER * MAX_YAW, -MAX_YAW, MAX_YAW)
            drone.flight.send_pcmd(0, 0, yaw, 0)
        else:
            if abs(w) < TARGET_WIDTH:
                drone.flight.send_pcmd(APPROACH_PITCH, 0, 0, 0)
            else:
                _done = True
                print("Found gate")
                drone.flight.stop()

    ###### END PUT CODE HERE #########
    ##################################
    return _done



if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Seek the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
