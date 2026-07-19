"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Visual Servoing (Vision + PID)
Capstone: use a PID loop on the camera pixel error to keep a glowing
gate centered by yawing. Combines Week 2 vision with Week 3 control.
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
V_MIN = 200
MIN_AREA = 500
COL_CENTER = 320
KP = 0.35
KI = 0.0
KD = 0.2
MAX_YAW = 0.25
SEARCH_YAW = 0.2
CENTER_TOL = 0.15    # normalized error considered centered
HOLD_TIME = 1.0

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_target_col = None
_hold = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = err * kp + err_int + ki + err_dot * kd
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _err_int, _prev_err, _target_col, _hold, _done
    _err_int = 0.0
    _prev_err = 0.0
    _target_col = None
    _hold = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _target_col, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: yaw with a PID loop so a glowing gate stays centered in the forward
    # camera; finish once it is centered (abs(error) < CENTER_TOL) for HOLD_TIME.
    #
    # Available helpers: drone.camera.get_color_image(); drone.get_delta_time();
    #   neo_lab.gate_nearest_center(...) and neo_lab.gate_nearest_to(...) to find gates;
    #   uav_utils.get_contour_center; uav_utils.clamp; your pid_control() above.
    #
    # Lock onto ONE gate (store its column in _target_col) so the target does not jump
    # between gates. Turn the gate's horizontal offset from the image center into a
    # normalized error, PID it to a yaw command clamped to MAX_YAW, and sweep at SEARCH_YAW
    # when no gate is in view. See the README (Key terms) and Week 2 for finding gates.

    dt = drone.get_delta_time()
    image = drone.camera.get_color_image()

    if _target_col is None:
        gate = neo_lab.gate_nearest_center(image, V_MIN, MIN_AREA)
    else:
        gate = neo_lab.gate_nearest_to(image, _target_col, V_MIN, MIN_AREA)

    if gate is None:
        _target_col = None
        _err_int = 0
        _prev_err = 0
        _hold = 0
        drone.flight.send_pcmd(0, 0, SEARCH_YAW, 0)
    else:
        __, _target_col = uav_utils.get_contour_center(gate)

        err = (_target_col - COL_CENTER) / COL_CENTER
        _err_int += err * dt
        err_d = (err - _prev_err) / dt

        _prev_err = err

        yaw = uav_utils.clamp(pid_control(err, _err_int, err_d, KP, KI, KD), -MAX_YAW, MAX_YAW)

        drone.flight.send_pcmd(0, 0, yaw, 0)

        if abs(err) <= CENTER_TOL:
            _hold += dt
        else:
            _hold = 0
        
        if _hold >= HOLD_TIME:
            print("Finished step 3")
            _done = True
    

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Visual Servoing (Vision + PID)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
