"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 2 — Step 1: Fly a Square
A flight path is just a list of waypoints visited in order. Here the list forms a
square. Reuse the waypoint controller from Module 1, one corner at a time, advancing
to the next corner when you reach the current one.
"""

import drone_core
import drone_utils as uav_utils

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
SIDE = 3.0
# Corners as (right, forward) meters from the start, traced as a square.
WAYPOINTS = [(0.0, SIDE), (SIDE, SIDE), (SIDE, 0.0), (0.0, 0.0)]
TARGET_HEIGHT = 3.0
KP_POS = 0.18
KD_POS = 0.5
ALT_KP = 0.12
ROLL_LIMIT = 0.25
PITCH_LIMIT = 0.25
THROTTLE_LIMIT = 0.5
WP_TOL = 0.6           # meters from a corner counted as reached

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_wp = 0
_done = False

def reset():
    global _x, _z, _wp, _done
    _x = 0.0
    _z = 0.0
    _wp = 0
    _done = False


def update(drone):
    global _x, _z, _wp, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: visit each corner in WAYPOINTS in order, then finish.
    #
    # Tools: drone.physics.get_linear_velocity(); drone.get_delta_time();
    #        neo_lab.height(drone); uav_utils.clamp(...); drone.flight.send_pcmd(...).
    #
    # Integrate vx, vz into (_x, _z) like Module 1. If _wp has passed the last
    # waypoint, stop and finish. Otherwise steer toward WAYPOINTS[_wp] with the same
    # PD command per axis (roll for right, pitch for forward, throttle for height).
    # When you are within WP_TOL of the current corner on both axes, advance _wp += 1.

    dt = drone.get_delta_time()
    vx, vy, vz = drone.physics.get_linear_velocity()

    _x += vx * dt
    _z += vz * dt
    _y  = neo_lab.height(drone)

    err_x = WAYPOINTS[_wp][0] - _x
    err_z = WAYPOINTS[_wp][1] - _z
    err_y = TARGET_HEIGHT - _y

    pitch = uav_utils.clamp(err_z * KP_POS - vz * KD_POS, -PITCH_LIMIT, PITCH_LIMIT)
    roll = uav_utils.clamp(err_x * KP_POS - vx * KD_POS, -ROLL_LIMIT, ROLL_LIMIT)
    throttle = uav_utils.clamp(err_y * ALT_KP - vy, -THROTTLE_LIMIT, THROTTLE_LIMIT)

    drone.flight.send_pcmd(pitch, roll, 0, throttle)

    if abs(err_x) <= WP_TOL and abs(err_z) <= WP_TOL:
        print("Next step")
        _wp += 1

    if _wp == 4:
        print("Done")
        drone.flight.stop()
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
        print("Step 1: Fly a Square")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
