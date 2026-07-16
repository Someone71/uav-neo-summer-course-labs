"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 7 — Step 2: Velocity from Optical Flow
Turn the tracked features' average motion into a velocity estimate and compare it
against the drone's true velocity. The flow is in pixels per processed interval;
converting to meters/second needs the ground footprint of one pixel (grows with
altitude) and the time between processed frames.
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
IMAGE_WIDTH = 640
HFOV_TAN = 1.0         # tan(half of a 90 deg horizontal field of view)
PROBE_PITCH = 0.12     # forward drift to create measurable flow
RUN_TIME = 6.0
SKIP = 2               # do the vision work every Nth frame
MIN_PTS = 20
FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.01, minDistance=8, blockSize=7)
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# -- Module-level state -----------------------------------------------------
_prev_gray = None
_prev_pts = None
_timer = 0.0
_interval = 0.0        # time accumulated since the last processed frame
_frame = 0
_done = False

def reset():
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    _prev_gray = None
    _prev_pts = None
    _timer = 0.0
    _interval = 0.0
    _frame = 0
    _done = False


def update(drone):
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: print an estimated horizontal velocity from optical flow next to the true
    # velocity, so you can see how well vision tracks motion.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.height(drone);
    #        drone.physics.get_linear_velocity(); drone.get_delta_time(); send_pcmd(...);
    #        plus the sparse optical-flow tracking you built in Step 1.
    #
    # Every frame: drift (PROBE_PITCH), add dt to _timer AND to _interval, and _frame += 1.
    # Only every SKIP-th frame: track corner points (sparse flow, like Step 1) and average
    # the kept points' displacement in pixels. Convert that to meters/second: one pixel's
    # ground footprint grows with height and the camera's field of view (use HFOV_TAN and
    # IMAGE_WIDTH), and divide by _interval (the time between PROCESSED frames, not one dt);
    # then reset _interval. The camera moves opposite the scene flow (sign flip). Finish at
    # RUN_TIME, printing the estimate vs. true velocity. See the README (Key terms).

    drone.flight.send_pcmd(PROBE_PITCH, 0, 0, 0)
    _frame += 1
    dt = drone.get_delta_time()
    _timer += dt
    _interval += dt

    if _frame % SKIP == 0:
        image = drone.camera.get_downward_image()
        image_gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if _prev_gray is None or _prev_pts is None or len(_prev_pts) < MIN_PTS:
            _prev_pts = cv2.goodFeaturesToTrack(image_gs, **FEATURE_PARAMS)
        else:
            points, status, err = cv2.calcOpticalFlowPyrLK(_prev_gray, image_gs, _prev_pts, None, **LK_PARAMS)
            
            if points is not None and status is not None:
                good = status.flatten() == 1
                good_new_points = points[good].reshape(-1, 2)
                good_old_points = _prev_pts[good].reshape(-1, 2)

                if len(good_new_points) > 0:
                    displacement = good_new_points - good_old_points

                    displacement_x = displacement[:,0].mean()
                    displacement_y = displacement[:,1].mean()
                    meters_per_pixel = 2 * neo_lab.height(drone) * HFOV_TAN / IMAGE_WIDTH

                    velocity_x = meters_per_pixel * displacement_x / _interval
                    velocity_y = meters_per_pixel * displacement_y / _interval
                    x, y, z = drone.physics.get_linear_velocity()

                    if _timer >= RUN_TIME:
                        print(neo_lab.height(drone))
                        print(f"Estimated velocity: {velocity_x:.2f}, {velocity_y:.2f} \n True velocity: {x:.2f}, {z:.2f}")
                        _done = True
                _prev_pts = good_new_points.reshape(-1, 1, 2)

        _prev_gray = image_gs
        _interval = 0





    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Velocity from Optical Flow")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
