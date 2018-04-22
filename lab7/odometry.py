#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math

import time

# Add duration to drive_wheels to account for warming up.
DURATION_WARMUP = 0.6

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """
    robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
    """Returns the radius of the Cozmo robot's front wheel in millimeters."""
    # ####
    # I moved the robot by 88mm and observed that the front wheel turned a full turn.
    # ####
    return 88 / (2 * math.pi)

def get_distance_between_wheels():
    """Returns the distance between the wheels of the Cozmo robot in millimeters."""
    # ####
    # I drove the robot with different speeds on the left/right wheel and used the delta in z-rotation
    # to compute the width (b):
    # b = duration * (left_speed - right_speed) / z-delta
    # The result is unstable (ranging from 80mm to 120mm) and not consistent with the distance measured with a ruler.
    # The reason might be
    # 1) robot.drive_wheels() doesn't respect the speed and duration passed in perfectly.
    #    I have observed that there's a small start-up time when the wheels accelerate to the desired speed,
    #    and at low speeds the wheels move only intermittently.
    # 2) The "wheels" are actually treads, and they work differently. I have observed that even if I keep one wheel at 0
    #    speed, the robot won't turn around that wheel but actually turns around a center of rotation somewhere farther.
    #
    # Code sample:
    # initial_rotation = robot.pose.rotation.angle_z.radians
    # left_speed = 40
    # right_speed = 15
    # duration = 10
    # robot.drive_wheels(left_speed, right_speed, duration=duration)
    # rotation_delta = robot.pose.rotation.angle_z.radians - initial_rotation
    # width = duration * (left_speed - right_speed) / rotation_delta
    # print("Rotation delta = {0}. Empirical width = {1}".format(rotation_delta, width))
    # ####
    return 88

def rotate_front_wheel(robot, angle_deg):
    """Rotates the front wheel of the robot by a desired angle.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle_deg -- Desired rotation of the wheel in degrees
    """
    my_drive_straight(robot, math.radians(angle_deg) * get_front_wheel_radius(), speed=30)

def my_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    duration = dist / speed + DURATION_WARMUP
    robot.drive_wheels(speed, speed, duration=duration)

def my_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """
    duration = abs(angle / speed) + DURATION_WARMUP
    wheel_speed = math.radians(speed) * get_distance_between_wheels() / 2
    if angle > 0:
        robot.drive_wheels(-wheel_speed, wheel_speed, duration=duration)
    else:
        robot.drive_wheels(wheel_speed, -wheel_speed, duration=duration)

def my_go_to_pose1(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # using the my_drive_straight and my_turn_in_place functions. This should
    # include a sequence of turning in place, moving straight, and then turning
    # again at the target to get to the desired rotation (Approach 1).
    # ####
    initial_rotation = math.degrees(math.atan2(y, x))
    final_rotation = angle_z - initial_rotation
    distance = math.sqrt(x * x + y * y)
    my_turn_in_place(robot, initial_rotation, 30)
    time.sleep(0.5)
    my_drive_straight(robot, distance, 30)
    time.sleep(0.5)
    my_turn_in_place(robot, final_rotation, 30)

def my_go_to_pose2(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # using the robot.drive_wheels() function to jointly move and rotate the
    # robot to reduce distance between current and desired pose (Approach 2).
    # ####
    pass

def my_go_to_pose3(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # as fast as possible. You can experiment with the built-in Cozmo function
    # (cozmo_go_to_pose() above) to understand its strategy and do the same.
    # ####
    pass

def run(robot: cozmo.robot.Robot):

    print("***** Back wheel radius: " + str(get_front_wheel_radius()))
    print("***** Distance between wheels: " + str(get_distance_between_wheels()))

    ## Example tests of the functions

    # cozmo_drive_straight(robot, 87, 10)
    #cozmo_turn_in_place(robot, 90, 30)
    #cozmo_go_to_pose(robot, 100, 100, 0)
    #
    #rotate_front_wheel(robot, 360)
    # my_drive_straight(robot, 62, 50)
    # my_turn_in_place(robot, 45, 30)
    # my_turn_in_place(robot, -45, 30)
    #
    my_go_to_pose1(robot, 100, 100, 0)
    # my_go_to_pose2(robot, 100, 100, 45)
    # my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

    cozmo.run_program(run)



