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
    # Approach 1.
    # ####
    initial_rotation = math.degrees(math.atan2(y, x))
    final_rotation = angle_z - initial_rotation
    distance = math.sqrt(x * x + y * y)
    my_turn_in_place(robot, initial_rotation, 30)
    time.sleep(0.5)
    my_drive_straight(robot, distance, 30)
    time.sleep(0.5)
    my_turn_in_place(robot, final_rotation, 30)


def normalize(radians):
    """Normalize an angle to +/- 180 degrees so the robot don't make stupid 270 degree turns."""
    while radians < -math.pi:
        radians += 2 * math.pi
    while radians > math.pi:
        radians -= 2 * math.pi
    return radians


def my_go_to_pose2(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # Formulae in Correl book chapter 3.5
    # ####

    # Transform desired pose into world coordinates.
    goal = cozmo.util.pose_z_angle(x + robot.pose.position.x, y + robot.pose.position.y, robot.pose.position.z,
                                   cozmo.util.degrees(robot.pose.rotation.angle_z.degrees + angle_z))
    while True:
        print("Robot pose: {0}".format(robot.pose))
        print("Goal pose: {0}".format(goal))

        delta_x = goal.position.x - robot.pose.position.x
        delta_y = goal.position.y - robot.pose.position.y

        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        bearing = normalize(math.atan2(delta_y, delta_x) - robot.pose.rotation.angle_z.radians)
        heading = normalize(goal.rotation.angle_z.radians - robot.pose.rotation.angle_z.radians)
        print("Distance = {0}; Bearing = {1}; Heading = {2}".format(distance, math.degrees(bearing),
                                                                    math.degrees(heading)))

        if distance < 8 and abs(heading) < 0.1:
            robot.stop_all_motors()
            return

        # Shorter distance -> more focus on heading
        # Longer distance -> more focus on bearing
        p1 = 0.2
        if distance < 20:
            p2 = 0.1
            p3 = 0.3
        elif distance < 80:
            p2 = 0.2
            p3 = 0.2
        else:
            p2 = 0.3
            p3 = 0.1

        drive_speed = min(p1 * distance, 30)
        rotation_speed = p2 * bearing + p3 * heading
        print("Drive speed = {0}; Rotation speed = {1}".format(drive_speed, rotation_speed))

        wheel_speed_adj = rotation_speed * get_distance_between_wheels() / 2
        left_speed = drive_speed - wheel_speed_adj
        right_speed = drive_speed + wheel_speed_adj
        print("Left speed = {0}; Right speed = {1}".format(left_speed, right_speed))

        robot.drive_wheels(left_speed, right_speed)
        if abs(left_speed) > 5 or abs(right_speed) > 5:
            time.sleep(0.1)
        else:
            time.sleep(1)


def my_go_to_pose3(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # Transform desired pose into world coordinates.
    goal = cozmo.util.pose_z_angle(x + robot.pose.position.x, y + robot.pose.position.y, robot.pose.position.z,
                                   cozmo.util.degrees(robot.pose.rotation.angle_z.degrees + angle_z))
    delta_x = goal.position.x - robot.pose.position.x
    delta_y = goal.position.y - robot.pose.position.y
    bearing = normalize(math.atan2(delta_y, delta_x) - robot.pose.rotation.angle_z.radians)

    if abs(bearing) > math.pi / 2:
        # Goal behind robot. Use strategy 1
        my_go_to_pose1(robot, x, y, angle_z)
    else:
        # Goal ahead of robot. Use strategy 2
        my_go_to_pose2(robot, x, y, angle_z)


def run(robot: cozmo.robot.Robot):
    print("***** Back wheel radius: " + str(get_front_wheel_radius()))
    print("***** Distance between wheels: " + str(get_distance_between_wheels()))

    ## Example tests of the functions

    # cozmo_drive_straight(robot, 87, 10)
    # cozmo_turn_in_place(robot, 90, 30)
    #cozmo_go_to_pose(robot, 100, 100, -90)
    #
    # rotate_front_wheel(robot, 360)
    # my_drive_straight(robot, 62, 50)
    # my_turn_in_place(robot, 45, 30)
    # my_turn_in_place(robot, -45, 30)
    #
    # my_go_to_pose1(robot, 100, 100, 0)
    # my_go_to_pose2(robot, 100, 100, 0)
    my_go_to_pose3(robot, 100, 100, -90)


if __name__ == '__main__':
    cozmo.run_program(run)
