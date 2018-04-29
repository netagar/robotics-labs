#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

from odometry import *

sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose


def move_relative_to_cube(robot: cozmo.robot.Robot):
    '''Looks for a cube while sitting still, when a cube is detected it
    moves the robot to a given pose relative to the detected cube pose.'''

    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    cube = None

    while cube is None:
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=30)
            if cube:
                print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
        except asyncio.TimeoutError:
            print("Didn't find a cube")

    desired_pose_relative_to_cube = Pose(0, 300, 0, angle_z=degrees(-90))

    cube_pose_relative_to_robot = get_relative_pose(cube.pose, robot.pose)
    desired_pose_relative_to_robot = cube_pose_relative_to_robot.define_pose_relative_this(
        desired_pose_relative_to_cube)

    my_go_to_pose3(robot, desired_pose_relative_to_robot.position.x, desired_pose_relative_to_robot.position.y,
                   desired_pose_relative_to_robot.rotation.angle_z.degrees)


if __name__ == '__main__':
    cozmo.run_program(move_relative_to_cube)
