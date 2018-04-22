#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import math
import numpy as np
from cozmo.util import degrees


def get_relative_pose(object_pose, reference_frame_pose):
	translation = np.matrix([[1, 0, 0, -reference_frame_pose.position.x],
							 [0, 1, 0, -reference_frame_pose.position.y],
							 [0, 0, 1, -reference_frame_pose.position.z],
							 [0, 0, 0, 1]])
	z_radian = -reference_frame_pose.rotation.angle_z.radians
	rotation_z = np.matrix([[math.cos(z_radian), -math.sin(z_radian), 0, 0],
							[math.sin(z_radian), math.cos(z_radian), 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]])
	new_position = (rotation_z * translation * np.matrix([[object_pose.position.x,
														   object_pose.position.y,
														   object_pose.position.z,
														   1]]).T).A1
	return cozmo.util.pose_z_angle(new_position[0], new_position[1], new_position[2],
								   cozmo.util.radians(object_pose.rotation.angle_z.radians + z_radian))


def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':
	cozmo.run_program(find_relative_cube_pose)
