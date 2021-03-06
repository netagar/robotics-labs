#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball

import cozmo

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)


    try:
        looking_around = None
        robot.set_lift_height(0)

        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            h = opencv_image.shape[0]
            w = opencv_image.shape[1]

            #find the ball
            ball = find_ball.find_ball(opencv_image)

            #set annotator ball
            BallAnnotator.ball = ball

            if np.array_equal(ball, [0, 0, 0]):
                if not looking_around:
                    robot.stop_all_motors()
                    looking_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            else:
                # Stop moving around
                if looking_around:
                    looking_around.stop()
                    looking_around = None

                # If reached the ball (by checking the visible radius), lift it
                if ball[2] > 95:
                    robot.stop_all_motors()
                    # Drive the final distance in a straight line (we'll lose sight of the ball when it's too close)
                    await robot.drive_straight(cozmo.util.distance_mm(30),
                                               cozmo.util.speed_mmps(10)).wait_for_completed()
                    await robot.set_lift_height(1.0).wait_for_completed()
                    return

                # Move head to center the ball along the y axis
                # Ball on the top -> head_speed = 0.5
                # Ball on the bottom -> head_speed = -0.5
                head_speed = 0.5 - ball[1] / h
                print("Ball = {0}, Head speed = {1}".format(ball, head_speed))
                robot.move_head(head_speed)

                # Drive towards the ball like a Braitenberg vehicle.
                # Larger radius -> slower
                speed = 300 / ball[2]

                # Ball on the left -> diff = -0.5
                # Ball on the right -> diff = 0.5
                diff = ball[0] / w - 0.5

                left_speed = speed * (1 + diff)
                right_speed = speed * (1 - diff)

                print("Speed = {0}, Diff = {1}, Left = {2}, Right = {3}".format(
                    speed, diff, left_speed, right_speed))
                robot.drive_wheel_motors(left_speed, right_speed)



    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

