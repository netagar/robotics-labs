# author1:
# author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import time
from collections import deque

sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    goal = grid.getGoals()[0]

    # map from current location -> (estimated total cost, current cost, best path)
    best = {}

    def makeState(path, current_cost):
        """Creates a state object to put into the priority queue.

        A state is the tuple of (estimated total cost, current cost, path).
        """
        remaining_cost = heuristic(path[-1], goal)
        return current_cost + remaining_cost, current_cost, path

    def pushState(path, current_cost):
        """Pushes the state to the best mapping if it's better than the current state."""
        if path[-1] not in best or best[path[-1]][1] > current_cost:
            best[path[-1]] = makeState(path, current_cost)

    def findNext():
        """Finds the next unvisited location with the lowest estimated total cost."""
        visited = grid.getVisited()
        best_state = None
        for loc, state in best.items():
            if loc not in visited and (best_state is None or state < best_state):
                best_state = state
        return best_state

    pushState([grid.getStart()], 0)
    grid.clearVisited()
    grid.clearPath()
    while True:
        state = findNext()
        if state is None:
            break
        (_, current_cost, path) = state
        grid.addVisited(path[-1])
        if path[-1] == goal:
            grid.setPath(path)
            return
        neighbors = grid.getNeighbors(path[-1])
        for (neighbor, weight) in neighbors:
            new_path = path + [neighbor]
            pushState(new_path, current_cost + weight)


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """

    # Go diagonally as much as possible, then the rest straight.
    (x1, y1) = current
    (x2, y2) = goal
    a = abs(x1 - x2)
    b = abs(y1 - y2)

    return math.sqrt(2) * min(a, b) + max(a, b) - min(a, b)


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent, motion

    robot.set_lift_height(0).wait_for_completed()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Useful vars for the local functions.
    origin = robot.pose
    start = grid.getStart()
    goal_relative_to_cube = cozmo.util.Pose(-100, 0, 0, angle_z=cozmo.util.degrees(0))
    center_of_arena = (grid.width / 2, grid.height / 2)
    motion = None  # Current motion of the robot, if the robot is moving.
    goal_pose = None

    def pose_to_coords(pose):
        """Transforms a cozmo pose in world coordinates to the grid coordinates.

        Assuming that the robot starts at the given start location on the map and facing +x direction."""
        pose_relative_to_origin = get_relative_pose(pose, origin)
        (x0, y0) = start
        return round(pose_relative_to_origin.position.x / grid.scale) + x0, round(
            pose_relative_to_origin.position.y / grid.scale) + y0

    def coords_to_pose(coords, angle):
        (x, y) = coords
        (x0, y0) = start
        return origin.define_pose_relative_this(
            cozmo.util.pose_z_angle((x - x0) * grid.scale, (y - y0) * grid.scale, 0, angle))

    def stop_motion():
        global motion
        if motion is not None and motion.is_running:
            motion.abort()
        motion = None

    def direction(p1, p2):
        """Returns the direction vector from p1 to p2."""
        return p2[0] - p1[0], p2[1] - p1[1]

    def build_plan(path):
        """Given a path (list of coords), build a plan that goes through each key point on the path.

        Returns a list of poses."""
        plan = deque()
        last_point = None
        last_direction = None
        for coords in path:
            # The first point is the starting location. Don't need to add movement.
            if last_point is None:
                last_point = coords
                continue

            dir = direction(last_point, coords)
            if dir != last_direction:
                # Move to last point, facing current direction
                plan.append(coords_to_pose(last_point, cozmo.util.radians(math.atan2(dir[1], dir[0]))))
                last_direction = dir
            # Otherwise, we're continuing to the same direction
            last_point = coords

        if goal_pose is not None:
            plan.append(goal_pose)
        else:
            plan.append(coords_to_pose(last_point, cozmo.util.radians(0)))

        return plan

    last_known_coords = [None, None, None]  # Last known coordinates for the 3 cubes.
    plan = None  # A list of poses to go through in order to reach the goal.
    while not stopevent.is_set():
        robot_coords = pose_to_coords(robot.pose)

        cubes = [robot.world.get_light_cube(id) for id in cozmo.objects.LightCubeIDs]
        update_map = False  # Only update map in case cubes are moved.
        for i, cube in enumerate(cubes):
            if cube.is_visible:
                coords = pose_to_coords(cube.pose)
                if coords != last_known_coords[i]:
                    last_known_coords[i] = coords
                    update_map = True
                if i == 0:
                    goal_pose = cube.pose.define_pose_relative_this(goal_relative_to_cube)

        if update_map or plan is None:
            print("Cubes moved, updating map. Cube locations: " + str(last_known_coords))

            stop_motion()
            grid.clearObstacles()
            grid.clearGoals()
            grid.setStart(robot_coords)
            for coords in last_known_coords:
                if coords:
                    # Mark each cube as a 3x3 square to account for the radius of the robot.
                    grid.addObstacle(coords)
                    for neighbor, _ in grid.getNeighbors(coords):
                        grid.addObstacle(neighbor)

            if goal_pose:
                grid.addGoal(pose_to_coords(goal_pose))
            elif robot_coords != center_of_arena:
                # Drive to the center of the arena.
                grid.addGoal(center_of_arena)
            else:
                # Turn in place, 30 degrees at a time, and re-evaluate.
                robot.turn_in_place(cozmo.util.degrees(30)).wait_for_completed()
                continue

            # Now that we have a goal, replan the path.
            astar(grid, heuristic)
            plan = build_plan(grid.getPath())

            print("Plan: " + str(plan))

        # Follow the plan.
        if len(plan) == 0:
            if motion is not None:
                motion.wait_for_completed()
            stopevent.set()
            return

        if motion is None or motion.is_completed:
            nextpose = plan.popleft()
            print("Following plan to next pose: " + str(nextpose))
            motion = robot.go_to_pose(nextpose)

        time.sleep(0.1)


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()
