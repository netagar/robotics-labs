# author1:
# author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import time

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

    global grid, stopevent

    origin = robot.pose
    start = grid.getStart()
    goal_relative_to_cube = cozmo.util.Pose(-100, 0, 0, angle_z=cozmo.util.degrees(0))

    def pose_to_coords(pose):
        """Transforms a cozmo pose in world coordinates to the grid coordinates.

        Assuming that the robot starts at the given start location on the map and facing +x direction."""
        pose_relative_to_origin = get_relative_pose(pose, origin)
        (x0, y0) = start
        return round(pose_relative_to_origin.position.x / grid.scale) + x0, round(
            pose_relative_to_origin.position.y / grid.scale) + y0

    last_know_poses = [None, None, None]
    while not stopevent.is_set():
        cubes = [robot.world.get_light_cube(id) for id in cozmo.objects.LightCubeIDs]
        for i, cube in enumerate(cubes):
            if cube.is_visible:
                last_know_poses[i] = cube.pose

        # Update obstacles and goals on the map
        grid.clearObstacles()
        grid.clearGoals()
        for pose in last_know_poses:
            if pose:
                grid.addObstacle(pose_to_coords(pose))
        if last_know_poses[0]:
            grid.addGoal(pose_to_coords(last_know_poses[0].define_pose_relative_this(goal_relative_to_cube)))

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
