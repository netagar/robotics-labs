
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    goal = grid.getGoals()[0]
    frontier = PriorityQueue()

    def makeState(path, current_cost):
        """Creates a state object to put into the priority queue.

        A state is the tuple of (estimated total cost, current cost, path).
        """
        remaining_cost = heuristic(path[-1], goal)
        return current_cost + remaining_cost, current_cost, path

    frontier.put(makeState([grid.getStart()], 0))
    while not frontier.empty():
        (_, current_cost, path) = frontier.get()
        grid.addVisited(path[-1])
        if path[-1] == goal:
            grid.setPath(path)
            return
        neighbors = grid.getNeighbors(path[-1])
        for (neighbor, weight) in neighbors:
            new_path = path + [neighbor]
            frontier.put(makeState(new_path, current_cost + weight))


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """

    # Go diagonally as much as possible, then the rest straight.
    (x1, y1) = current
    (x2, y2) = goal
    a = abs(x1-x2)
    b = abs(y1-y2)

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
    
    while not stopevent.is_set():
        pass # Your code here


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

