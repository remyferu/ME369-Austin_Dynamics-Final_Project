import random
import time


# creates a node class
class Node:

    def __init__(self, pose):
        self.pose = pose
        self.parent = 0
        # placeholder attributes that will be updated as the node called in the algorithm
        self.g = float('inf')
        self.h = float('inf')
        self.f = self.g + self.h

    # heuristic evaluation
    def h_eval(self, goal):
        # uses manhattan distance to calculate a fitness score
        dx = self.pose[0] - goal[0]
        dy = self.pose[1] - goal[1]
        self.h = abs(dx) + abs(dy)

    # cost evaluation
    def g_eval(self):
        self.g = self.parent.g + 1

    # fitness evaluation
    def f_eval(self, goal):
        self.h_eval(goal)
        self.g_eval()
        self.f = self.h + self.g


def find_adjacent(node, check_list):
    x = node.pose[0]
    y = node.pose[1]
    return [Node((x+i, y+j)) for (i, j) in [(1, 0), (-1, 0), (0, 1), (0, -1)]if (x+i, y+j) in check_list]


def A_star(destination, start_location, pose_list):
    time_i = time.time()

    # creates the open list containing only the start location node
    open_list = [Node(start_location)]

    # manually sets the fitness
    open_list[0].h_eval(start_location)
    open_list[0].g = 0
    open_list[0].f = open_list[0].h

    closed_list = []
    destination_reached = False

    while not destination_reached:
        # sorts open_list and grabs the node with the lowest f value (cheapest node to move to)
        node = sorted(open_list, key=lambda x: x.f)[0]

        # prevents the script from running for too long in case of unsolvable path
        if time.time() - time_i > 10:
            return node, TimeoutError


        # checks if current node is the destination
        if node.pose == destination:
            destination_reached = True
            return node, False
        else:
            # removes current node from the open_list (will be replaced with neighbors)
            open_list.pop(open_list.index(node))
            for adjacent_node in find_adjacent(node, pose_list):
                # print('evaluating node at:', adjacent_node.pose)

                # cost to travel to the adjacent node through the current node
                g_tentative = node.g + 1

                # checks if teh adjacent_node can be moved to cheaply (node initialized with inf cost)
                if g_tentative < adjacent_node.g:
                    adjacent_node.parent = node  # sets current node as adjacent node parent
                    adjacent_node.f_eval(destination)  # evaluates fitness metrics (f,g,h)
                    if adjacent_node not in open_list:
                        open_list.append(adjacent_node)  # adds adjacent node to open list if not already there

    print('unable to complete')
    return False


# un-nests the solution node using recursion
def node_extraction(node):
    node_list = []
    if node.parent == 0:
        return [node]
    else:
        for n in node_extraction(node.parent):
            node_list.append(n)
        node_list.append(node)
        return node_list


# extracts the position from the solution node_list
def pos_extraction(node_list):
    pos_list = []
    for node in node_list:
        pos_list.append(node.pose)
    return pos_list


# creates the MAP indices (valid positions in a grid)
def MAP_make(Column, Row, fill, start, end):
    Map = []
    # iterates over every column and row by a given step
    for i in range(0, Column):
        for j in range(0, Column):
            # converts coordinate systems from a corner (0,0) to a central (0,0)
            x = i - (Column - 1) / 2
            y = j - (Row - 1) / 2

            Map.append((x, y))

    # empties the MAP according to the fill percent
    for i in range(0, int(len(Map) * fill)):
        Map.pop(random.randint(0, int(len(Map)) - 1))

    # ensures the presence of the start and final positions
    if (0, 0) not in Map:
        Map.append((0, 0))
    if (7, 7) not in Map:
        Map.append((7, 7))

    return Map


# prints the map to the terminal
def MAP_vis(Column, Row, Map, Results=[]):
    print('')
    MAP_vis = ''
    # prints out a MAP visualization
    for i in range(0, Row):
        for j in range(0, Row):
            x = i - (Column - 1) / 2
            y = j - (Row - 1) / 2

            if (x, y) in Results:
                MAP_vis += ' o '
            elif (x, y) in Map:
                MAP_vis += '   '
            else:
                MAP_vis += ' # '

        # prints out current row
        print(MAP_vis)
        # resets map_vis
        MAP_vis = ''
    print('')


begin = (7, 7)
finish = (-7, -7)
MAP = MAP_make(17, 17, 1/4, begin, finish)

try:
    print('SOLVING MAP WITH START AND END LOCATIONS MARKED')
    MAP_vis(17, 17, MAP, [begin, finish])
    result_raw, error = A_star(finish, begin, MAP)
    if error == TimeoutError:
        raise TimeoutError
    else:
        result_unpacked = pos_extraction(node_extraction(result_raw))
        print('FINAL PATH BETWEEN POINTS')
        MAP_vis(17, 17, MAP, result_unpacked)
except TimeoutError:
    print('Runtime Error, A* script has exceeded allocated runtime (10s)')
    result_unpacked = pos_extraction(node_extraction(result_raw))
    MAP_vis(17, 17, MAP, result_unpacked)
