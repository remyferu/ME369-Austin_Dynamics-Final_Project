#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  3 22:40:31 2022

@author: remyferu
"""


import rospy
import random
import time
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel


# =============================================================================
# A* ALGORITHIM SETUP USED IN TURTLEBOT NAVIGATION
# =============================================================================

# creates a node class for the A* algorithim 
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


# function that finds adjacent nodes for A* algorithim
def find_adjacent(node, check_list):
    x = node.pose[0]
    y = node.pose[1]
    return [Node((x+i, y+j)) for (i, j) in [(1, 0), (-1, 0), (0, 1), (0, -1)]if (x+i, y+j) in check_list]


# A* search algorithim
def A_star(destination, start_location, pose_list):
    time_i = time.time()

    # creates the open list containing only the start location node
    open_list = [Node(start_location)]

    # manually sets the fitness
    open_list[0].h_eval(start_location)
    open_list[0].g = 0
    open_list[0].f = open_list[0].h

    destination_reached = False

    while not destination_reached:
        # sorts open_list and grabs the node with the lowest f value (cheapest node to move to)
        node = sorted(open_list, key=lambda x: x.f)[0]

        # prevents the script from running for too long in case of unsolvable path
        if time.time() - time_i > 5:
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


# un-nests the A* solution node using recursion
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
def MAP_make(Column, Row, block_locations):
    Map = []
    # iterates over every column and row by a given step
    for i in range(0, Column):
        for j in range(0, Row):
            # converts coordinate systems from a corner (0,0) to a central (0,0)
            x = i - (Column - 1) / 2
            y = j - (Row - 1) / 2

            Map.append((x, y))

    # einverts the block_position list, finds all movable locations
    for i in Map:
        if i in block_locations:
            Map.pop(Map.index(i))
            
    return Map


# prints the map to the terminal
def MAP_vis(Column, Row, Map, Results=[]):
    print('')
    MAP_vis = ''
    # prints out a MAP visualization
    for i in range(0, Row):
        for j in range(0, Column):
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



# =============================================================================
# ROS NODE, USED TO MOVE TURTLEBOT AND BOX MODELS THROUGHOUT THE MAP
# =============================================================================
def model_mover():
    # initilizes a node in ROS 
    rospy.init_node('model_mover',anonymous=True) 
    
    # sets the rate that the node will run at (in Hz)
    rate = rospy.Rate(0.2)
    
    # node shutdown controller 
    destination_reached = False

    # creates a handle for the get_world_properties service
    world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties) 
    
    # creates a handle for the get_model_state service
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    # creates a handle for the set_model_state service 
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


    n = 1
    while not rospy.is_shutdown():
        
        # break condition for when destination is met 
        if destination_reached:
            rospy.signal_shutdown('The Turtlebot Has Reached its Destination')
        
        # # # PULLS GAZEBO MODEL INFORMAITON # # # 
        
        # prints the current iteration
        print('\n', 'iteration: ',n)
        
        # hangs the script until the service can be called 
        rospy.wait_for_service('/gazebo/get_world_properties') 
    
        # calls the get_world_properties and stores all model names in list model_names
        model_names = world_properties().model_names 

        # strips all non box models from the model_name list 
        for model in model_names:
            if 'box' not in model:
                model_names.pop(model_names.index(model))
        
        # pulls the turtlebot position during the first iteration 
        if n == 1: 
            # hangs the script until the service can be called 
            rospy.wait_for_service('/gazebo/get_model_state')
            
            # calls the get_model_state service and stores the turtlebot position
            turtle_bot_state = get_model_state('turtlebot3_waffle_pi', 'base_footprint')
            
            # pulls the x and y coordinates from the turtlebot model state
            turtle_bot_pos = (turtle_bot_state.pose.position.x,turtle_bot_state.pose.position.y)

        # logs the turtlebot position
        # rospy.loginfo(turtle_bot_pos)
        
        
        # =====================================================================
        #       BOX MOVEMENT
        # =====================================================================
        
        col = 17  # number of columns
        row = 17  # number of rows
        pose_list = []  # list of valid positions in map
        
        # iterates over every column and row to return a list of valid positions
        for i in range(0, col):
            for j in range(0, row):
                # converts coordinate systems from a corner (0,0) to a central (0,0)
                x = i - (col-1)/2
                y = j - (row-1)/2
                pose_list.append((x, y))
        
        # removes current turtlebot and adjacent turtlebot location from position list
        pose_list.pop(pose_list.index(turtle_bot_pos))
        # for i in range(-1, 2, 2):
        #     pose_list.pop(pose_list.index((turtle_bot_pos[0]+i, turtle_bot_pos[1])))
        #     pose_list.pop(pose_list.index((turtle_bot_pos[0], turtle_bot_pos[1]+i)))
        
        
        # generates new positions for each block model
        new_pose_list = []  # list of new block positions
        for i in range(0, len(model_names)):
            # appends a random position from pose_list to new_pose_list
            new_pose_list.append(pose_list[random.randint(0, len(pose_list)-1)])
        
            # pops the random position from pose_list to avoid duplicates
            pose_list.pop(pose_list.index(new_pose_list[i]))
        
        
        # sets the new state for every block model
        i = 0
        for name in model_names:

            # creates an empty ModelState object
            model_state_msg = ModelState()
            
            # populates the model_state message with a name from model_names
            model_state_msg.model_name = name
            
            # populates the model_state message with a new position
            model_state_msg.pose.position.x = new_pose_list[i][0]
            model_state_msg.pose.position.y = new_pose_list[i][1]
                
            # hangs the script until the service can be called
            rospy.wait_for_service('/gazebo/set_model_state')
            
            rospy.logdebug('moving:', name, 'to:',(model_state_msg.pose.position.x,  model_state_msg.pose.position.y))
            
            # updates the gazebo environment with the changes applied to model_state_msg
            set_model_state(model_state_msg)
            
            i += 1
                
        
        # =====================================================================
        #       TURTLEBOT NAVIGATION/MOVEMENT
        # =====================================================================
        
        # the A* algo begins at current turtlebot position
        begin = turtle_bot_pos
        
        if n == 1:
            # user inputed finish position
            finish = tuple([int(i) for i in input('input a coordinate as a tuple (x,y): ').strip("()").split(",")])
        
        # builds a list of valid locations the turtlebot can move to
        MAP = MAP_make(17, 17, new_pose_list)
       
        
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
            print('Runtime Error, A* script has exceeded allocated runtime (5s)')
            result_unpacked = pos_extraction(node_extraction(result_raw))
            MAP_vis(17, 17, MAP, result_unpacked)
        
        # TURTLEBOT MOVING LOOP
        for i in range(1, 4):
            
            # creates an empty ModelState object
            model_state_msg = ModelState()
                
            # populates the model_state message with a name from model_names
            model_state_msg.model_name = 'turtlebot3_waffle_pi'
            
            # catches the index error that occurs when the turtlebot reaches its destination
            try:     
                turtle_bot_pos = (result_unpacked[i][0], result_unpacked[i][1])
            except IndexError:
                destination_reached = True
                print('The Turtlebot Has Reached its Destination')
                break
            
            # populates the model_state message with a new position
            model_state_msg.pose.position.x = turtle_bot_pos[0]
            model_state_msg.pose.position.y = turtle_bot_pos[1]
                
            # hangs the script until the service can be called
            rospy.wait_for_service('/gazebo/set_model_state')
            
            rospy.logdebug('moving:', name, 'to:',(model_state_msg.pose.position.x,  model_state_msg.pose.position.y))
            
            # updates the gazebo environment with the changes applied to model_state_msg
            set_model_state(model_state_msg)
        
            time.sleep(1)
            
        n += 1
        rate.sleep()
        
if __name__ == "__main__": 
    try:     
        model_mover()
    except rospy.ROSInterruptException:
        pass