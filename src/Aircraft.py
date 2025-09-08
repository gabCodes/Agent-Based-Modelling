import random

from single_agent_planner import simple_single_agent_astar, prioritized_astar
import math

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict, edges_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
        """
        
        #Fixed parameters
        self.speed = 1         #how much a/c moves per unit of t
        self.id = flight_id       #flight_id
        self.type = a_d           #arrival or departure (A/D)
        self.spawntime = spawn_time #spawntime
        self.start = start_node   #start_node_id
        self.goal = goal_node     #goal_node_id
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        self.edges_dict = edges_dict

        #Route related
        self.status = None
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]

        #State related
        self.heading = 0
        self.position = (0,0) #xy position on map

        #current location
        self.curr_loc = start_node
        self.last_loc = None

        self.endTime = None

        #self.potential_next_locs = []




    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading
      
    def move(self, dt, t):   
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = 
            - t = 
        """
        
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep

        if from_node != to_node:
        #Update position with rounded values
            x = xy_to[0]-xy_from[0]
            y = xy_to[1]-xy_from[1]
        #if x == 0 and y == 0:
        #    posx = round(self.position[0])
        #    posy = round(self.position[1])
        #    self.position = (posx, posy)
            x_normalized = x / math.sqrt(x**2+y**2)
            y_normalized = y / math.sqrt(x**2+y**2)
            posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
            posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
            self.position = (posx, posy)
            self.get_heading(xy_from, xy_to)

        #Check if goal is reached or if to_node is reached

        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if from_node != to_node: #only change last (and curr) location if it did not wait
                self.curr_loc = to_node
                self.last_loc = from_node

            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"
                self.endTime = t + dt

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]

                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                #if new_from_id != self.from_to[0]:
                    #print ("Im here!")
                    #self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node
                #else:
                #    self.from_to = [new_from_id, new_next_id] #update new from and to node
        self.first_step = False


    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        
        if self.status == "taxiing":
            start_node = self.start #node from which planning should be done
            goal_node = self.goal #node to which planning should be done
            
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
            else:
                raise Exception("No solution found for", self.id)
            
            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

    
    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t, constraints):
        constraintsOfThisAC = []
        success, path = prioritized_astar(self, nodes_dict, self.start, self.goal, heuristics, self.spawntime, constraints)
        if self.status == "taxiing":
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
            else:
                print('path, constraintsOfThisAC, success', path, constraintsOfThisAC, success)
                return path, constraintsOfThisAC, success

            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")


        for cell in path:
            if cell[0] == 1:
                constraintsOfThisAC.append({'loc': [2], 'timestep': cell[1]})
            if cell[0] == 2:
                constraintsOfThisAC.append({'loc': [1], 'timestep': cell[1]})
            if cell[0] == 37:
                constraintsOfThisAC.append({'loc': [38], 'timestep': cell[1]})
            if cell[0] == 38:
                constraintsOfThisAC.append({'loc': [37], 'timestep': cell[1]})

            constraintsOfThisAC.append({'loc': [cell[0]], 'timestep': cell[1]})
            if cell != path[0]:
                constraintsOfThisAC.append({'loc': [cell[0], lastCell[0]], 'timestep': cell[1]})
            lastCell = cell

        return path, constraintsOfThisAC, success
    
    def plan_cbs(self, path, t):
        self.path_to_goal = path[1:]
        next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
        self.from_to = [path[0][0], next_node_id]

        if path[0][1] != t:
            raise Exception("Something is wrong with the timing of the path planning")


    def plan_distr(self, t):
        """
        if all outgoing edges are info. stand still
        else take the edge with the lowest weight. If there are 2 edges with the same weight it is a 50/50 percent chance which one you take

        """
        edge_options = []
        for neighbor in self.nodes_dict[self.curr_loc]['neighbors']:
            edge_options.append({'neighbor': neighbor,'edge': (self.curr_loc, neighbor), 'weight': self.edges_dict[self.curr_loc, neighbor]['weight']})

        sortedEdges = sorted(edge_options,key=lambda d: d['weight'])  # sorts the list from shortest to longest path

        if sortedEdges[0]['weight'] == float('inf'):
            self.from_to = [self.curr_loc, self.curr_loc]
            self.path_to_goal = [[self.curr_loc, t + 0.5], [None]]
        elif len(sortedEdges)>1 and sortedEdges[0]['weight'] == sortedEdges[1]['weight']:
            if random.randint(0,100) <= 50:
                minWeight = sortedEdges[0]
                self.from_to = [self.curr_loc, minWeight['neighbor']]
                self.path_to_goal = [[minWeight['neighbor'], t + 0.5], [None]]
            else:
                minWeight = sortedEdges[1]
                self.from_to = [self.curr_loc, minWeight['neighbor']]
                self.path_to_goal = [[minWeight['neighbor'], t + 0.5], [None]]
        else:
            minWeight = sortedEdges[0]
            self.from_to = [self.curr_loc, minWeight['neighbor']]
            self.path_to_goal = [[minWeight['neighbor'], t+0.5],[None]]



