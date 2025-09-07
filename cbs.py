"""
Implement CBS here
"""

import heapq


# Format for collision: Input paths: [(node, time), (node, time)]
# Vertex collision --> (location,time) Edge collision --> [(node, node, time)]
def detect_collision(path1, path2):
    collision = 0
    if len(path1) <= len(path2):
        for i in range(len(path1)):
            # If The locations match at same time there's a collision
            if path1[i][0] == path2[i][0] and path1[i][1] == path2[i][1]:
                collision = (path1[i][0], path1[i][1])
                return collision
            # If the paths cross by each other
            if i < len(path1) - 1:
                if path1[i][0] == path2[i + 1][0] and path1[i][1] + 0.5 == path2[i + 1][1] and path1[i + 1][0] == \
                        path2[i][0]:
                    collision = (path1[i][0], path2[i][0], path1[i + 1][1])
                    return collision
    else:
        for i in range(len(path2)):
            # If The locations match at same time there's a collision
            if path2[i][0] == path1[i][0] and path2[i][1] == path1[i][1]:
                collision = (path1[i][0], path1[i][1])
                return collision
            # If the paths cross by each other
            if i < len(path2) - 1:
                if path2[i][0] == path1[i + 1][0] and path2[i][1] + 0.5 == path1[i + 1][1] and path2[i + 1][0] == \
                        path1[i][0]:
                    collision = (path1[i][0], path2[i][0], path1[i + 1][1])
                    return collision

    return None


# Input: input paths look like [(agent, path), (agent, path)]
# Outputs: [{type, ac1, ac2, node, time}, {type, ac2, ac3, [node1, node2], time}]
# type: #Vertex = 0, Edge = 1
def detect_collisions(paths):
    collisions = []
    collisions_dict = []
    for i in range(len(paths)):
        for j in range(len(paths)):
            if j > i:
                collision = detect_collision(paths[i][1], paths[j][1])
                if collision != None:
                    if collision not in collisions and len(collision) == 2:
                        collisions.append(collision)
                        collisions_dict.append(
                            {'type': 0, 'a1': paths[i][0], 'a2': paths[j][0], 'loc': collision[0],
                             'timestep': collision[1]})
                    if collision not in collisions and len(collision) == 3:
                        collisions.append(collision)
                        collisions_dict.append({'type': 1, 'a1': paths[i][0], 'a2': paths[j][0],
                                                'loc': [collision[0], collision[1]], 'timestep': collision[2]})

    return collisions_dict


def standard_splitting(collision):
    """
    Generates constraints depending on collisions
    """
    constraint_list = []
    is_Vertex = collision['type'] == 0

    if is_Vertex == True:
        constraint_list.append(
            {'type': 0, 'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraint_list.append(
            {'type': 0, 'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    if is_Vertex == False:
        constraint_list.append(
            {'type': 1, 'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        collision_loc = collision['loc']
        copy_collision_loc = collision_loc[:]
        copy_collision_loc.reverse()
        constraint_list.append(
            {'type': 1, 'agent': collision['a2'], 'loc': copy_collision_loc, 'timestep': collision['timestep']})

    return constraint_list

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path[1]) - 1
    return rst

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def cbs_astar(aircraft, nodes_dict, heuristics, constraints, t):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time.
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    from_node_id = aircraft.curr_loc
    goal_node_id = aircraft.goal
    time_start = t

    constraint_table = cbs_build_constraint_table(constraints, aircraft)
    count = 0
    open_list = []
    closed_list = dict()
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'last_loc': aircraft.last_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    astar_push_node(open_list, root, count)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        count += 1
        curr = astar_pop_node(open_list)
        if curr['loc'] == goal_node_id:
            return [aircraft, get_path(curr)]

        if not cbs_is_constrained(curr['loc'], curr['loc'], curr['timestep'] + 0.5,
                                  constraint_table):  # stand still / do not move option
            w_child = {'loc': curr['loc'],
                       'last_loc': curr['last_loc'],
                       'g_val': curr['g_val'] + 0.5,
                       'h_val': curr['h_val'],
                       'parent': curr,
                       'timestep': curr['timestep'] + 0.5}
            closed_list[(w_child['loc'], w_child['timestep'])] = w_child
            astar_push_node(open_list, w_child, count)
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                     'last_loc': curr['loc'],
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}
            if child['loc'] == curr['last_loc']:
                continue
            if cbs_is_constrained(curr['loc'], child['loc'], curr['timestep'] + 0.5, constraint_table):
                continue
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    astar_push_node(open_list, child, count)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                astar_push_node(open_list, child, count)
    return [aircraft, []]  # Failed to find solutions


def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    return path


def cbs_build_constraint_table(constraints, aircraft):
    table = []
    for constraint in constraints:
        if constraint['agent'] == aircraft:
            table.append(constraint)
        elif constraint['type'] == 2:
            table.append(constraint)
    return table


def cbs_is_constrained(curr_loc, next_loc, next_time, constraint_table):
    for constraint in constraint_table:
        if constraint['type'] == 0 and constraint['loc'] == next_loc and constraint['timestep'] == next_time:
            return True
        if constraint['type'] == 1 and constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc and \
                constraint['timestep'] == next_time:
            return True
        if constraint['type'] == 2 and constraint['loc'] == next_loc and constraint['timestep'] == next_time:
            return True
    return False


def astar_push_node(open_list, node, count):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], count, node))


def astar_pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr




def run_CBS(aircraft_list, nodes_dict, edges_dict, heuristics, t):
    taxiing_list = []
    for ac in aircraft_list:
        if ac.status != 'arrived':
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.curr_loc]["xy_pos"]
            taxiing_list.append(ac)
    cbs = CBS(taxiing_list, nodes_dict, edges_dict, heuristics, t)
    taxi_ac_n_paths = cbs.find_solution()
    for ac_n_path in taxi_ac_n_paths:
        ac_n_path[0].plan_cbs(ac_n_path[1],t)



class CBS(object):
    def __init__(self, ac_list, nodes_dict, edges_dict, heuristics, t):
        self.ac_list = ac_list
        self.num_of_agents = len(ac_list)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.nodes_dict = nodes_dict
        self.edges_dict = edges_dict
        self.t = t
        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = heuristics



    def push_node(self,node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1


    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def addOnePerLaneConstraints(self, ac_n_path, constraints):
        curr_ac = ac_n_path[0]
        last_of_path = ac_n_path[1][-1]
        last_node = last_of_path[0]
        last_time = last_of_path[1]
        if last_node == 1:
            for ac in self.ac_list:
                if ac != curr_ac:
                    constraints.append({'type': 2, 'agent': ac, 'loc': [2], 'timestep': last_time})
        elif last_node == 2:
            for ac in self.ac_list:
                if ac != curr_ac:
                    constraints.append({'type': 2, 'agent': ac, 'loc': [1], 'timestep': last_time})

    def checkOnePerLaneConstraints(self, curr):
        ac_n_paths = curr['paths']
        for ac_n_path in ac_n_paths:
            ac = ac_n_path[0]
            last_in_path = ac_n_path[1][-1]
            constraint_table = cbs_build_constraint_table(curr['constraints'], ac)
            if cbs_is_constrained(None, last_in_path[0], last_in_path[1], constraint_table):
                return False
        return True



    def find_solution(self, disjoint=True):
            """ Finds paths for all agents from their start locations to their goal locations

            disjoint    - use disjoint splitting or not
            """


            root = {'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}
            for aircraft in self.ac_list:  # Find initial path for each agent IMPLEMENT CBS_ASTAR
                ac_n_path = cbs_astar(aircraft, self.nodes_dict, self.heuristics, root['constraints'], self.t)
                self.addOnePerLaneConstraints(ac_n_path, root['constraints'])

                if len(ac_n_path[1]) <1:
                    raise Exception('No solutions')
                root['paths'].append(ac_n_path)

            root['cost'] = get_sum_of_cost(root['paths'])
            root['collisions'] = detect_collisions(root['paths'])
            self.push_node(root)

            while len(self.open_list) > 0:
                curr = self.pop_node()

                if len(curr['collisions']) == 0:
                    if self.checkOnePerLaneConstraints(curr):
                        return curr['paths']

                if len(curr['collisions']) >0:
                    cbs_collision = curr['collisions'][0]
                    cbs_constraints = standard_splitting(cbs_collision)
                else:
                    cbs_constraints = curr['constraints']


                for constraint in cbs_constraints:
                    Q = {'cost': 0,
                            'constraints': [],
                            'paths': [],
                            'collisions': []}
                    Q['constraints'] = list (curr['constraints'] )
                    Q['constraints'].append(constraint)
                    Q['paths'] = list ( curr['paths'] )
                    ac_i = constraint['agent']

                    ac_n_path = cbs_astar(ac_i, self.nodes_dict, self.heuristics, Q['constraints'], self.t)

                    if len(ac_n_path[1]) > 0: # if there is a path

                        self.addOnePerLaneConstraints(ac_n_path, Q['constraints'])

                        for i in range(len(Q['paths'])):
                            if Q['paths'][i][0] == ac_i:
                                Q['paths'][i] = ac_n_path
                        Q['collisions'] = detect_collisions(Q['paths'])
                        Q['cost'] = get_sum_of_cost(Q['paths'])
                        self.push_node(Q)


            self.print_results(root)
            return root['paths']

