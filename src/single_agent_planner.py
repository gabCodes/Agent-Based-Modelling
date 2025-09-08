"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length

def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start):
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
    
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start

    count = 0
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root, count)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        count += 1
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child, count)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child, count)
    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def prioritized_astar(agent, nodes_dict, from_node, goal_node, heuristics, time_start, constraints):
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
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    constraint_table = build_constraint_table(constraints, agent)
    count = 0
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root, count)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        count += 1
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)

        if not is_constrained(curr['loc'], curr['loc'], curr['timestep'] + 0.5, constraint_table):
            w_child = {'loc': curr['loc'],
                       'g_val': curr['g_val'] + 0.5,
                       'h_val': curr['h_val'],
                       'parent': curr,
                       'timestep': curr['timestep'] + 0.5}
            closed_list[(w_child['loc'], w_child['timestep'])] = w_child
            push_node(open_list, w_child, count)
        #(15, 7, 8, xy, iteration, node 18), (15, 7, 8, xy, iteration, node 29)
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}
            if curr['parent'] is not None and child['loc'] == curr['parent']['loc']: #cant go back to last location (180 degree turn)
                continue
            if curr['parent'] is not None and curr['parent']['parent'] is not None and child['loc'] == \
                    curr['parent']['parent']['loc']: #cant go back to last location if just stood still for one timestep (180 degree turn)
                continue
            if is_constrained(curr['loc'], child['loc'], curr['timestep'] + 0.5, constraint_table):
                continue
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child, count)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child, count)
    print("No path found, " + str(len(closed_list)) + " nodes visited")
    return False, []  # Failed to find solutions

def cbs_astar(aircraft, nodes_dict, heuristics, constraints):
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

    from_node_id = aircraft.start
    goal_node_id = aircraft.goal
    time_start = aircraft.spawntime

    constraint_table = cbs_build_constraint_table(constraints, aircraft)
    count = 0
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root, count)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        count += 1
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            print ("Constraint table looks like: " + str(constraint_table) + " for " + str(aircraft.id))
            print ("Path looks like " + str(get_path(curr)) + " for " + str(aircraft.id))
            return aircraft, get_path(curr)

        if not is_constrained_cbs(curr['loc'], curr['loc'], curr['timestep'] + 0.5, constraint_table): #stand still / do not move option
            w_child = {'loc': curr['loc'],
                       'g_val': curr['g_val'] + 0.5,
                       'h_val': curr['h_val'],
                       'parent': curr,
                       'timestep': curr['timestep'] + 0.5}
            closed_list[(w_child['loc'], w_child['timestep'])] = w_child
            push_node(open_list, w_child, count)
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}
            if curr['parent'] is not None and child['loc'] == curr['parent']['loc']: #cant go back to last location (180 degree turn)
                continue
            if curr['parent'] is not None and curr['parent']['parent'] is not None and child['loc'] == \
                    curr['parent']['parent']['loc']: #cant go back to last location if just stood still for one timestep (180 degree turn)
                continue
            if is_constrained_cbs(curr['loc'], child['loc'], curr['timestep'] + 0.5, constraint_table):
                continue
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child, count)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child, count)
    print("No path found, " + str(len(closed_list)) + " nodes visited")
    return aircraft, []  # Failed to find solutions

def push_node(open_list, node, count):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], count, node))

def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    return path

def build_constraint_table(constraints, agent):
    table = []
    for i in range(len(constraints)):
        table.append([constraints[i]['timestep'], constraints[i]['loc']])
    return table

#Constraints input look like [{type,agent, location, time}, {type,agent, location, time}]
#Output looks like table [[time, ]]
def cbs_build_constraint_table(constraints, agent):
    table = []
    for constraint in constraints:
        if agent == constraint['agent']:
            table.append({'type': constraint['type'], 'loc': constraint['loc'], 'time': constraint['timestep']})
    print ("cbs table of constraint for " + str(agent.id) + " looks like " + str(table))
    return table

def is_constrained_cbs(curr_loc, next_loc, next_time, constraint_table):
    for constraint in constraint_table:
        if constraint['type'] == 'vertex':
            if next_time == constraint['time'] and next_loc == constraint['loc']:
                return True
        if constraint['type'] == 'edge':
            if next_time == constraint['time'] and curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                return True
        return False

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    #constraint_table looks like: [[1,[1,4]],[1,[[1,4],[1,5]]] where first constraint is vertex, 2nd is edge
    for i in range(len(constraint_table)):
        if len(constraint_table[i][1]) == 1:
            if constraint_table[i][0] == next_time and constraint_table[i][1][0] == next_loc:
                return True
        elif len(constraint_table[i][1]) > 1:
            if (constraint_table[i][0] == next_time) and constraint_table[i][1][0] == curr_loc and constraint_table[i][1][1] == next_loc:
                return True

    return False

