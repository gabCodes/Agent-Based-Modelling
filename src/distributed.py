import heapq
import itertools

from radar import Radar


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
        path.append((curr['loc'], curr['t']))
        curr = curr['parent']
    path.reverse()
    return path

def lengthPath_edge(aircraft, next_node, nodes_dict, heuristics, t):
    """
        the length of the shortest path (no constraints taken into account)
         to goal from a specific node, namely next_node is one of the
         neighbors from the current location node the ac is at
    """
    from_node_id = next_node
    goal_node_id = aircraft.goal

    count = 0
    open_list = []
    closed_list = dict()
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'last_loc': aircraft.curr_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 't': t}
    push_node(open_list, root, count)
    closed_list[root['loc']] = root
    while len(open_list) > 0:
        count += 1
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id:
            path = get_path(curr)
            return len(path), path

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                     'last_loc': curr['loc'],
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     't': curr['t'] + 0.5}
            if child['loc'] == curr['last_loc']:
                continue
            if (child['loc'], child['t']) in closed_list:
                existing_node = closed_list[(child['loc'],child['t'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['t'])] = child
                    push_node(open_list, child, count)
            else:
                closed_list[(child['loc'],child['t'])] = child
                push_node(open_list, child, count)
    print("No path found, " + str(len(closed_list)) + " nodes visited")
    return None, None

def direction_from_node_to_node(from_node, to_node, nodes_dict):
    X_from_node = nodes_dict[from_node]['x_pos']
    Y_from_node = nodes_dict[from_node]['y_pos']
    X_to_node = nodes_dict[to_node]['x_pos']
    Y_to_node = nodes_dict[to_node]['y_pos']

    if X_from_node + 0.5 == X_to_node and Y_from_node == Y_to_node:
        dir = 2#'right'
    elif X_from_node - 0.5 == X_to_node and Y_from_node == Y_to_node:
        dir = 1#'left'
    elif X_from_node == X_to_node and Y_from_node + 0.5 == Y_to_node:
        dir = 4#'up'
    elif X_from_node == X_to_node and Y_from_node -0.5 == Y_to_node:
        dir = 3#'down'
    else:
        dir = None
    return dir

def weight_priority_crowded_intersection(leftover_heads_from_intersection, nodes_dict):
    priority_list = []
    for edge in leftover_heads_from_intersection:
        dir = direction_from_node_to_node(*edge, nodes_dict)
        priority_list.append((dir,edge))

    priority_list.sort(reverse=True) #lowest priority will be first item
    edge_to_constrain = priority_list[0][1]
    return edge_to_constrain

def edge_case_edges_link(constrain_edge):
    linked_edge = None
    if constrain_edge == (33,87):
        linked_edge = (32,91)
    elif constrain_edge == (33,91):
        linked_edge = (28,87)
    elif constrain_edge == (15, 66):
        linked_edge = (14, 59)
    elif constrain_edge == (15, 59):
        linked_edge = (22, 66)
    elif constrain_edge == (11, 62):
        linked_edge = (12, 56)
    elif constrain_edge == (11, 56):
        linked_edge = (18, 62)
    elif constrain_edge == (29, 83):
        linked_edge = (30, 88)
    elif constrain_edge == (29, 88):
        linked_edge = (24, 83)
    return linked_edge

def weight_priority_crowded_intersection_edge_cases(leftover_heads_from_intersection, nodes_dict):
    special_edgesA = [(14, 59), (22, 66), (32, 91), (28, 87), (12, 56), (18, 62), (30, 88), (24, 83)]
    priority_list = []
    edges_to_constrain = []

    for special_edge in special_edgesA:
        if special_edge in leftover_heads_from_intersection:
            leftover_heads_from_intersection.remove(special_edge)


    for edge in leftover_heads_from_intersection:
        dir = direction_from_node_to_node(*edge, nodes_dict)
        priority_list.append((dir, edge))

    priority_list.sort(reverse=True)  # lowest priority will be first item
    constrain_edge = priority_list[0][1]
    edges_to_constrain.append(constrain_edge)

    linked_edge = edge_case_edges_link(constrain_edge)
    if linked_edge != None:
        edges_to_constrain.append(linked_edge)

    return edges_to_constrain

def weight_priority_between(acA, acB, between_pos):
    """
    Going left has priority over going right.
    Going down has priority over going up.
    """
    X_positionA = acA.position[0]
    Y_positionA = acA.position[1]
    X_positionB = acB.position[0]
    Y_positionB = acB.position[1]

    if acB.position == (X_positionA + 1, Y_positionA) and (X_positionA + 0.5, Y_positionA) == between_pos:
        return acA  # priority for B
    elif acA.position == (X_positionB + 1, Y_positionB) and (X_positionB + 0.5, Y_positionB) == between_pos:
        return acB  # priority for A

    # priority for plane coming from top vs bottom
    if acB.position == (X_positionA, Y_positionA + 1) and (X_positionA, Y_positionA + 0.5) == between_pos:
        return acA  # priority for B
    elif acA.position == (X_positionB, Y_positionB + 1) and (X_positionB, Y_positionB + 0.5) == between_pos:
        return acB  # priority for A
    else:
        return None

def weight_priority_intersection(acA, acB, intersection_pos, nr_acs):
    """
    Here the weight of plane pairs headed at the same intersection are decided.
    If the planes are standing with an angle of 90 degrees from each other the plane on the right is prioritized
    If there are only 2 planes at the intersection AND they are standing straight ahead of each other,
    then the plane coming from the right is prioritized over the one coming from the left
    or the plane coming from the top is prioritized over the one coming from the bottom
    """
    ############## right priority
    X_positionA = acA.position[0]
    Y_positionA = acA.position[1]
    X_positionB = acB.position[0]
    Y_positionB = acB.position[1]
    if acB.position == (X_positionA-0.5, Y_positionA+0.5) and (X_positionA-0.5, Y_positionA) == intersection_pos:
        return acA #print('priority for B')
    elif acA.position == (X_positionB - 0.5, Y_positionB + 0.5) and (X_positionB - 0.5, Y_positionB) == intersection_pos:
        return acB #print('priority for A')
    elif acB.position == (X_positionA+0.5,Y_positionA-0.5) and (X_positionA+0.5,Y_positionA) == intersection_pos:
        return acA #print('priority for B')
    elif acA.position == (X_positionB + 0.5, Y_positionB - 0.5) and (X_positionB + 0.5, Y_positionB) == intersection_pos:
        return acB #print('priority for A')
    elif acB.position == (X_positionA-0.5,Y_positionA-0.5) and (X_positionA,Y_positionA-0.5) == intersection_pos:
        return acA #print('priority for B')
    elif acA.position == (X_positionB - 0.5, Y_positionB - 0.5) and (X_positionB, Y_positionB - 0.5) == intersection_pos:
        return acB #print('priority for A')
    elif acB.position == (X_positionA+0.5,Y_positionA+0.5) and (X_positionA,Y_positionA+0.5) == intersection_pos:
        return acA #print('priority for B')
    elif acA.position == (X_positionB + 0.5, Y_positionB + 0.5) and (X_positionB, Y_positionB + 0.5) == intersection_pos:
        return acB #print('priority for A')

    # just two planes at intersection straight ahead of each other
    # priority for plane coming from the right vs left
    # priority for plane coming from top vs bottom
    if nr_acs == 2:
        if acB.position == (X_positionA + 1, Y_positionA) and (X_positionA + 0.5, Y_positionA) == intersection_pos:
           return acA  # print('priority for B') AA
        elif acA.position == (X_positionB + 1, Y_positionB) and (X_positionB + 0.5, Y_positionB) == intersection_pos:
           return acB #prio A
        elif acB.position == (X_positionA, Y_positionA + 1) and (X_positionA, Y_positionA + 0.5) == intersection_pos:
            return acA  # print('priority for B')
        elif acA.position == (X_positionB, Y_positionB + 1) and (X_positionB, Y_positionB + 0.5) == intersection_pos:
            return acB  # print('priority for A')

    else:
        return None

def distance_between_2_acs(acA, acB):
    horizontal_dist = abs(acA.position[0]-acB.position[0])
    vertical_dist = abs(acA.position[1]-acB.position[1])
    total_distance = horizontal_dist + vertical_dist
    return total_distance

def edge_case_weights_at_betweens(acA, acB):
    """
    Returns the edge that should be constrained
    """
    if (acA.curr_loc == 12 and acA.last_loc !=56 and acB.curr_loc == 18 and acB.last_loc != 62) \
        or (acB.curr_loc == 12 and acB.last_loc !=56 and acA.curr_loc == 18 and acA.last_loc != 62):
        return(12,56)
    elif (acA.curr_loc == 30 and acA.last_loc !=88 and acB.curr_loc == 24 and acB.last_loc != 83) \
        or (acB.curr_loc == 30 and acB.last_loc !=88 and acA.curr_loc == 24 and acA.last_loc != 83):
        return (30,88)
    elif (acA.curr_loc == 32 and acA.last_loc != 91 and acB.curr_loc == 28 and acB.last_loc != 87) \
            or (acB.curr_loc == 32 and acB.last_loc != 91 and acA.curr_loc == 28 and acA.last_loc != 87):
        return (32, 91)
    elif (acA.curr_loc == 14 and acA.last_loc != 59 and acB.curr_loc == 22 and acB.last_loc != 66) \
            or (acB.curr_loc == 14 and acB.last_loc != 59 and acA.curr_loc == 22 and acA.last_loc != 66):
        return (14, 59)
    else:
        return None

def in_between_node(acA, acB, nodes_dict):
    nodeA = nodes_dict[acA.curr_loc]
    nodeB = nodes_dict[acB.curr_loc]
    between_node = list(set(nodeA['neighbors']) & set(nodeB['neighbors']))
    if len(between_node) > 1:
        raise Exception("in_between_node > 1")
    #if the position in between is not a node
    if len(between_node) == 0:
        return None
    # one of the pair just came from the intersection
    if between_node[0] == acA.last_loc or between_node[0] == acB.last_loc:
        return None

    return between_node[0]

def run_Distributed(aircraft_list, nodes_dict, edges_dict, heuristics, t):
    taxiing_list = []
    for ac in aircraft_list:
        if ac.status != 'arrived':
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.curr_loc]["xy_pos"]
            taxiing_list.append(ac)
    distr = DISTR(taxiing_list, nodes_dict, edges_dict, heuristics, t)
    distr.weights_shortest_paths()
    distr.weights_avoid_huge_detour()
    distr.weight_turn_back()
    distr.weights_seperation()
    distr.weights_at_intersection()
    distr.weight_crowded_intersection()
    distr.weights_at_betweens()
    distr.weights_runways()
    distr.weights_terminal_edge_cases()
    distr.weights_near_departure()
    for ac in taxiing_list:
        ac.plan_distr(t)



class DISTR(object):
    def __init__(self, ac_list, nodes_dict, edges_dict, heuristics, t):
        self.ac_list = ac_list
        self.num_of_agents = len(ac_list)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.nodes_dict = nodes_dict
        self.edges_dict = edges_dict
        self.t = t
        self.open_list = []
        self.radar = Radar(ac_list, nodes_dict, edges_dict, t)
        self.heuristics = heuristics

    def weights_shortest_paths(self):
        """
        This method calculates the weights of all outgoing edges from the curr location of every aircraft
        For every next node the shortest path to the goal will be calculated
        The edge to the node with shortest route to the goal will get a weight of 0
        The other edges will get a weight of their path length - the length of the shortest route

        """
        for ac in self.ac_list:
            shortest_path_length = float('inf')
            neighbor_length_list = []
            for neighbor_id in self.nodes_dict[ac.curr_loc]["neighbors"]:
                length, neighborPath = lengthPath_edge(ac, neighbor_id, self.nodes_dict, self.heuristics, self.t + 0.5)
                if length == None:
                    self.edges_dict[ac.curr_loc, neighbor_id]['weight'] = float('inf')
                    continue
                if length < shortest_path_length:
                    shortest_path_length = length

                neighbors_with_length = {'neighbor': neighbor_id, 'length': length, 'path': neighborPath}
                neighbor_length_list.append(neighbors_with_length)

            for path in neighbor_length_list:
                # shortest route edge weight == 0, 2nd is length - shortest path length
                self.edges_dict[ac.curr_loc, path['neighbor']]['weight'] = path['length'] - shortest_path_length

    def weights_avoid_huge_detour(self):
        """
        General rules for arriving aircrafts. They have a demarcated grid. See word doc for overview.
        """
        for ac in self.ac_list:
            if ac.type == 'A':
                if ac.curr_loc == 11:
                    self.edges_dict[11, 55]['weight'] = float('inf')
                    self.edges_dict[11, 101]['weight'] = float('inf')
                elif ac.curr_loc == 18:
                    self.edges_dict[18, 68]['weight'] = float('inf')
                elif ac.curr_loc == 24:
                    self.edges_dict[24, 105]['weight'] = float('inf')
                elif ac.curr_loc == 15:
                    self.edges_dict[15, 60]['weight'] = float('inf')
                    self.edges_dict[15, 53]['weight'] = float('inf')
                elif ac.curr_loc == 22:
                    self.edges_dict[22, 73]['weight'] = float('inf')
                elif ac.curr_loc == 28:
                    self.edges_dict[28, 106]['weight'] = float('inf')

    def weight_turn_back(self):
        """
        Aircrafts are not allowed to turn 180 degrees.
        """
        for ac in self.ac_list:
            if ac.last_loc != None:
                self.edges_dict[(ac.curr_loc,ac.last_loc)]['weight'] = float('inf')

    def weights_seperation(self):
        """
        Aircrafts are not allowed to go to a node (t+0.5) where another aircraft is (t)
        """
        for ac in self.ac_list:
            for neighbor_id in self.nodes_dict[ac.curr_loc]["neighbors"]:
                self.edges_dict[(neighbor_id,ac.curr_loc)]['weight'] = float('inf')

    def weights_at_betweens(self):
        """
        For all plane pairs that are located at intersection or gate nodes
        First check all special edge cases.
            (12 via 11 to 18) and vice versa, (24 via 29 to 30) and vice versa,
            (14 via 15 to 22) and vice versa, (32 via 33 to 28) and vice versa

        Decide which ac will be allowed to go along that path. This is decided in edge_case_weights_at_betweens.

        Then check al plane pairs that have a distance of 1 from one another and have an overlapping node as neighbor
        that overlapping node is the between node they are standing at
        decide which plane can taxi along that path with weight_priority_between(*ac_pair, between_node_position)
        Give the other edges a weight of infinity

        """
        intersect_planes = []
        for ac in self.ac_list:
            if self.nodes_dict[ac.curr_loc]['type'] == 'intersection' or self.nodes_dict[ac.curr_loc]['type'] == 'gate':
                intersect_planes.append(ac)

        all_plane_pairs = list(itertools.combinations(intersect_planes, 2))
        for ac_pair in all_plane_pairs:
            edge_case_between = edge_case_weights_at_betweens(*ac_pair)
            if edge_case_between != None:
                self.edges_dict[edge_case_between]['weight'] = float('inf')

            elif distance_between_2_acs(*ac_pair) == 1:
                between_node_id = in_between_node(*ac_pair, self.nodes_dict)
                if between_node_id == None:
                    continue

                between_node_position = self.nodes_dict[between_node_id]['xy_pos']
                ac_wait = weight_priority_between(*ac_pair, between_node_position)
                if ac_wait != None:
                    self.edges_dict[ac_wait.curr_loc, between_node_id]['weight'] = float('inf')

    def weights_at_intersection(self):
        """
        for all planes that are located at between nodes
        check al plane pairs that have a distance of 1 from one another and have an overlapping node as neighbor
        that overlapping node is the intersection node they are standing at
        check the amount of acs that are headed at that specific intersection with self.radar.nr_acs_heading_for_intersection(intersection_node_id)
        decide which plane can cross the intersection first with weight_priority_intersection(*ac_pair, between_node_position, nr_acs)
        Give the other edges a weight of infinity

        """
        between_planes = []
        for ac in self.ac_list:
            if self.nodes_dict[ac.curr_loc]['type'] == 'between':
                between_planes.append(ac)

        all_plane_pairs = list(itertools.combinations(between_planes, 2))
        for ac_pair in all_plane_pairs:
            if distance_between_2_acs(*ac_pair) == 1:
                intersection_node_id = in_between_node(*ac_pair, self.nodes_dict)
                if intersection_node_id == None: #one of the pair just came from the intersection
                    continue

                nr_acs = self.radar.nr_acs_heading_for_intersection(intersection_node_id)

                between_node_position = self.nodes_dict[intersection_node_id]['xy_pos']
                ac_wait = weight_priority_intersection(*ac_pair, between_node_position, nr_acs)
                if ac_wait != None:
                    self.edges_dict[ac_wait.curr_loc, intersection_node_id]['weight'] = float('inf')

    def weights_runways(self):
        """
        only one plane at the runway at the same time
        arrive runway is no problem due to way of spawning
        departure runway entrance/node 1 has priority over entrance/node 2
        """
        for ac in self.ac_list:
            if ac.curr_loc == 95:
                self.edges_dict[96,2]['weight'] = float('inf')

    def weights_near_departure(self):
        """
        Special weights near the departure.
        Here different rules apply than in the rest of the grid.
        that is also why two edges are overwritten with a weight of 0
        """
        for ac in self.ac_list:
            if ac.curr_loc == 45:
                self.edges_dict[43, 4]['weight'] = float('inf')

            elif ac.curr_loc == 4:
                self.edges_dict[45, 5]['weight'] = float('inf')
                if ac.last_loc == 44:
                    self.edges_dict[4, 43]['weight'] = float('inf')
                if ac.goal == 1:
                    self.edges_dict[4, 43]['weight'] = float('inf')
                    self.edges_dict[4, 44]['weight'] = float('inf')


            elif ac.curr_loc == 5:
                self.edges_dict[43, 4]['weight'] = float('inf')
                if ac.last_loc == 4:
                    self.edges_dict[5, 45]['weight'] = float('inf')
                if ac.goal == 2:
                    self.edges_dict[5, 44]['weight'] = float('inf')
                    self.edges_dict[5, 45]['weight'] = float('inf')


            elif ac.curr_loc == 44 and ac.last_loc == 5:
                self.edges_dict[43, 4]['weight'] = float('inf')
                self.edges_dict[44, 4]['weight'] = 0
            elif ac.curr_loc == 44 and ac.last_loc == 4:
                self.edges_dict[45, 5]['weight'] = float('inf')
                self.edges_dict[44, 5]['weight'] = 0

    def weight_crowded_intersection(self):
        """
        The general idea is that if an intersection has 4 outgoing edges, not all edges can be occupied because then all planes will be stuck

        For all intersection nodes we check how many of the nodes one edge away are occupied with self.radar.acs_heading_for_intersection(intersection_node)
        An edge is occupied if there is a plane on a between node headed to that intersection, or if a plane is on the intersection and then the node/edge it just came from
        This method returns all nodes 1 edge away from the intersection node that are NOT occupied

        For those nodes we check whether there is an ac one node further that might want to go to the intersection. But since it is one node further/on another intersection node,
        we can still let it go somewhere else.

        But because we made the grid 'smaller' for the arriving planes and the terminals are not open for every plane there are some edge cases

        For example (12 via 11 to 18) and vice versa, (24 via 29 to 30) and vice versa, (14 via 15 to 22) and vice versa, (32 via 33 to 28) and vice versa
        should be considered as one sort of. When looking at intersection 28. A plane standing at 32 is the same as one standing at 22
        and a plane standing in 91 is the same as a plane is 78. (both heading towards 28)

        Because we dont allow arriving plane to go some places like (22,73) we fix this by acting like that edge is always occupied. We do the same for the terminals.

        """
        special_intersections = [14,22,28,32,12,18,24,30]
        special_edges = [(14, 59), (15, 66), (15, 59), (22, 66), (32, 91), (33, 87), (28, 87), (33, 91), (12, 56),
                         (11, 62), (18, 62), (11, 56), (29, 83), (30, 88), (29, 88), (24, 83)]

        for intersection_node in self.nodes_dict:
            if self.nodes_dict[intersection_node]['type'] == 'intersection':
                leftover_routes_from_intersection = self.radar.acs_heading_for_intersection(intersection_node)
                nr_leftover_edges_intersection = len(leftover_routes_from_intersection)
                if nr_leftover_edges_intersection == 0:
                    raise Exception('Full intersection problem! Should never happen')


                if nr_leftover_edges_intersection ==  1:
                    for neighbor in self.nodes_dict[leftover_routes_from_intersection[0]]['neighbors']:
                        if neighbor != intersection_node:
                            self.edges_dict[neighbor,leftover_routes_from_intersection[0]]['weight'] = float('inf')
                            if intersection_node in special_intersections and (neighbor,leftover_routes_from_intersection[0]) in special_edges:
                                linked_edge = edge_case_edges_link((neighbor,leftover_routes_from_intersection[0]))
                                self.edges_dict[linked_edge]['weight'] = float('inf')


                elif nr_leftover_edges_intersection >1:
                    other_heads_from_intersection = self.radar.head_of_leftover_edges(leftover_routes_from_intersection, intersection_node)
                    leftover_edges = self.radar.head_of_leftover_edges_occupied(other_heads_from_intersection)

                    if intersection_node in special_intersections:
                        if len(leftover_edges) == 1 and leftover_edges[0] not in special_edges:
                            continue
                        elif len(leftover_edges) <= 1:
                            edges = weight_priority_crowded_intersection_edge_cases(other_heads_from_intersection, self.nodes_dict)
                            for edge in edges:
                                self.edges_dict[edge]['weight'] = float('inf')

                    else:
                        if len(leftover_edges) == 0:
                            edge = weight_priority_crowded_intersection(other_heads_from_intersection, self.nodes_dict)
                            self.edges_dict[edge]['weight'] = float('inf')

    def weights_terminal_edge_cases(self):
        """
        Weights to prevent edge case conflicts at the end of the grid near the terminals
        eg If there is a plane in 28 that could go to 87, no ac is allowed to go from 100 to 33
        """
        for ac in self.ac_list:
            if (ac.curr_loc == 28 and ac.last_loc != 87) or (ac.curr_loc == 32 and ac.last_loc != 91):
                self.edges_dict[100,33]['weight'] = float('inf')
            elif (ac.curr_loc == 30 and ac.last_loc != 88) or (ac.curr_loc == 24 and ac.last_loc != 83):
                self.edges_dict[99,29]['weight'] = float('inf')





