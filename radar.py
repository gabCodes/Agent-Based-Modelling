

class Radar(object):
    def __init__(self, taxi_ac_list, nodes_dict, edges_dict, t):
        self.curr_num_of_agents = len(taxi_ac_list)
        self.nodes_dict = nodes_dict
        self.edges_dict = edges_dict
        self.t = t

        self.acID_n_loc_lst = []
        self.curr_locations_list = []
        for ac in taxi_ac_list:
            self.curr_locations_list.append(ac.curr_loc)
            possible_next = []
            for neighbor in nodes_dict[ac.curr_loc]['neighbors']:
                if neighbor != ac.last_loc:
                    possible_next.append(neighbor)
            possible_next.append(ac.curr_loc)

            self.acID_n_loc_lst.append({'id':ac.id,'last_loc': ac.last_loc, 'curr_loc':ac.curr_loc, 'curr_loc_type': nodes_dict[ac.curr_loc]['type'],'dir': ac.heading, 'possible_next_locs': possible_next })


    def acs_heading_for_intersection(self, intersection_node):
        """
        Returns a list of node locations of planes that are one edge away from an
        intersection node and did not just come from that node
        """
        neighbors_of_intersect = list(self.nodes_dict[intersection_node]['neighbors'])

        if intersection_node == 15:
            neighbors_of_intersect.remove(53)
            neighbors_of_intersect.remove(60)
        elif intersection_node == 22:
            neighbors_of_intersect.remove(73)
        elif intersection_node == 28:
            neighbors_of_intersect.remove(106)
        elif intersection_node == 11:
            neighbors_of_intersect.remove(101)
            neighbors_of_intersect.remove(55)
        elif intersection_node == 18:
            neighbors_of_intersect.remove(68)
        elif intersection_node == 24:
            neighbors_of_intersect.remove(105)

        elif intersection_node == 29:
            neighbors_of_intersect.remove(99)
        elif intersection_node == 30:
            neighbors_of_intersect.remove(92)
        elif intersection_node == 31:
            neighbors_of_intersect.remove(93)
        elif intersection_node == 32:
            neighbors_of_intersect.remove(94)
        elif intersection_node == 33:
            neighbors_of_intersect.remove(100)

        for id_n_loc in self.acID_n_loc_lst:
            curr_loc = id_n_loc['curr_loc']
            last_loc = id_n_loc['last_loc']
            if intersection_node == 18:
                if 62 in neighbors_of_intersect:
                    if (curr_loc == 62 and last_loc == 11) or (curr_loc == 11 and last_loc == 56) or (curr_loc == 56 and last_loc == 12):
                        neighbors_of_intersect.remove(62)
            elif intersection_node == 12:
                if 56 in neighbors_of_intersect:
                    if (curr_loc == 56 and last_loc == 11) or (curr_loc == 11 and last_loc == 62) or (curr_loc == 62 and last_loc == 18):
                        neighbors_of_intersect.remove(56)

            elif intersection_node == 24:
                if 83 in neighbors_of_intersect:
                    if (curr_loc == 83 and last_loc == 29) or (curr_loc == 29 and last_loc == 88) or (curr_loc == 88 and last_loc == 30):
                        neighbors_of_intersect.remove(83)
            elif intersection_node == 30:
                if 88 in neighbors_of_intersect:
                    if (curr_loc == 88 and last_loc == 29) or (curr_loc == 29 and last_loc == 83) or (curr_loc == 83 and last_loc == 24):
                        neighbors_of_intersect.remove(88)

            elif intersection_node == 22:
                if 66 in neighbors_of_intersect:
                    if (curr_loc == 66 and last_loc == 15) or (curr_loc == 15 and last_loc == 59) or (curr_loc == 59 and last_loc == 14):
                        neighbors_of_intersect.remove(66)
            elif intersection_node == 14:
                if 59 in neighbors_of_intersect:
                    if (curr_loc == 59 and last_loc == 15) or (curr_loc == 15 and last_loc == 66) or (curr_loc == 66 and last_loc == 22):
                        neighbors_of_intersect.remove(59)

            elif intersection_node == 28:
                if 87 in neighbors_of_intersect:
                    if (curr_loc == 87 and last_loc == 33) or (curr_loc == 33 and last_loc == 91) or (curr_loc == 91 and last_loc == 32):
                        neighbors_of_intersect.remove(87)
            elif intersection_node == 32:
                if 91 in neighbors_of_intersect:
                    if (curr_loc == 91 and last_loc == 33) or (curr_loc == 33 and last_loc == 87) or (curr_loc == 87 and last_loc == 28):
                        neighbors_of_intersect.remove(91)



            if id_n_loc['curr_loc'] in neighbors_of_intersect and id_n_loc['last_loc'] != intersection_node:
                #acs_locs.append(id_n_loc['curr_loc'])
                #if id_n_loc['curr_loc'] in neighbors_of_intersect:
                neighbors_of_intersect.remove(id_n_loc['curr_loc'])
            elif id_n_loc['curr_loc'] == intersection_node and id_n_loc['last_loc'] in neighbors_of_intersect:
                #acs_locs.append(id_n_loc['curr_loc'])
                #if id_n_loc['last_loc'] in neighbors_of_intersect:
                neighbors_of_intersect.remove(id_n_loc['last_loc'])
        return neighbors_of_intersect


    def head_of_leftover_edges(self, leftover_between_nodes, intersection_node):
        """
        returns the heads of the edges that are not occupied
        """
        leftover_head_edges = []
        for between_node in leftover_between_nodes:
            for neighbor_of_between in self.nodes_dict[between_node]['neighbors']:
                if neighbor_of_between != intersection_node:
                    head_of_edge_node = neighbor_of_between
                    leftover_head_edges.append((head_of_edge_node, between_node))

        if intersection_node == 28 and 87 in leftover_between_nodes:
            leftover_head_edges.append((32,91))
        elif intersection_node == 22 and 66 in leftover_between_nodes:
            leftover_head_edges.append((14,59))
        elif intersection_node == 14 and 59 in leftover_between_nodes:
            leftover_head_edges.append((22, 66))
        elif intersection_node == 32 and 91 in leftover_between_nodes:
            leftover_head_edges.append((28, 87))
        elif intersection_node == 12 and 56 in leftover_between_nodes:
            leftover_head_edges.append((18, 62))
        elif intersection_node == 18 and 62 in leftover_between_nodes:
            leftover_head_edges.append((12, 56))
        elif intersection_node == 24 and 83 in leftover_between_nodes:
            leftover_head_edges.append((30, 88))
        elif intersection_node == 30 and 88 in leftover_between_nodes:
            leftover_head_edges.append((24, 83))

        return leftover_head_edges


    def head_of_leftover_edges_occupied(self, leftover_edges):
        "checks wethere the leftover heads are occupied and returns all occupied heads nodes"
        weight_edges = list(leftover_edges)
        for id_n_loc in self.acID_n_loc_lst:
            for leftover_head_edge in weight_edges:
                if id_n_loc['curr_loc'] == leftover_head_edge[0] and id_n_loc['last_loc'] != leftover_head_edge[1]:
                    weight_edges.remove(leftover_head_edge)
        return weight_edges

    def nr_acs_heading_for_intersection(self, intersection_node):
        """
        Returns a list of planes that are one edge away from an
        intersection node and did not just come from that node
        """
        count = 0
        neighbors_of_intersect = self.nodes_dict[intersection_node]['neighbors']
        for id_n_loc in self.acID_n_loc_lst:
            if id_n_loc['curr_loc'] in neighbors_of_intersect and id_n_loc['last_loc'] != intersection_node:
                count += 1
        return count

