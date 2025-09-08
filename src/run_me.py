"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import random
import time
from statistics import mean

import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import csv

from cbs import run_CBS
from distributed import run_Distributed
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 10000
#planner = "Distributed"
#planner = "CBS"
planner = "CBS" #choose which planner to use (currently only Independent is implemented)
testMode = False

#Visualization (can also be changed)
plot_graph = False   #show graph representation in NetworkX
visualization = True      #pygame visualization
visualization_speed = 0.2 #set at 0.1 as default


#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

def general_edge_constraints(edges_dict):
    general_constraint_list = [(4,43), (5,45),(11,101),(12,102)]
    for edge in general_constraint_list:
        edges_dict[edge]['weight'] = float('inf')


#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================

#%%Analysis variables
start_time = time.time()
time_per_ac_list = []

nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
general_edge_constraints(edges_dict)

graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time
dt = 0.5 #should be factor of 0.5 (0.5/dt should be integer)
t= 0
currACID = 0
departNodes = [1,2]
arriveNodes = [37, 38]
terminalNodes = [97, 34, 35, 36, 98]
maxAmountOfPlanes = 5
total_amountOfACS = 0
spawnedPlane = False
time_between_spawn = 0
allPaths = []
allConstraints = []
acDList = []

# departure edge case
ac1 = Aircraft(1, 'A', 40, 2, t, nodes_dict, edges_dict)
ac2 = Aircraft(2, 'D', 45, 1, t, nodes_dict, edges_dict)
ac1.last_loc = 42
ac2.last_loc = 41


if testMode:
    aircraft_lst.append(ac1)
    aircraft_lst.append(ac2)

def spawnACs(currID, spawnTime, map, edges_dict, last_spawn_arrive, last_spawn_97, last_spawn_34, last_spawn_35, last_spawn_36, last_spawn_98):
    """

    Args:
        currID:
        spawnTime:
        map:
        edges_dict:
        last_spawn_arrive:
        last_spawn_97:
        last_spawn_34:
        last_spawn_35:
        last_spawn_36:
        last_spawn_98:

    Returns:

    """
    spawn_gap_A = 3
    spawn_gap_D = 3
    chanceSpawn = 60
    acD97 = None
    acD34 = None
    acD35 = None
    acD36 = None
    acD98 = None
    acA = None

    if last_spawn_arrive >= spawn_gap_A:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acA = Aircraft(currID, 'A', random.choice(arriveNodes), random.choice(terminalNodes), spawnTime, map, edges_dict)
    if last_spawn_97 >= spawn_gap_D:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acD97 = Aircraft(currID, 'D', 97, random.choice(departNodes), spawnTime, map, edges_dict)
    if last_spawn_34 >= spawn_gap_D:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acD34 = Aircraft(currID, 'D', 34, random.choice(departNodes), spawnTime, map, edges_dict)
    if last_spawn_35 >= spawn_gap_D:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acD35 = Aircraft(currID, 'D', 35, random.choice(departNodes), spawnTime, map, edges_dict)
    if last_spawn_36 >= spawn_gap_D:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acD36 = Aircraft(currID, 'D', 36, random.choice(departNodes), spawnTime, map, edges_dict)
    if last_spawn_98 >= spawn_gap_D:
        if random.randint(0, 100) <= chanceSpawn:
            currID += 1
            acD98 = Aircraft(currID, 'D', 98, random.choice(departNodes), spawnTime, map, edges_dict)





def spawnAC(currID, spawnTime, map, edges_dict):
    """
    there's a 75% chance an aircraft spawns at all
    and if this is satisfied then its further broken down
    into either only an arriving aircraft spawns (45%)
    or only a departing aircraft spawns (45%) or an arriving
     and departing aircraft spawn simultaneously (10%).
    """
    chanceSpawn = 75
    chanceDepartingFlight = 45
    chanceArrivingFlight = 45
    acD = None
    acA = None
    if random.randint(0, 100) <= chanceSpawn:
        spawnedPlane = True
        departArriveOrBoth = random.randint(0, 100)
        if departArriveOrBoth <= chanceDepartingFlight:  # 40% chance Departing flight
            currID += 1
            acD = Aircraft(currID, 'D', random.choice(terminalNodes), random.choice(departNodes), spawnTime, map, edges_dict)
        elif chanceDepartingFlight < departArriveOrBoth <= chanceDepartingFlight + chanceArrivingFlight:  # 40% chance Arriving flight
            currID += 1
            acA = Aircraft(currID, 'A', random.choice(arriveNodes), random.choice(terminalNodes), spawnTime, map, edges_dict)
        else:  # 20% chance a departing an arriving flight spawn at the same time
            currID += 1
            acA = Aircraft(currID, 'A', random.choice(arriveNodes), random.choice(terminalNodes), spawnTime, map, edges_dict)
            currID += 1
            acD = Aircraft(currID, 'D', random.choice(terminalNodes), random.choice(departNodes), spawnTime, map, edges_dict)
    else:
        spawnedPlane = False

    return spawnedPlane, acA, acD, currID

if testMode and planner == 'CBS':
    run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t)


print("Simulation Started")
resolve = True
all_arrived = []
while running:
    t= round(t,2)    
       
    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    for ac in aircraft_lst:
        all_arrived.append(ac.status == "arrived")
    
    if len(aircraft_lst) >= 80 and all(all_arrived):
        running = False
        pg.quit()
        print("Simulation Stopped")
        break
    all_arrived = []
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                current_states[ac.id] = {"ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed) 
      

    amountOfACs = 0
    next_locations = []
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            amountOfACs +=1
            next_locations.append(ac.from_to[0])
            next_locations.append(ac.from_to[1])

    runway_n_terminal_spawn_rules = []
    term_constraints = [83,29,99,92,93,94,100,33,87,37,38]
    terms = [97,97,97,34,35,36,98,98,98,37,38]

    for ac in aircraft_lst:
        if ac.status == "taxiing":
            for i in range(len(term_constraints)):
                if ac.curr_loc == term_constraints[i]:
                    runway_n_terminal_spawn_rules.append(terms[i])


    total_amountOfACS += amountOfACs

    time_between_spawn +=1

    if amountOfACs < maxAmountOfPlanes and time_between_spawn >= 3 and len(aircraft_lst) < 80:
        spawnedPlane, acA, acD, currACID = spawnAC(currACID, t, nodes_dict, edges_dict)
    else:
        spawnedPlane = False

    if not testMode or planner == "Distributed":
        #Do planning
        if planner == "Independent":
            if t == 1:
                run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
        elif planner == "Prioritized":
            if spawnedPlane == True: #plan for every plane that spawns
                if acA is not None: #prioritize the planning of arriving airplanes vs departing
                    t1 = time.time()
                    allPaths, allConstraints, success = run_prioritized(acA, nodes_dict, edges_dict, heuristics, t, allPaths, allConstraints)
                    t2 = time.time()
                    time_per_ac_list.append(t2 - t1)
                    if success == True:
                        aircraft_lst.append(acA)
                    else:
                        currACID -= 1
                if acD is not None:
                    t1 = time.time()
                    allPaths, allConstraints, success = run_prioritized(acD, nodes_dict, edges_dict, heuristics, t, allPaths, allConstraints)
                    t2 = time.time()
                    time_per_ac_list.append(t2 - t1)
                    if success == True:
                        aircraft_lst.append(acD)
                    else:
                        currACID -= 1
        elif planner == "CBS":
            if spawnedPlane: #plan for every plane that spawns
                if acA is not None:
                    if acA.start not in next_locations:
                        aircraft_lst.append(acA)
                    else:
                        currACID -=1
                if acD is not None:
                    if acD.start not in next_locations:
                        aircraft_lst.append(acD)
                        acDList.append(acD)
                    else:
                        currACID-=1
                t1 = time.time()
                run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
                t2 = time.time()
                time_per_ac_list.append(t2-t1)
        elif planner == "Distributed":
            if not testMode:
                if spawnedPlane: #plan for every plane that spawns
                    if acA is not None:
                        if acA.start not in runway_n_terminal_spawn_rules:
                            aircraft_lst.append(acA)
                            time_between_spawn =  0
                        else:
                            currACID -=1
                    if acD is not None:
                        if acD.start not in runway_n_terminal_spawn_rules:
                            aircraft_lst.append(acD)
                            acDList.append(acD)
                            time_between_spawn =  0
                        else:
                            currACID-=1
            t1 = time.time()
            run_Distributed(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
            t2 = time.time()
            time_per_ac_list.append(t2 - t1)


        else:
            raise Exception("Planner:", planner, "is not defined.")
                       
    #Move the aircraft that are taxiing
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            ac.move(dt, t)
    #print(t)
    t = t + dt

endTime = time.time()
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?


average_calc_time_per_plane = mean(time_per_ac_list)
max_time_per_plane = max(time_per_ac_list)
min_time_per_plane = min(time_per_ac_list)
average_ac_amount = total_amountOfACS/(t/dt)

route_time_list = []
nr_ac = 0
for ac in aircraft_lst:
    if ac.endTime != None:
        route_time_list.append(ac.endTime-ac.spawntime)
        nr_ac += 1
avg_route_time_per_ac = mean(route_time_list)
max_route_time = max(route_time_list)
max_route_length = max_route_time / dt
avg_route_length = avg_route_time_per_ac / dt


fieldNames = ['Planner', 'Visualization','Simulation Time','Max nr of acs in grid', 'dt', 'Nr Spawned acs', 'Avg nr of acs in grid', 'Mean calc t', 'Max calc t', 'Total time','Average route time per ac', 'Max route time per ac', 'Average route length per ac', 'Max route length per ac']

row = [
    {'Planner': planner,
     'Visualization': visualization,
     'Simulation Time': t,
     'Max nr of acs in grid': maxAmountOfPlanes,
     'dt': dt,
     'Nr Spawned acs': currACID,
     'Avg nr of acs in grid': average_ac_amount,
     'Mean calc t': average_calc_time_per_plane,
     'Max calc t': max_time_per_plane,
     'Total time': endTime - start_time,
     'Average route time per ac': avg_route_time_per_ac,
     'Max route time per ac': max_route_time,
     'Average route length per ac': avg_route_length,
     'Max route length per ac': max_route_length

     }
]



with open('analysis.csv', 'a', encoding='UTF8', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldNames)
    if os.stat('analysis.csv').st_size == 0:
        writer.writeheader()
        writer.writerows(row)
    else:
        writer.writerows(row)




