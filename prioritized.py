
def run_prioritized(ac, nodes_dict, edges_dict, heuristics, t, allPaths, allConstraints):
    ac.status = "taxiing"
    ac.position = nodes_dict[ac.start]["xy_pos"]
    path, ac_constraints, success = ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t, allConstraints)
    if success == False:
        ac.status = None
        return allPaths, allConstraints, success
    allPaths.append(path)
    allConstraints.extend(ac_constraints)
    return allPaths, allConstraints, success

