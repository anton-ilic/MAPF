from single_agent_planner import get_location

from mpaf_solver import MAPFSolver

# gets a list of all agents that violate a passed positive constraint
# not sure this is actually very helpful. might not use it
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    max_time = max( len( path1 ), len( path2 ) )

    for i in range( max_time ):
        a1_loc = get_location( path1, i )
        a2_loc = get_location( path2, i )

        if (a1_loc is not None and a2_loc is not None):
            if a1_loc == a2_loc:
                return {
                    "loc": [ a1_loc ],
                    "timestep": i
                }
            
            if ( a1_loc == get_location( path2, i - 1 ) and
                a2_loc == get_location( path1, i - 1 ) ):
                return {
                    "loc": [ a1_loc, a2_loc ],
                    "timestep": i
                }
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []

    for a1, path1 in enumerate( paths ):
        # look through only the agents with higher numbers
        for a2 in range( a1 + 1, len( paths ) ):
            path2 = paths[ a2 ]
            col = detect_collision( path1, path2 )

            if col is not None:
                collisions.append( {
                    "a1": a1,
                    "a2": a2,
                    "loc": col[ "loc" ],
                    "timestep": col[ "timestep" ]
                } )
    
    return collisions

"""
A class for solving multi-agent pathfinding problems.

works by generating a set of paths with collisions and then resolving the collisions in the paths

requires the abstract function `resolve_collisions` to be defined
"""
class ResolvingSolver(MAPFSolver):
    pass