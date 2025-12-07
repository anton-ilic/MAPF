from single_agent_planner import (
    get_location,
    compute_heuristics,
    a_star
)

import time as timer

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

def split_path( path, timestep ):
    """
    takes a path and splits off the portion that takes place after the specified timestep

    if required extends the path the specified timestep so theres something to split
    
    :param path: the path to split
    :param timestep: the timestep to split the path at
    """
    if timestep > 0:
        if len( path ) < timestep:
            return [ path[ -1 ] ]
        else:
            return path[timestep-1:]
    else:
        return path
    
def extend_path( path, timestep, new_path ):
    """
    replaces the segement of a path starting at timestep with the provided new path

    extends the path as needed to allow indexing at timestep

    ex. extending the path `[(1, 1), (1, 2), (1, 3), (1, 4)]` from timestep
    2 with [(1, 3), (2, 3), (3, 3)] results in 
    [(1, 1), (1, 2), (1, 3), (2, 3), (3, 3)]
    
    :param path: Description
    :param timestep: Description
    :param new_path: Description
    """

    if timestep > 0:
        # lengthens the path if needed
        while len( path ) < timestep:
            path.append( path[-1] )

        # cuts and re-extends the path
        path = path[:timestep-1]
        path.extend(new_path)

        return path
    else:
        return new_path
    
"""
A class for solving multi-agent pathfinding problems.

works by generating a set of paths with collisions and then resolving the collisions in the paths

requires the abstract function `resolve_collisions` to be defined
"""
class ResolvingSolver(MAPFSolver):
    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals)

        self.paths = []

        # for agents that can't make it to the goal, allows them to be
        # n steps away from the goal instead
        self.nonarriveDist = int( ( self.num_of_agents / 4 ) + 1 )

        # stores any agents that need new paths in a later update
        self.pending_agents = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def calculate_colliding_paths(self):
        """
        calculates a set of colliding paths for the agents in the solver

        creates a new path for each agent
        """
        self.paths = []

        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            self.paths.append(path)

    def resolve_collisions(self, timestep=0):
        """
        Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions
        """
        raise BaseException( "abstract function resolve_collisions not implemented" )
    
    def find_solution(self):

        self.start_time = timer.time()

        self.calculate_colliding_paths()

        self.resolve_collisions()

        print( "calculating all paths" )
        self.print_results( self.paths )

        return self.paths
    
    def mark_agent_for_updates( self, agent ):
        if agent not in self.pending_agents:
            self.pending_agents.append( agent )

    def is_marked_for_updates( self, agent ):
        return agent in self.pending_agents
    
    # calculates conflicting paths for all plending agents
    def get_paths_for_pending_agents( self, timestep ):

        for agent in self.pending_agents:
            agentPath = []
            if len(self.paths) > agent:
                agentPath = self.paths[ agent ]

            start = self.starts[agent]

            if agentPath != []:
                if ( len(agentPath) >= timestep ):
                    # uses timestep location as start location
                    start = agentPath[ timestep-1]
                else:
                    # uses the end of the path as the start
                    start = agentPath[ -1 ]

            # calculates a new path
            newPath = a_star(self.my_map, start, self.goals[ agent ], self.heuristics[ agent ],
                            agent, [] )

            self.paths[agent] = extend_path( agentPath, timestep, newPath )

        self.pending_agents = []

    def update_goal(self, agent, goal, timestep):
        self.goals[ agent ] = goal

        self.start_time = timer.time()

        # re-calcuates the heuristic to use the new goal
        self.heuristics[ agent ] = compute_heuristics( self.my_map, goal )

        self.mark_agent_for_updates( agent )

        self.get_paths_for_pending_agents( timestep )

        self.resolve_collisions(timestep=timestep)

        print( f"recalculating paths for agent {agent}" )
        self.print_results( self.paths )

        return self.paths