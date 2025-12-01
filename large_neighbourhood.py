import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from mpaf_solver import MAPFSolver

from cbs import (
    paths_violate_constraint,
    detect_collision,
    detect_collisions,

)

def get_remaining_agents( num_of_agents, neighbourhood ):
    """
    gets the list of agents that are not in the neighbourhood
    """
    remaining_agents = []

    for agent in range( num_of_agents ):
        if agent not in neighbourhood:
            remaining_agents.append( agent )

    return remaining_agents

class LargeNeighbourhoodSolver(MAPFSolver):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals)

        self.paths = []

        # stores any agents that need new paths in a later update
        self.pending_agents = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
    
    def calculate_colliding_paths(self):
        self.paths = []

        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            self.paths.append(path)


    def find_neighbourhood( self, collisions, neighbour_count ):
        """
        finds a subset of agents to generate new paths for

        must return a list of agent ID's that can be used as the neighbourhood 
        """
        pass

    def recalculate_paths_for_neighbourhood( self, neighbourhood ):
        """
        recalculates a new set of paths for all agents in the neighbourhood

        treats all paths not in the neighbourhood as a set of moving obstacles

        if no paths are possible, leave paths as is
        """
        pass


    def resolve_collisions(self, timestep=0):
        """ Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions

        disjoint    - use disjoint splitting or not
        """

        # finds some set of collisions
        collisions = detect_collisions( self.paths )

        # loops until no collisions left in the list
        while collisions != []:

            # finds a subset of agents to recalcuate paths for
            # identifiying the neighbourhood
            neighbourhood = self.find_neighbourhood( collisions, 4 )

            self.recalculate_paths_for_neighbourhood( neighbourhood )

            # rechecks for collisions
            collisions = detect_collisions( self.paths )


    def find_solution(self):
        self.start_time = timer.time()

        self.calculate_colliding_paths()

        self.resolve_collisions()

        print( f"recalculating paths for all agents" )
        self.print_results( self.paths )

        return self.paths
    
    def mark_agent_for_updates( self, agent ):
        if agent not in self.pending_agents:
            self.pending_agents.append( agent )
    
    # calculates conflicting paths for all plending agents
    def get_paths_for_pending_agents( self, timestep ):

        for agent in self.pending_agents:
            agentPath = []
            if len(self.paths) > agent:
                agentPath = self.paths[ agent ]

            start = self.starts[agent]

            if agentPath != []:
                if ( len(agentPath) > timestep ):
                    # uses timestep location as start location
                    start = agentPath[ timestep ]
                    # cuts agent path to timestep length
                    agentPath = agentPath[:timestep]
                else:
                    # lengthens the path to satisfy the timestep requirements
                    while len( agentPath ) < timestep :
                        agentPath.append( agentPath[ -1 ] )
                    start = agentPath[ -1 ]
            else:
                # lengthens the path to satisfy the timestep requirements
                while len( agentPath ) < timestep:
                    agentPath.append( start )

            # calculates a new path
            newPath = a_star(self.my_map, start, self.goals[ agent ], self.heuristics[ agent ],
                            agent, [] )
            
            # adds it to the current path
            agentPath.extend(newPath)

            self.paths[agent] = agentPath

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


    """def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print( "proposed paths: {}".format( node[ "paths" ] ) )"""
