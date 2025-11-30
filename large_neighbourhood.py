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
    
    def generate_nodes( self, collision, parent_node, disjoint=True ):
        if disjoint:
            constr_split = disjoint_splitting( collision )
        else:
            constr_split = standard_splitting(collision)
        
        for constr in constr_split:
            # creates a new node with the parent constraints and the new constraint
            new_node = {
                "cost": 0,
                "constraints": parent_node[ "constraints" ].copy(),
                "paths": parent_node[ "paths" ].copy(),
                "collisions": []
            }
            new_node[ "constraints" ].append( constr )

            # generates a list of all agents who need to have thier paths recalcuated
            updated_agents = [ constr[ "agent" ] ]
            if "positive" in constr and constr[ "positive" ]:
                for agent in paths_violate_constraint( constr, new_node[ "paths" ] ):
                    updated_agents.append( agent )

            skipped_path = False

            #calculate new paths with new constraints
            for agent in updated_agents:
                path = a_star( self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                               agent, new_node[ "constraints" ] )
                
                # checks if the path was calculated successfully
                if path is not None:
                    new_node[ "paths" ][ agent ] = path
                else:
                    skipped_path = True

            if skipped_path:
                # if a new path was not calculated for every agent, 
                # don't add this node
                continue

            # calculates cost and collisions on new path
            new_node[ "cost" ] = get_sum_of_cost( new_node[ "paths" ] )
            new_node[ "collisions" ] = detect_collisions( new_node[ "paths" ] )

            self.push_node( new_node )

    def calculate_colliding_paths(self):
        self.paths = []

        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            self.paths.append(path)

    # used to handle a collision where both agents have the same goal and
    # the collision occurs at that goal.
    # in this case, its impossible for both agents to reach thier goals so
    # we simple drop the end of the path of one agent and wait for the other to move
    # to recalcuate this path
    def handleSharedGoalCollision( self, collision, node ):
        delayedAgent = collision[ "a1" ]
        priorityAgent = collision[ "a2" ]

        location = collision[ "loc" ][0]

        if location != self.goals[ delayedAgent ] or location != self.goals[ priorityAgent ]:
            raise RuntimeError( "handleSharedGoalCollision called on non shared goal collision" )
        
        new_node = {
            "cost": 0,
            "constraints": node[ "constraints" ].copy(),
            "paths": node[ "paths" ].copy(),
            "collisions": []
        }

        # removes the end of the path from the delayed agent
        new_node[ "paths" ][ delayedAgent ].pop()

        # marks the skipped agent to be updated at the next cycle
        self.mark_agent_for_updates( delayedAgent )

        # calculates cost and collisions on new path
        new_node[ "cost" ] = get_sum_of_cost( new_node[ "paths" ] )
        new_node[ "collisions" ] = detect_collisions( new_node[ "paths" ] )

        self.push_node( new_node )




    def resolve_collisions(self, timestep=0):
        """ Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions

        disjoint    - use disjoint splitting or not
        """

        pass
        # should implement later

    def find_solution(self):
        self.calculate_colliding_paths()

        self.resolve_collisions()

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

        # re-calcuates the heuristic to use the new goal
        self.heuristics[ agent ] = compute_heuristics( self.my_map, goal )

        self.mark_agent_for_updates( agent )

        self.get_paths_for_pending_agents( timestep )

        self.resolve_collisions(timestep=timestep)

        return self.paths


    """def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print( "proposed paths: {}".format( node[ "paths" ] ) )"""
