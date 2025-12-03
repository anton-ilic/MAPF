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
        if self.num_of_agents <= neighbour_count:
            # neighbourhood count includes all agents, return list of all agents
            neighbour_count = self.num_of_agents
            return list( range( self.num_of_agents ) )

        agents_in_collisions = []

        # gets a list of all agents involved in a collision
        for col in collisions:
            col_agents = [ col[ 'a1' ], col[ 'a2' ] ]

            for agent in col_agents:
                if agent not in agents_in_collisions:
                    agents_in_collisions.append( agent )

        # randomly selects one agent involved in a collision to start the neighbourhood
        path_agent = random.randint( 0, len( agents_in_collisions ) - 1)
        neighbourhood = [ path_agent ]
        checked_agents = []

        while len( neighbourhood ) < neighbour_count:

            path_agent = None

            for agent in neighbourhood:
                if agent not in checked_agents:
                    # trys to find a another agent in the neighbourhood to check the paths of
                    # to expand the neighbourhood
                    path_agent = agent
                    break

            if path_agent is None:
                # no suitable agents found in current neighbourhood, select random agent from remaining agents
                remaining_agents = get_remaining_agents( self.num_of_agents, neighbourhood )
                path_agent = random.choice( remaining_agents )
                neighbourhood.append( path_agent )

            for col in collisions:
                if len( neighbourhood ) >= neighbour_count:
                    # stop checking if neighbourhood is full
                    break

                # adds any agents involved in collisions with the current agent to the neighbourhood
                if col[ 'a1' ] == path_agent and col[ 'a2' ] not in neighbourhood:
                    neighbourhood.append( col[ 'a2' ] )
                if col[ 'a2' ] == path_agent and col[ 'a1' ] not in neighbourhood:
                    neighbourhood.append( col[ 'a1' ] )

            # mark that this agents path has been checked and that all
            # the agents it collides with are in the neighbourhood
            checked_agents.append( path_agent )

        # returns the found neighbourhood
        return neighbourhood

    def recalculate_paths_for_neighbourhood( self, neighbourhood ):
        """
        recalculates a new set of paths for all agents in the neighbourhood

        treats all paths not in the neighbourhood as a set of moving obstacles

        if no paths are possible, leave paths as is
        """
        # Get all agents NOT in the neighbourhood
        remaining_agents = get_remaining_agents( self.num_of_agents, neighbourhood )
        
        # Build constraints from the paths of agents outside the neighbourhood
        # These agents' paths are treated as immovable obstacles
        constraints = []
        
        for agent in remaining_agents:
            path = self.paths[agent]
            # Create vertex constraints for each timestep of this agent's path
            for timestep, location in enumerate(path):
                # Vertex constraint: no neighbourhood agent can be at this location at this time
                constraints.append({
                    'agent': -1,  # applies to all neighbourhood agents
                    'loc': [location],
                    'timestep': timestep
                })
                
                # Edge constraint: prevent swapping positions
                if timestep > 0:
                    prev_location = path[timestep - 1]
                    # Prevent neighbourhood agents from using this edge
                    constraints.append({
                        'agent': -1,
                        'loc': [prev_location, location],
                        'timestep': timestep
                    })
            
            # Add goal constraints to prevent neighbourhood agents from occupying
            # the final position after the path ends
            if len(path) > 0:
                final_location = path[-1]
                final_timestep = len(path) - 1
                # Extend constraint into the future
                for t in range(final_timestep + 1, final_timestep + 100):
                    constraints.append({
                        'agent': -1,
                        'loc': [final_location],
                        'timestep': t,
                        'goal': True
                    })
        
        # Store old paths in case we need to revert
        old_paths = {agent: self.paths[agent].copy() for agent in neighbourhood}
        
        # Try to compute new paths for each agent in the neighbourhood
        # using prioritized planning (agents planned in order)
        new_paths = {}
        success = True
        
        for agent in neighbourhood:
            # Collect constraints from:
            # 1. Non-neighbourhood agents (already in constraints)
            # 2. Already-planned neighbourhood agents (to avoid collisions within neighbourhood)
            agent_constraints = []
            
            # Add constraints from non-neighbourhood agents
            for constr in constraints:
                agent_constraints.append({
                    'agent': agent,
                    'loc': constr['loc'],
                    'timestep': constr['timestep'],
                    'goal': constr.get('goal', False)
                })
            
            # Add constraints from already-planned neighbourhood agents
            for planned_agent in new_paths:
                planned_path = new_paths[planned_agent]
                for timestep, location in enumerate(planned_path):
                    agent_constraints.append({
                        'agent': agent,
                        'loc': [location],
                        'timestep': timestep
                    })
                    
                    if timestep > 0:
                        prev_location = planned_path[timestep - 1]
                        agent_constraints.append({
                            'agent': agent,
                            'loc': [prev_location, location],
                            'timestep': timestep
                        })
                
                # Goal constraint for already-planned agents
                if len(planned_path) > 0:
                    final_location = planned_path[-1]
                    final_timestep = len(planned_path) - 1
                    for t in range(final_timestep + 1, final_timestep + 100):
                        agent_constraints.append({
                            'agent': agent,
                            'loc': [final_location],
                            'timestep': t,
                            'goal': True
                        })
            
            # Compute new path with A* using all constraints
            new_path = a_star(
                self.my_map,
                self.starts[agent],
                self.goals[agent],
                self.heuristics[agent],
                agent,
                agent_constraints
            )
            
            if new_path is None:
                # Failed to find a path for this agent
                success = False
                break
            
            new_paths[agent] = new_path
        
        # If successful, update the paths; otherwise keep old paths
        if success:
            for agent in neighbourhood:
                self.paths[agent] = new_paths[agent]
            print(f"Successfully replanned neighbourhood: {neighbourhood}")
        else:
            # Revert to old paths
            for agent in neighbourhood:
                self.paths[agent] = old_paths[agent]
            print(f"Failed to replan neighbourhood {neighbourhood}, keeping old paths")


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
            neighbourhood = self.find_neighbourhood( collisions, 3 )

            print( f"testing output: \n\ncollisions: {collisions}\nneighbourhood:{neighbourhood}" )

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
