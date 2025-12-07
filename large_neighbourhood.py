import time as timer
import heapq
import random
from single_agent_planner import a_star, get_timestep_for_location
from resolvingSolver import (
    ResolvingSolver,
    detect_collisions,
    split_path,
    extend_path
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

def get_constraints_for_path( path, agent ):

    constraints = []

    # Create vertex constraints for each timestep of this agent's path
    for timestep, location in enumerate(path):
        # Vertex constraint: no agent can be at this location at this time
        constraints.append({
            'agent': agent,
            'loc': [location],
            'timestep': timestep,
        })

        # checks if this is the last location in the path
        if timestep == len(path) - 1:
            # marks this as a goal constraint
            constraints[-1]['goal'] = True
                
        # Edge constraint: prevent swapping positions
        if timestep > 0:
            prev_location = path[timestep - 1]
            # Prevent agents from using this edge
            constraints.append({
                'agent': agent,
                'loc': [location, prev_location],
                'timestep': timestep
            })
    
    return constraints

class LargeNeighbourhoodSolver(ResolvingSolver):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals)


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

        random.shuffle( neighbourhood )

        # returns the found neighbourhood
        return neighbourhood

    def recalculate_paths_for_neighbourhood( self, neighbourhood, collisions, timestep ):
        """
        recalculates a new set of paths for all agents in the neighbourhood

        treats all paths not in the neighbourhood as a set of moving obstacles
        
        preserves already-traveled portion of paths before earliest collision

        if no paths are possible, leave paths as is
        """
        
        relevantPaths = {}
        new_starts = {}

        # splits the complete path portions from the complete ones
        for agent in neighbourhood:
            relevantPaths[agent] = split_path( self.paths[ agent ], timestep )
            new_starts[agent] = relevantPaths[agent][0]

        # Get all agents NOT in the neighbourhood
        remaining_agents = get_remaining_agents( self.num_of_agents, neighbourhood )

        static_paths = []

        # finds all the static paths by non-neighbourhood agents that must be avoided
        for agent in remaining_agents:
            static_paths.append( split_path( self.paths[agent], timestep ) )

        # Store old paths in case we need to revert
        old_paths = {agent: self.paths[agent].copy() for agent in neighbourhood}

        planned_agents = []

        # a flag to mark if the replanning was successful
        success = True

        for agent in neighbourhood:

            #collect constraints from:
            # 1. static paths
            # 2. previously calculated agents

            constraints = []

            for path in static_paths:
                # adds the constraints for each static path
                constraints.extend( get_constraints_for_path( path, agent ) )

            for prev_agent in planned_agents:
                print( f"adding constraints for prev agent {prev_agent}")
                # adds the constraints for each previous path
                constraints.extend( get_constraints_for_path( relevantPaths[ prev_agent ], agent ) )

                print( f"constraints:\n{constraints}")

            if self.is_marked_for_updates( agent ):
                # calculates a new path that goes close to the goal but doesn't arrive at it
                new_path = a_star(
                    self.my_map,
                    new_starts[agent],
                    self.goals[agent],
                    self.heuristics[agent],
                    agent,
                    constraints,
                    goalDist=self.nonarriveDist
                )
            else:
                # calculates a new path directly to the goal
                new_path = a_star(
                    self.my_map,
                    new_starts[agent],
                    self.goals[agent],
                    self.heuristics[agent],
                    agent,
                    constraints
                )

            if new_path is None:
                # Failed to find a path for this agent
                success = False
                break

            # stores the new path
            relevantPaths[agent] = new_path.copy()

            print( f"replanned for agent {agent} with path {new_path}")

            # marks this agent as having completed its path calculations
            planned_agents.append( agent )

        # If successful, update the paths; otherwise keep old paths
        if success:
            for agent in neighbourhood:
                self.paths[agent] = extend_path( self.paths[agent], timestep, relevantPaths[agent] )
            print(f"Successfully replanned neighbourhood: {neighbourhood}")
        else:
            # Revert to old paths
            for agent in neighbourhood:
                self.paths[agent] = old_paths[agent]
            print(f"Failed to replan neighbourhood {neighbourhood}, keeping old paths")

    def check_shared_goals( self, a1, a2 ):
        if self.goals[a1] != self.goals[a2] or a1 == a2:
            # if these agents do not share goals then skip this
            return
        
        if self.is_marked_for_updates( a1 ) or self.is_marked_for_updates( a2 ):
            # one of these two agents is already marked to not go to its goal, skip this
            return
        
        # gets the conflicting goal locations
        location = self.goals[a1]
        
        # figures out when both agents arrive at that goal location
        a1timestep = get_timestep_for_location( self.paths[ a1 ], location )
        a2timestep = get_timestep_for_location( self.paths[ a2 ], location )

        if a1timestep < a2timestep:
            # a1 arrives first, mark a2 for incomplete pathfinding
            self.mark_agent_for_updates(a2)
        else:
            # a2 arrives first, mark a1 for incomplete pathfinding
            self.mark_agent_for_updates(a1)

    def handle_shared_goals( self ):
        for a1 in range( self.num_of_agents ):
            for a2 in range( a1, self.num_of_agents ):
                self.check_shared_goals( a1, a2 )


    def resolve_collisions(self, timestep=0):
        """ Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions

        disjoint    - use disjoint splitting or not
        """

        # checks for and marks shared goal agents
        self.handle_shared_goals()

        # finds some set of collisions
        collisions = detect_collisions( self.paths )
        
        # loops until no collisions left in the list
        while collisions != []:
            
            # finds a subset of agents to recalcuate paths for
            # identifiying the neighbourhood
            neighbourhood = self.find_neighbourhood( collisions, 3 )

            self.recalculate_paths_for_neighbourhood( neighbourhood, collisions, timestep )

            # rechecks for collisions
            collisions = detect_collisions( self.paths )
