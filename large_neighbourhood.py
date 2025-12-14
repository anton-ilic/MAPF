import time as timer
import heapq
import random
from single_agent_planner import (
    a_star,
    get_timestep_for_location,
    search_closest
)
from resolvingSolver import (
    ResolvingSolver,
    detect_collisions,
    split_path,
    extend_path,
    find_shortest_non_final_path
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

def get_constraints_for_path( path, agent, final_goal ):

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
            # print( 'adding final constraint' )
            if final_goal:
                # marks this as a goal constraint if this is the agents final goal
                constraints[-1]['goal'] = True
            else:
                # otherwise, just assumes this agent will stay here for another timestep then move
                # Vertex constraint: no agent can be at this location at this time
                constraints.append({
                    'agent': agent,
                    'loc': [location],
                    'timestep': timestep + 1,
                })
                
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

    def __init__(self, my_map, starts, goals, final, search_type, weight):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals, final, search_type, weight)


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

        # print( f"agents in collisions: {agents_in_collisions}" )

        # randomly selects one agent involved in a collision to start the neighbourhood
        path_agent = random.choice( agents_in_collisions )
        neighbourhood = [ path_agent ]
        checked_agents = []

        while len( neighbourhood ) < neighbour_count:

            path_agent = None

            for agent in neighbourhood:
                if agent not in checked_agents:
                    # trys to find a another agent in the neighbourhood to check the paths of
                    # to expand the neighbourhood
                    path_agent = agent

                    # print( f"checking agent {path_agent}" )
                    break

            if path_agent is None:
                # no suitable agents found in current neighbourhood, select random agent from remaining agents
                remaining_agents = get_remaining_agents( self.num_of_agents, neighbourhood )
                path_agent = random.choice( remaining_agents )
                # print( f"randomly adding agent {path_agent}" )
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

        # print( f"selected neightbourhood: {neighbourhood}" )

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
        static_finals = []
        static_goals = []

        # finds all the static paths by non-neighbourhood agents that must be avoided
        for agent in remaining_agents:
            static_paths.append( split_path( self.paths[agent], timestep ) )
            static_finals.append( self.final_goals[ agent ] )
            static_goals.append( self.goals[ agent ] )
        
        # calculate window limit for constraints (shortest path not in neighbourhood)
        window_timestep = find_shortest_non_final_path( static_paths, static_finals, static_goals )
        window_limit = None
        if window_timestep is not None:
            window_limit = window_timestep[1] + 3  # Adjust for split path offset

        # print( f"shortest timestep for recalculation: {window_limit}")

        # print( "before recalcuation:" )
        # for agent, path in relevantPaths.items():
            # print( f"{agent}:{path}")

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
                path_constraints = get_constraints_for_path( path, agent, self.final_goals[ agent ] )
                # filter constraints by window if applicable
                if window_limit is not None:
                    path_constraints = [c for c in path_constraints if c['timestep'] < window_limit]
                constraints.extend( path_constraints )

            for prev_agent in planned_agents:
                # adds the constraints for each previous path
                path_constraints = get_constraints_for_path( relevantPaths[ prev_agent ], agent, self.final_goals[ agent ] )
                # filter constraints by window if applicable
                if window_limit is not None:
                    path_constraints = [c for c in path_constraints if c['timestep'] < window_limit]

                # print( f"converted path {relevantPaths[ prev_agent ]} into \n{path_constraints}" )
                constraints.extend( path_constraints )

            # calculates a new path directly to the goal
            new_path = a_star(
                self.my_map,
                new_starts[agent],
                self.goals[agent],
                self.heuristics[agent],
                agent,
                constraints,
                self.search_type,
                self.weight
            )

            if new_path is None:
                # Failed to find a path for this agent
                success = False

                print( f"failed to replan agent {agent} from {new_starts[agent]} to {self.goals[agent]}")

                # print( f"had constraints: {constraints}")
                break

            # stores the new path
            relevantPaths[agent] = new_path.copy()

            # print( f"replanned for agent {agent} with path {new_path}")

            # marks this agent as having completed its path calculations
            planned_agents.append( agent )

        # If successful, update the paths; otherwise keep old paths
        if success:
            for agent in neighbourhood:
                self.paths[agent] = extend_path( self.paths[agent], timestep, relevantPaths[agent] )
            print(f"Successfully replanned neighbourhood: {neighbourhood}")

            # print( "after recalcuation:" )
            # for agent, path in relevantPaths.items():
                # print( f"{agent}:{path}")
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
        # self.handle_shared_goals()

        print( f"recalculating for timestep {timestep}")

        # finds some set of collisions
        collisions = detect_collisions( self.paths )

        # finds the new shortest non-final path
        window_timestep = find_shortest_non_final_path( self.paths, self.final_goals, self.goals, timestep=timestep )
        
        # loops until no collisions left in the list
        while collisions != []:
            
            # finds a subset of agents to recalcuate paths for
            # identifiying the neighbourhood
            neighbourhood = self.find_neighbourhood( collisions, 6 )

            self.recalculate_paths_for_neighbourhood( neighbourhood, collisions, timestep )

            # rechecks for collisions
            collisions = detect_collisions( self.paths )

            print( f"{len(collisions)} collisions remaiming")

            print( f"remaining collisions: {collisions}" )

            # finds the new shortest non-final path
            window_timestep = find_shortest_non_final_path( self.paths, self.final_goals, self.goals, timestep=timestep )

            if window_timestep is not None:
                # filter out collisions that occur after the window
                window_limit = window_timestep[1] + 2
                collisions = [col for col in collisions if col['timestep'] < window_limit]

        print( f"next calculation: {window_timestep}" )
