import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star
from resolvingSolver import (
    ResolvingSolver,
    detect_collisions
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

        # returns the found neighbourhood
        return neighbourhood

    def recalculate_paths_for_neighbourhood( self, neighbourhood, collisions ):
        """
        recalculates a new set of paths for all agents in the neighbourhood

        treats all paths not in the neighbourhood as a set of moving obstacles
        
        preserves already-traveled portion of paths before earliest collision

        if no paths are possible, leave paths as is
        """
        # Find the earliest collision timestep to determine where to start replanning from
        earliest_collision_time = min(collision['timestep'] for collision in collisions)
        replan_from_timestep = max(0, earliest_collision_time - 1)
        
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
            # BUT: Only if this agent's goal is NOT shared with neighbourhood agents
            if len(path) > 0:
                final_location = path[-1]
                goal_of_this_agent = self.goals[agent]
                
                # Check if any neighbourhood agent shares this goal
                goal_is_shared = any(self.goals[n_agent] == goal_of_this_agent 
                                    for n_agent in neighbourhood)
                
                # Only add future goal constraints if the goal is NOT shared
                if not goal_is_shared:
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
        
        # Check for shared goals within the neighbourhood
        # Group agents by their goals
        goal_groups = {}
        for agent in neighbourhood:
            goal = self.goals[agent]
            if goal not in goal_groups:
                goal_groups[goal] = []
            goal_groups[goal].append(agent)
        
        # Identify which agents share goals
        agents_with_shared_goals = []
        for goal, agents in goal_groups.items():
            if len(agents) > 1:
                agents_with_shared_goals.extend(agents)
        
        # Try to compute new paths for each agent in the neighbourhood
        # using prioritized planning (agents planned in order)
        # For agents with shared goals, plan in sequence with temporal separation
        new_paths = {}
        success = True
        
        # Sort neighbourhood so that agents with shared goals are planned in a fixed order
        # This ensures consistent priority and prevents both from being replanned to same solution
        sorted_neighbourhood = sorted(neighbourhood)
        
        for agent in sorted_neighbourhood:
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
                # BUT: Only if this agent's goal is NOT shared with remaining neighbourhood agents
                if len(planned_path) > 0:
                    final_location = planned_path[-1]
                    final_timestep = len(planned_path) - 1
                    goal_of_planned_agent = self.goals[planned_agent]
                    
                    # Check if any remaining (not yet planned) neighbourhood agent shares this goal
                    remaining_neighbourhood = [a for a in neighbourhood if a not in new_paths]
                    goal_is_shared = any(self.goals[n_agent] == goal_of_planned_agent 
                                        for n_agent in remaining_neighbourhood)
                    
                    # Only add future goal constraints if the goal is NOT shared
                    if not goal_is_shared:
                        for t in range(final_timestep + 1, final_timestep + 100):
                            agent_constraints.append({
                                'agent': agent,
                                'loc': [final_location],
                                'timestep': t,
                                'goal': True
                            })
                    else:
                        # For shared goals, add temporal separation:
                        # The current agent cannot reach the shared goal until the planned agent
                        # has been there and potentially moved on
                        # Block the shared goal location for a sufficient duration
                        # This forces the second agent to wait or find alternate timing
                        for t in range(max(0, final_timestep - 2), final_timestep + 5):
                            agent_constraints.append({
                                'agent': agent,
                                'loc': [final_location],
                                'timestep': t
                            })
            
            # Preserve already-traveled portion of path and replan from collision point
            preserved_path = []
            start_location = self.starts[agent]
            
            if replan_from_timestep > 0:
                print( f"extending path for agent {agent}" )
                if len(self.paths[agent]) > replan_from_timestep:
                    # Keep the path up to replan_from_timestep
                    preserved_path = self.paths[agent][:replan_from_timestep]
                else:
                    while len(self.paths[agent]) < replan_from_timestep:
                        # extends the path by assuming the agent stays in its last timestep
                        self.paths[agent].append(self.paths[agent][-1])

                    # copies the previous path to preserve
                    preserved_path = self.paths[agent].copy()
                # Start replanning from the last preserved location
                start_location = preserved_path[-1]
            
            # Compute new path with A* from the preserved endpoint (or start)
            new_path = a_star(
                self.my_map,
                start_location,
                self.goals[agent],
                self.heuristics[agent],
                agent,
                agent_constraints
            )
            
            if new_path is None:
                # Failed to find a path for this agent
                success = False
                break
            
            # Combine preserved path with new path
            # Note: new_path includes the start_location, so skip it to avoid duplication
            if len(preserved_path) > 0:
                full_path = preserved_path + new_path[1:]
            else:
                full_path = new_path
            
            new_paths[agent] = full_path
        
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

            self.recalculate_paths_for_neighbourhood( neighbourhood, collisions )

            # rechecks for collisions
            collisions = detect_collisions( self.paths )
