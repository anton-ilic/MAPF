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
            
            if replan_from_timestep > 0 and len(self.paths[agent]) > replan_from_timestep:
                # Keep the path up to replan_from_timestep
                preserved_path = self.paths[agent][:replan_from_timestep]
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

        # Track collision history to detect infinite loops
        collision_history = []
        max_same_collision = 10  # Increased for sorting instances with dynamic goals
        max_total_iterations = 50  # Safety limit
        iteration_count = 0
        
        # loops until no collisions left in the list
        while collisions != []:
            iteration_count += 1
            
            # Safety check for too many iterations
            if iteration_count > max_total_iterations:
                print(f"Reached maximum iterations ({max_total_iterations}), stopping collision resolution")
                print(f"Remaining collisions: {len(collisions)}")
                break

            # Check if we're stuck in a loop with the same collision
            collision_key = str(sorted([(c['a1'], c['a2'], tuple(c['loc']), c['timestep']) for c in collisions]))
            collision_history.append(collision_key)
            
            # Count how many times this exact collision set has appeared
            collision_count = collision_history.count(collision_key)
            
            if collision_count > max_same_collision:
                print(f"Detected repeated collision after {collision_count} attempts")
                
                # Check if this is a shared goal problem
                shared_goal_collisions = []
                for collision in collisions:
                    a1, a2 = collision['a1'], collision['a2']
                    if self.goals[a1] == self.goals[a2]:
                        shared_goal_collisions.append((a1, a2, self.goals[a1]))
                
                if shared_goal_collisions:
                    # For shared goals, use a simpler strategy: 
                    # Make one agent "give up" temporarily by shortening their path
                    # This allows the other agent to reach the goal first
                    # The animation/update_goal will then give the waiting agent a new goal
                    
                    print(f"Attempting shared goal resolution:")
                    for a1, a2, goal in shared_goal_collisions:
                        print(f"  Agents {a1} and {a2} both targeting {goal}")
                        
                        # Determine which agent is closer to the goal
                        path1 = self.paths[a1]
                        path2 = self.paths[a2]
                        
                        # Find how far each agent is from reaching the goal
                        dist1 = float('inf')
                        dist2 = float('inf')
                        goal_index1 = -1
                        goal_index2 = -1
                        
                        for i, loc in enumerate(path1):
                            if loc == goal:
                                dist1 = i
                                goal_index1 = i
                                break
                        
                        for i, loc in enumerate(path2):
                            if loc == goal:
                                dist2 = i
                                goal_index2 = i
                                break
                        
                        # Priority agent is the one closer to the goal (or lower index if tied)
                        if dist1 <= dist2:
                            priority_agent = a1
                            delayed_agent = a2
                            delayed_goal_index = goal_index2
                        else:
                            priority_agent = a2
                            delayed_agent = a1
                            delayed_goal_index = goal_index1
                        
                        print(f"  Priority agent: {priority_agent}, Delayed agent: {delayed_agent}")
                        
                        # Truncate the delayed agent's path to stop before the goal
                        delayed_path = self.paths[delayed_agent]
                        
                        if delayed_goal_index > 0:
                            # Stop before reaching the goal
                            wait_location = delayed_path[delayed_goal_index - 1]
                            wait_time = max(5, dist1 + 5)  # Wait for priority agent to reach and leave
                            new_path = delayed_path[:delayed_goal_index] + [wait_location] * wait_time
                            
                            self.paths[delayed_agent] = new_path
                            print(f"  Agent {delayed_agent} will wait at {wait_location}")
                            
                            # Mark delayed agent for future updates
                            self.mark_agent_for_updates(delayed_agent)
                        else:
                            # If we can't find goal in path or it's the first location,
                            # just make them wait at current location
                            if len(delayed_path) > 0:
                                current_loc = delayed_path[0] if len(delayed_path) == 1 else delayed_path[-1]
                                self.paths[delayed_agent] = [current_loc] * 10
                                self.mark_agent_for_updates(delayed_agent)
                    
                    # Clear collision history and try one more time
                    collision_history = []
                    collisions = detect_collisions(self.paths)
                    if collisions != []:
                        # Still colliding after shared goal resolution - try again with collision resolution
                        print(f"Collision remains after shared goal handling, continuing to resolve...")
                        # Don't raise error, let it try regular collision resolution
                        collision_history = []  # Reset to give it more attempts
                    else:
                        # If no more collisions, continue to next iteration
                        continue
                else:
                    print(f"Unable to resolve collision - this may indicate an unsolvable instance")
                    print(f"Continuing with current paths (may have remaining collisions)")
                    # Don't raise error during animation - just break and continue
                    break  # Exit collision resolution loop
            
            # finds a subset of agents to recalcuate paths for
            # identifiying the neighbourhood
            neighbourhood = self.find_neighbourhood( collisions, 3 )
            
            # Special handling for shared goals: if agents in neighbourhood share goals,
            # only replan one agent at a time to avoid both being pushed to later timesteps
            # Also check if we should apply shared goal resolution early
            if len(neighbourhood) > 1:
                # Check if any agents share goals
                neighbourhood_goals = [self.goals[a] for a in neighbourhood]
                has_shared_goals = len(neighbourhood_goals) != len(set(neighbourhood_goals))
                
                if has_shared_goals:
                    # Apply shared goal resolution if we've tried a few times
                    if iteration_count >= 5:  # After a few regular attempts, try shared goal resolution
                        print(f"Detected shared goal collision early (iteration {iteration_count})")
                        # Manually trigger shared goal resolution
                        shared_goal_collisions = []
                        for collision in collisions:
                            a1, a2 = collision['a1'], collision['a2']
                            if self.goals[a1] == self.goals[a2]:
                                shared_goal_collisions.append((a1, a2, self.goals[a1]))
                        
                        if shared_goal_collisions:
                            # Apply the same shared goal resolution logic
                            for a1, a2, goal in shared_goal_collisions:
                                # Determine priority based on distance to goal
                                path1 = self.paths[a1]
                                path2 = self.paths[a2]
                                dist1 = next((i for i, loc in enumerate(path1) if loc == goal), float('inf'))
                                dist2 = next((i for i, loc in enumerate(path2) if loc == goal), float('inf'))
                                
                                if dist1 <= dist2:
                                    delayed_agent = a2
                                    delayed_goal_index = dist2 if dist2 != float('inf') else len(path2)
                                else:
                                    delayed_agent = a1
                                    delayed_goal_index = dist1 if dist1 != float('inf') else len(path1)
                                
                                delayed_path = self.paths[delayed_agent]
                                if delayed_goal_index > 0 and delayed_goal_index <= len(delayed_path):
                                    wait_location = delayed_path[min(delayed_goal_index - 1, len(delayed_path) - 1)]
                                    new_path = delayed_path[:delayed_goal_index] + [wait_location] * 10
                                    self.paths[delayed_agent] = new_path
                                    self.mark_agent_for_updates(delayed_agent)
                            
                            # Reset collision history and continue
                            collision_history = []
                            collisions = detect_collisions(self.paths)
                            if collisions == []:
                                continue
                    
                    # Only replan the higher-indexed agent to preserve priority
                    neighbourhood = [max(neighbourhood)]

            print( f"testing output: \n\ncollisions: {collisions}\nneighbourhood:{neighbourhood}" )

            self.recalculate_paths_for_neighbourhood( neighbourhood, collisions )

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
