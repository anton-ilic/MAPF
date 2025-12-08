import time as timer
import heapq
import random
from single_agent_planner import ( 
    a_star,
    get_sum_of_cost,
    get_timestep_for_location,
    move as move_goal
)
from resolvingSolver import (
    ResolvingSolver,
    detect_collisions,
    paths_violate_constraint,
    split_path,
    extend_path
)

SHARED_COLLISION_MULT = 100


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if len( collision[ "loc" ] ) == 1:
        return [
            {
                "agent": collision[ "a1" ],
                "loc": collision[ "loc" ],
                "timestep": collision[ "timestep" ]
            },
            {
                "agent": collision[ "a2" ],
                "loc": collision[ "loc" ],
                "timestep": collision[ "timestep" ]
            }
        ]
    elif len( collision[ "loc" ] ) == 2:
        reverse_loc = collision[ "loc" ].copy()
        reverse_loc.reverse()
        return [
            {
                "agent": collision[ "a1" ],
                "loc": reverse_loc,
                "timestep": collision[ "timestep" ]
            },
            {
                "agent": collision[ "a2" ],
                "loc": collision[ "loc" ].copy(),
                "timestep": collision[ "timestep" ]
            }
        ]
    else:
        raise BaseException( "unknown number of locations in collision: "
              f"{len( collision['loc'] )}, {collision['loc']}" )


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agent = "a1" if random.randint( 0, 1 ) == 1 else "a2"

    if len( collision[ "loc" ] ) == 1:
        return [
            {
                "agent": collision[ agent ],
                "loc": collision[ "loc" ],
                "timestep": collision[ "timestep" ]
            },
            {
                "agent": collision[ agent ],
                "loc": collision[ "loc" ],
                "timestep": collision[ "timestep" ],
                "positive": True
            }
        ]
    elif len( collision[ "loc" ] ) == 2:
        reverse_loc = collision[ "loc" ].copy()
        if agent == "a1":
            # only need to reverse the location list for agent 1
            # leave as is for agent 2
            reverse_loc.reverse()
        return [
            {
                "agent": collision[ agent ],
                "loc": reverse_loc,
                "timestep": collision[ "timestep" ]
            },
            {
                "agent": collision[ agent ],
                "loc": reverse_loc,
                "timestep": collision[ "timestep" ],
                "positive": True
            }
        ]
    else:
        raise BaseException( "unknown number of locations in collision: "
              f"{len( collision['loc'] )}, {collision['loc']}" )


class CBSSolver(ResolvingSolver):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals)

    def get_goal( self, agent, node ):

        # returns the goal for the agent, may be its overal goal or its node specific goal
        if agent in node[ 'non-goal' ]:
            return node[ 'non-goal' ][ agent ]
        else:
            return self.goals[ agent ]

    def count_overlapping_goals( self, node ):
        goals_found = []
        overlapping = 0
        
        # loops through each agent
        for agent in range( self.num_of_agents ):

            # finds its goal
            agent_goal = self.get_goal( agent, node )

            # either adds it to goals found or increments the overlap counter
            if agent_goal in goals_found:
                overlapping += 1
            else:
                goals_found.append( agent_goal )

        return overlapping
    
    def create_node( self, constraints, paths, non_goal, updated_agents ):
        # creates a new node with the parent constraints and the new constraint
        new_node = {
            "cost": 0,
            "constraints": constraints,
            "paths": paths,
            "collisions": [],
            "non-goal": non_goal,
        }

        skipped_path = False

        #calculate new paths with new constraints
        for agent in updated_agents:
            # calculates the path for the agent
            path = a_star(
                self.my_map,
                self.starts[agent],
                self.get_goal(agent, new_node),
                self.heuristics[agent],
                agent,
                new_node[ "constraints" ]
            )
                    
            # checks if the path was calculated successfully
            if path is not None:
                new_node[ "paths" ][ agent ] = path
            else:
                skipped_path = True

            if skipped_path:
                # if a new path was not calculated for every agent, 
                # node is invalid, don't return
                return None

            # calculates cost and collisions on new path
            new_node[ "cost" ] = (
                get_sum_of_cost( new_node[ "paths" ] ) +
                ( SHARED_COLLISION_MULT * self.count_overlapping_goals( new_node ) )
            )
            new_node[ "collisions" ] = detect_collisions( new_node[ "paths" ] )

            return new_node

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node
    
    def generate_nodes( self, collision, parent_node, disjoint=True ):
        a1 = collision[ 'a1' ]
        a2 = collision[ 'a2' ]
        
        constr_split = disjoint_splitting( collision )
        
        for constr in constr_split:
            new_constraints = parent_node[ "constraints" ].copy()

            new_constraints.append( constr )

            # generates a list of all agents who need to have thier paths recalcuated
            updated_agents = [ constr[ "agent" ] ]
            if "positive" in constr and constr[ "positive" ]:
                for agent in paths_violate_constraint( constr, parent_node[ "paths" ] ):
                    updated_agents.append( agent )

            new_node = self.create_node(
                new_constraints,
                parent_node[ "paths" ].copy(),
                parent_node[ "non-goal" ].copy(),
                updated_agents
            )

            if new_node is None:
                continue

            self.push_node( new_node )

    # used to handle a collision where both agents have the same goal and
    # the collision occurs at that goal.
    # in this case, its impossible for both agents to reach thier goals so
    # we simple drop the end of the path of one agent and wait for the other to move
    # to recalcuate this path
    def handleSharedGoalCollision( self, collision, node ):
        location = collision[ "loc" ][0]

        a1 = collision[ 'a1' ]
        a2 = collision[ 'a2' ]

        a1timestep = get_timestep_for_location( node[ 'paths' ][ a1 ], location )
        a2timestep = get_timestep_for_location( node[ 'paths' ][ a2 ], location )

        if a1timestep > a2timestep:
            delayedAgent = a1
            priorityAgent = a2
        else:
            delayedAgent = a2
            priorityAgent = a1


        if self.goals[ delayedAgent ] != self.goals[ priorityAgent ] or (
            delayedAgent in node[ 'non-goal' ] or priorityAgent in node[ 'non-goal' ]
        ):
            raise RuntimeError( "handleSharedGoalCollision called on non shared goal collision" )
        
        new_node = {
            "cost": 0,
            "constraints": node[ "constraints" ].copy(),
            "paths": node[ "paths" ].copy(),
            "collisions": [],
            "non-goal": node[ 'non-goal' ].copy()
        }

        # marks the skipped agent to be updated at the next cycle
        new_node[ 'non-goal' ].append( delayedAgent )

        # calculates cost and collisions on new path
        new_node[ "cost" ] = get_sum_of_cost( new_node[ "paths" ] )
        new_node[ "collisions" ] = detect_collisions( new_node[ "paths" ] )

        self.push_node( new_node )




    def resolve_collisions(self, timestep=0):
        """ Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions

        disjoint    - use disjoint splitting or not
        """

        self.num_of_generated = 0
        self.num_of_expanded = 0

        self.open_list = []

        releventPaths = []

        # copies starts to restore later
        oldStarts = self.starts.copy()
        self.starts = []

        for path in self.paths:
            # splits the path and adds it to the relevant potions
            releventPaths.append( split_path( path, timestep ) )
            # writes the new start location
            self.starts.append( releventPaths[-1][0] )

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': releventPaths.copy(),
                'collisions': [],
                'non-goal': {} }
        
        """for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)"""

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        best_node = None
        while self.open_list != []:
            # print( "handling nodes" )
            # handle node
            node = self.pop_node()

            if node[ "collisions" ] == []:
                best_node = node
                print( "found best!!" )
                break

            # print( f"node has {len( node[ 'collisions' ] )} collisions" )
            # print( f"handling collision {node[ 'collisions' ][ 0 ]} with timestep {timestep}" )
            # print( f"paths: {node[ "paths" ]}")
            # print( f"goals: {self.goals}" )

            # generates a node for the first collision
            collision = node[ "collisions" ][ 0 ]

            print( f"handling collision {collision}" )

            agent1goal = self.goals[ collision[ 'a1' ] ]
            agent2goal = self.goals[ collision[ 'a2' ] ]
            collLoc = collision[ 'loc' ][0] if len(collision[ 'loc' ]) == 1 else None

            # print( f"a1Goal: {agent1goal}, a2Goal: {agent2goal}, collLoc: {collLoc}")

            if ( collLoc is not None and agent1goal == agent2goal and
                 not ( collision[ 'a1' ] in node[ 'non-goal' ]  or 
                       collision[ 'a2' ] in node[ 'non-goal' ] ) ):
                print( "handling shared goal collision" )
                self.handleSharedGoalCollision( collision, node )
            else:
                self.generate_nodes( collision, node )

        # adds the found paths onto the existing paths
        for i, path in enumerate( best_node[ "paths" ] ):
            self.paths[i] = extend_path( self.paths[i], timestep, path )

        # marks the non-goal agents as pending
        self.pending_agents = list( best_node[ 'non-goal' ].keys() )

        # restores the previous start values
        self.starts = oldStarts.copy()
