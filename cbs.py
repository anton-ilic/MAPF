import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
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
              f"{len( collision[ "loc" ] )}, {collision[ "loc" ]}" )


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
              f"{len( collision[ "loc" ] )}, {collision[ "loc" ]}" )


class CBSSolver(MAPFSolver):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        super().__init__(my_map, starts, goals)

        self.paths = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node
    
    def generate_nodes( self, collision, parent_node, disjoint=True, timestep=0 ):
        if disjoint:
            constr_split = disjoint_splitting( collision )
        else:
            constr_split = standard_splitting(collision)

        print( f"split: {collision} into:\n{constr_split}")
        
        for constr in constr_split:
            if constr[ "timestep" ] < timestep:
                # skips this contraint as it occurs before the active timestep
                print( "skipping" )
                continue

            print( f"parent constraints: {parent_node[ 'constraints' ]}" )

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
                path = new_node[ "paths" ][ agent ].copy()

                while len( path ) < timestep + 1:
                    path.append( path[-1] )

                newStart = path[timestep]

                path = path[:timestep]

                modifed_constraints = []

                # reduces all constraint timesteps by timestep to sync
                for modConstr in new_node[ "constraints"] :
                    newConstr = modConstr.copy()

                    newConstr[ 'timestep' ] -= timestep

                    modifed_constraints.append(newConstr)

                print( f"modifedConstraints: {modifed_constraints}\nconstraints:{new_node[ 'constraints' ]}")

                newpath = a_star( self.my_map, newStart, self.goals[agent], self.heuristics[agent],
                               agent, modifed_constraints )
                
                # checks if the path was calculated successfully
                if newpath is not None:
                    path.extend( newpath )
                    
                    new_node[ "paths" ][ agent ] = path
                else:
                    skipped_path = True

            if skipped_path:
                # if a new path was not calculated for every agent, 
                # don't add this node
                print( "failed to calcuate paths for this constriant" )
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

    def resolve_collisions(self, timestep=0):
        """ Finds paths for all agents from their start locations to their goal locations

        requires a set of paths in place that may have collisions

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        self.num_of_generated = 0
        self.num_of_expanded = 0

        self.open_list = []

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': self.paths.copy(),
                'collisions': []}
        
        """for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)"""

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print( standard_splitting(collision) )


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
            print( "handling nodes" )
            # handle node
            node = self.pop_node()

            if node[ "collisions" ] == []:
                best_node = node
                print( "found best!!" )
                break

            print( f"node has {len( node[ 'collisions' ] )} collisions" )
            print( f"handling collision {node[ 'collisions' ][ 0 ]} with timestep {timestep}" )

            # generates a node for the first collision
            collision = node[ "collisions" ][ 0 ]
            self.generate_nodes( collision, node, timestep=timestep )
            

        self.print_results(best_node[ "paths"] )
        self.paths = best_node['paths'].copy()

    def find_solution(self):
        self.calculate_colliding_paths()

        self.resolve_collisions()

        return self.paths
    
    def update_goal(self, agent, goal, timestep):
        self.goals[ agent ] = goal

        # re-calcuates the heuristic to use the new goal
        self.heuristics[ agent ] = compute_heuristics( self.my_map, goal )

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
                start = agentPath[ -1 ]

        # calculates a new path
        newPath = a_star(self.my_map, start, goal, self.heuristics[ agent ],
                         agent, [] )
        
        # adds it to the current path
        agentPath.extend(newPath)

        self.paths[agent] = agentPath

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
