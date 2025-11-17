import heapq

def move(loc, dir):
    # added move that doesn't move ( move( loc, 4 ) )
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()

    for constr in constraints:
        # also includes positive constraints for other agents to ensure they are avoided
        if ( constr[ 'agent' ] == agent or 
             "positive" in constr and constr[ "positive" ] ):
            
            time_step = constr[ 'timestep' ]

            new_constr = constr.copy()

            # checks if this is a positive constraint for another agent
            if ( "positive" in new_constr and new_constr[ "positive" ] and
                 new_constr[ "agent" ] != agent ):
                
                # converts this to a negative constraint
                del new_constr[ "positive" ]
                new_constr[ "agent" ] = agent
                if len( constr[ "loc" ] ) == 2:

                    # if this is an edge constraint, reverses it
                    new_constr[ "loc" ] = new_constr[ "loc" ].copy()
                    new_constr[ "loc" ].reverse()


            if time_step in constraint_table:
                # if an entry already exists for this time step add to its list
                constraint_table[ time_step ].append( new_constr )
            else:
                # if not, initializes this time_step in the table with a list
                constraint_table[ time_step ] = [ new_constr ]

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    elif time < len(path) + 2:
        return path[-1]  # wait at the goal location
    else:
        None


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


# updates the constraint table to move any goal constraints
# from this timestep to the next
def propogate_constraints( next_time, constraint_table ):
    if next_time in constraint_table:
        #loops through all constraints for this time step to propogate 
        # any goal constraints
        for constr in constraint_table[ next_time ]:
            if 'goal' in constr and constr[ 'goal' ]:
                # if this is a goal constraint, copies it to the next timestep
                if next_time + 1 in constraint_table:
                    if constr not in constraint_table[ next_time + 1 ]:
                        constraint_table[ next_time + 1 ].append( constr )
                else:
                    constraint_table[ next_time + 1 ] = [ constr ]


def is_constrained( curr_loc, next_loc, next_time, constraint_table, agent ):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    if next_time in constraint_table:  
        for constr in constraint_table[ next_time ]:
            if "positive" in constr and constr[ "positive" ]:
                # handle positive constraint
                if len( constr[ 'loc' ] ) == 1:
                    # movement constrained so next vertex must be loc[ 0 ]
                    # all other vertexs constrained
                    if constr[ 'loc' ][ 0 ] != next_loc:
                        return True
                elif len( constr[ 'loc' ] ) == 2:
                    # movement constrained so next vertex must move from loc[ 0 ] to loc[ 1 ]
                    # requires correct starting and ending locations
                    if ( constr[ 'loc' ][ 0 ] != curr_loc or
                        constr[ 'loc' ][ 1 ] != next_loc ):
                        return True
            else:
                # handle negative constraint
                if len( constr[ 'loc' ] ) == 1:
                    if constr[ 'loc' ][ 0 ] == next_loc:
                        return True
                elif len( constr[ 'loc' ] ) == 2:
                    # checks both current and next location for the edge constraint
                    if ( constr[ 'loc' ][ 0 ] == curr_loc and
                        constr[ 'loc' ][ 1 ] == next_loc ):
                        return True
    return False

# finds the next location this agent must be at to satisfy a positive constraint
# returns a tuple of the location the agent must be at for the constraint and the
# timestep they must be at that location for
# returns None if there are no positive constraints after current_time
def find_next_positive_constraint( current_time, agent, constraint_table:dict ):

    # stores a tuple of the next positive constraint and the time it happens at
    next_constraint = None
    for ( time, constraint_list ) in constraint_table.items():
        # skip any times less than or equal to the current time
        # those constraints have already passed or been checked elsewhere
        if time > current_time:
            for constr in constraint_list:
                # checks if this is a positive constraint for this agent
                if ( "positive" in constr and constr[ "positive" ] and
                     constr[ "agent" ] == agent ):
                    # if so, check if its time is earlier than the current next constraint
                    if next_constraint is not None:
                        if next_constraint[0] > time:
                            next_constraint = ( time, constr )
                    else:
                        next_constraint = ( time, constr )

    if next_constraint is not None:
        if len( next_constraint[1][ 'loc' ] ) == 1:
            # vertex constraint, must be at vertex at timestep
            return ( next_constraint[0], next_constraint[1][ 'loc' ][0] )
        elif len( next_constraint[1][ 'loc' ] ) == 2:
            # edge constraint, must be at first vertex of edge in timestep before constraint
            # eg.
            # constraint on vertex [(2, 3), (3, 3)] at time 3
            # agent must be at (2, 3) at time 2 to be able to move to (3, 3) at time 3
            return ( next_constraint[0] - 1, next_constraint[1][ 'loc' ][0] )

    return None

# checks if this agent can make its positive constraints from this location
# returns False if this agent cannot make it from its current location to its
# next postive constraint assume that there are no obsticles or other constraints
def check_can_make_constraints( loc, current_time, agent, constraint_table:dict ):
    next_constraint = find_next_positive_constraint( current_time, agent, constraint_table )

    if next_constraint is not None:
        constr_loc = next_constraint[1]
        # the number of steps left before the constraint
        steps = next_constraint[0] - current_time

        # calculates taxicab distance to goal
        # this is also the number of steps required to get to the goal
        constr_dist = abs( loc[0] - constr_loc[0] ) + abs( loc[1] - constr_loc[1] )
        
        # if distance is less than available steps, all good, otherwise, return false
        return constr_dist <= steps

    # if no constraints, no problems, return true
    return True


# checks the constraint table for any future vertex constraints on being in this location
# returns True if there is a constraint on loc in the future
def check_future_constraints( loc, current_time, constraint_table:dict ):
    for ( time, constraint_list ) in constraint_table.items():
        if time < current_time:
            continue

        for constr in constraint_list:
            if len( constr[ 'loc' ] ) == 1:
                if constr[ 'loc' ][ 0 ] == loc:
                    # future constraint on this location, return true
                    return True
            elif len( constr[ 'loc' ] ) == 2:
                if ( constr[ 'loc' ][ 0 ] == loc and
                     constr[ 'loc' ][ 1 ] == loc ):
                    # future constraint on moving to this location from this location
                    # return true
                    return True
    return False

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    # populating the constraints table
    constraint_table = build_constraint_table( constraints, agent )

    # counts the number of constraint in the constraint table
    # also gets the largest timestep of a constraint affecting this agent
    constraint_count = 0
    last_constr_timestep = 0
    for constr_list in constraint_table.values():
        constraint_count = constraint_count + len( constr_list )
        for constr in constr_list:
            if constr[ "timestep" ] > last_constr_timestep:
                last_constr_timestep = constr[ "timestep" ]

    # sets the maximum search tree depth the heuristic value of the starting location + the number of constraints
    # since the heuristic is the dikstra search distance from this cell to the goal,
    # with no constraints it will take this h steps to get to the goal
    # this approach assumes that each constraint will require 1 additional timestep to handle
    # however, in the case of constraints on the goal location, it may be required to search far after the goal has been reached
    # for this reason, if the last constraint timestep is larger than the heuristic value, its used instead
    max_steps = max( h_values[ start_loc ], last_constr_timestep ) + constraint_count

    open_list = []
    # keyed using cell location and timestep
    # agents can be in the same location at different times
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0 }
    # pushes the root to the open list initally
    push_node(open_list, root)
    # starts the closed list with the root node at time 0 in it
    closed_list[(root['loc'], root['time_step'])] = root
    while len(open_list) > 0:
        # gets the next best node to check
        curr = pop_node(open_list)

        if curr[ 'time_step' ] > max_steps:
            # skips this node if its too deep
            continue

        # updates the constraint table
        propogate_constraints( curr[ 'time_step' ], constraint_table )
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if ( curr['loc'] == goal_loc and not 
             check_future_constraints( curr[ 'loc' ], curr[ 'time_step' ], 
                                       constraint_table ) ):
            # returns the path if the goal is found
            return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            if ( child_loc[ 0 ] < 0 or child_loc[ 0 ] >= len( my_map ) or
                 child_loc[ 1 ] < 0 or child_loc[ 1 ] >= len( my_map[ 0 ] ) ):
                # location is not on map
                continue

            if my_map[child_loc[0]][child_loc[1]]:
                # if new location is not in the map, try another
                continue

            if is_constrained(curr[ 'loc' ], child_loc, curr[ 'time_step' ] + 1, 
                              constraint_table, agent ):
                # new location is constrained, cannot visit
                continue

            if not check_can_make_constraints( child_loc, curr[ "time_step" ] + 1,
                                               agent, constraint_table ):
                # impossible to satisfy positive constraints from this location, skip
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr[ 'time_step' ] + 1 }
            if (child['loc'], child['time_step'] ) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
