from single_agent_planner import (
    move,
    build_constraint_table,
    is_blocked,
    is_constrained,
    check_can_make_constraints,
    propogate_constraints,
    check_future_constraints
)
import heapq

def multi_move( locations, dir ):
    new_locations = locations.copy()

    i = 0

    while i < len( new_locations ):
        # moves the current agent with the 5 lowest numbers of dir
        new_locations[ i ] = move( locations[ i ], dir % 5)

        # divides dir by 5 to get the next dir
        dir = int( dir / 5 )

        i += 1

    return new_locations

def get_total_heuristic( locs, h_values ):
    total_heuristic = 0

    # sums the heuristic values for each agent based on its location
    for i, loc in enumerate( locs ):
        total_heuristic += h_values[ i ][ loc ]

    # returns the total heuristic value for this set of locations
    return total_heuristic

def other_agent_occupying( loc, list, index ):
    for i, list_loc in enumerate( list ):
        if i != index and list_loc == loc:
            return True
        
    return False

def traded_locations( locs, prev_locs, index ):
    curr_loc = locs[ index ]
    for i, prev_loc in enumerate( prev_locs ):
        if i != index and prev_loc == curr_loc and locs[i] == prev_locs[index]:
            # agent i and agent index traded places last timestep, this is invalid
            return True
        
    return False

def is_valid_position( prev_locs, locs, map, constraint_tables, agents, timestep ):

    # loops through each agents, location
    for i, loc in enumerate( locs ):
        # checks if this location is blocked on the map
        if is_blocked( loc, map ):
            # location is block, therefore invalid
            return False
        
        # check if another agent is occupying this location
        if other_agent_occupying( loc, locs, i ):
            # overlaps with another agent, therefore invalid
            return False
        
        # check if another agent occupied this location last turn
        if traded_locations( locs, prev_locs, i ):
            # trades places with another agent, therefore invalid
            return False
        
        # check if this location is limited by any constraints
        if ( is_constrained( 
                prev_locs[i],
                loc,
                timestep,
                constraint_tables[i],
                agents[i] )
        ):
            # this location violates a constraint, is therefore invalid
            return False
        
        if not (
            check_can_make_constraints(
                loc,
                timestep,
                agents[i],
                constraint_tables[i]
            )
        ):
            # this location will make it impossible for this agent to satisfy its positive constraints
            # it is therefore invalid
            return False
    

    return True

def find_next_best_move( node, h_values, map, constraint_tables, agents ):
    # counts how many agents there are (used to determine move count)
    agent_count = len( h_values )

    # get the location to move from
    locs = node[ 'loc' ]
    best_move = None
    best_heuristic = None

    # loops for each 5^agent_count possible sets of moves
    for dir in range( 5 ** agent_count ):
        if dir in node[ 'completed_moves' ]:
            # this move has already been expanded, skip this
            continue

        # gets the new set of locations for this move
        new_locs = multi_move( locs, dir )

        if not is_valid_position( 
            locs, 
            new_locs, 
            map, 
            constraint_tables, 
            agents, 
            node[ 'time_step' ] + 1 
        ):
            # this move is invalid, not point in giving it a heuristic value
            continue

        # gets the sum of the heuristic values for this location
        total_h = get_total_heuristic( new_locs, h_values )

        # if there is no best move so far or if this move is better than the current best
        if best_heuristic is None or best_heuristic > total_h:
            # update the best move to this move
            best_heuristic = total_h
            best_move = dir

    if best_move is None:
        return None
    else:
        return (best_move, best_heuristic)


    

def generate_node( locs, timestep, parent, h_values, map, agents, constraint_tables ):
    node = {
        'loc': locs.copy(),
        'g_val': timestep * len( locs ),
        'h_val': 0,
        'parent': parent,
        'time_step': timestep,
        'completed_moves': [],
        'best_move': 0
    }

    next_best = find_next_best_move(
        node,
        h_values,
        map,
        constraint_tables,
        agents
    )

    if next_best is None:
        # no valid moves from this node, return no node
        return None
    
    best_move, heuristic = next_best

    # assigns the best move and heuristic to the node
    node[ 'best_move' ] = best_move
    node[ 'h_val' ] = heuristic
    node[ 'completed_moves' ] = [ best_move ]

    # returns the new node
    return node

def check_for_goal( locs, goal_locs, constraint_tables, agents, timestep ):
    for i, loc in enumerate( locs ):
        if loc != goal_locs[i]:
            # this agent is not at the goal, non-goal configuration return false
            return False
        
        if check_future_constraints( loc, timestep, constraint_tables[i], agents[i] ):
            # this agents location is constrained in the future, non-goal configuration
            return False
        
    # if all agents in goal location with no constraints, goal configuration: True
    return True

def get_paths( goal_node ):
    paths = []

    # loops through each agent with a path
    for i in range( len( goal_node[ 'loc' ] ) ):

        curr_path = []
        curr_node = goal_node
        while curr_node is not None:
            # steps through each node up the tree until the root node
            curr_path.append( curr_node['loc'][i] )
            curr_node = curr_node[ 'parent' ]

        # Reverses the path to ensure its in the right order
        curr_path.reverse()
        # adds it to the path list
        paths.append( curr_path )

    return paths



def weight_multi_a_star(
        my_map,
        start_locs,
        goal_locs,
        h_values,
        agents,
        constraints,
        weight=1
):
    
    constraint_tables = []

    # builds a constraint table for each agent in the search.
    # may have different constraints for each agent
    for i in range( len( start_locs ) ):
        if i < len( constraints ):
            constraint_list = constraints[i]
            constraint_tables.append( build_constraint_table( constraint_list, agents[ i ] ) )
        else:
            constraint_tables.append( {} )


    max_constraint_count = 0
    max_last_timestep = 0

    # counts the constraints in each constraint table
    for constraint_table in constraint_tables:
        # counts the number of constraint in the constraint table
        # also gets the largest timestep of a constraint affecting this agent
        constraint_count = 0
        last_constr_timestep = 0
        for constr_list in constraint_table.values():
            constraint_count = constraint_count + len( constr_list )
            for constr in constr_list:
                if constr[ "timestep" ] > last_constr_timestep:
                    last_constr_timestep = constr[ "timestep" ]

        if constraint_count > max_constraint_count:
            max_constraint_count = constraint_count

        if last_constr_timestep > max_last_timestep:
            max_last_timestep = last_constr_timestep

    max_start_heuristic = 0
    total_h_value = 0

    # finds the maximum heuristic value of the start location
    for i, start_loc in enumerate( start_locs ):
        total_h_value += h_values[i][ start_loc ]
        if max_start_heuristic < h_values[i][ start_loc ]:
            max_start_heuristic = h_values[i][ start_loc ]

    # sets the maximum search tree depth the heuristic value of the starting location + the number of constraints
    # since the heuristic is the dikstra search distance from this cell to the goal,
    # with no constraints it will take this h steps to get to the goal
    # this approach assumes that each constraint will require 1 additional timestep to handle
    # however, in the case of constraints on the goal location, it may be required to search far after the goal has been reached
    # for this reason, if the last constraint timestep is larger than the heuristic value, its used instead
    max_steps = ( max( max_start_heuristic, max_last_timestep ) + max_constraint_count ) * 2

    # defines the open and closed lists for the A* search
    open_list = []
    closed_list = dict()

    root = generate_node(
        start_locs.copy(),
        0,
        None,
        h_values,
        my_map,
        agents,
        constraint_tables
    )

    if root is None:
        # no valid moves from root node, unsolvable case
        return None

    # pushes the root node into the open list
    heapq.heappush(open_list, 
                   (root['g_val'] + weight * root['h_val'], root['h_val'], root['loc'], root))
    
    # starts the closed list with the root node in it
    closed_list[ ( tuple(root[ 'loc' ]), root[ 'time_step' ] )] = root

    # updates the constraints tables
    for table in constraint_tables:
        # pushes forward the goal constraints for each agent
        propogate_constraints( 0, table )
    
    while len(open_list) > 0:
        # gets the next best node to check
        _, _, _, curr_node = heapq.heappop( open_list )

        print( f"checking node at locations: {curr_node['loc']}" )

        if curr_node[ 'time_step' ] > max_steps:
            # this node is too deep in the tree, purge it
            continue

        # updates the constraints tables
        for table in constraint_tables:
            # pushes forward the goal constraints for each agent
            propogate_constraints( curr_node[ 'time_step' ] + 1, table )

        if check_for_goal(
            curr_node[ 'loc' ],
            goal_locs,
            constraint_tables,
            agents,
            curr_node[ 'time_step' ]
        ):
            # this is a goal configuration! 
            return get_paths( curr_node )
        
        # gets the next move for this node
        next_move = curr_node[ 'best_move' ]
        child_loc = multi_move( curr_node[ 'loc' ], next_move )

        # creates a child node from this move
        child_node = generate_node(
            child_loc,
            curr_node[ 'time_step' ] + 1,
            curr_node,
            h_values,
            my_map,
            agents,
            constraint_tables
        )

        # checks if a child node was successfuly created
        if child_node is not None:
            
            # checks if this configuration is in the closed list
            if not ( tuple( child_node[ 'loc' ] ), child_node[ 'time_step' ] ) in closed_list:
                closed_list[ ( tuple( child_node[ 'loc' ] ), child_node[ 'time_step' ] ) ] = child_node

                heapq.heappush( open_list, 
                   (child_node['g_val'] + weight * child_node['h_val'], child_node['h_val'], child_node['loc'], child_node))
                
        # finds the next best move to try on the parent node
        next_best_move = find_next_best_move(
            curr_node,
            h_values,
            my_map,
            constraint_tables,
            agents
        )

        if next_best_move is not None:
            best_move, heuristic = next_best_move

            # reconfigures the current node for its next move
            curr_node[ 'best_move' ] = best_move
            curr_node[ 'h_val' ] = heuristic
            curr_node[ 'completed_moves' ].append( best_move )

            # re-adds this node to the open list
            heapq.heappush( open_list, 
                   (curr_node['g_val'] + weight * curr_node['h_val'], curr_node['h_val'], curr_node['loc'], curr_node))
            
        
    # could not find a valid set of path, return None
    return None