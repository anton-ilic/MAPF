from single_agent_planner import (
    move,
    build_constraint_table
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
    for i, constraint_list in enumerate( constraints ):
        constraint_tables.append( build_constraint_table( constraint_list, agents[ i ] ) )


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

    root = {
        'loc': start_locs.copy(),
        'g_val': 0,
        'h_val': total_h_value,
        'parent': None,
        'time_step': 0
    }

    # pushes the root node into the open list
    heapq.heappush(open_list, 
                   (root['g_val'] + weight * root['h_val'], root['h_val'], root['loc'], root))
    
    # starts the closed list with the root node in it



    