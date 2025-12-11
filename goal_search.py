from single_agent_planner import (
    move,
    is_blocked
)
import heapq

def calc_goal_node_heuristic( node ):
    return node[ 'heuristic_dist' ] + node[ 'start_dist' ]

def find_unqiue_location_for_goal( start, goals, h_values, agent_num, map ):
    # creates an open and closed list for 
    open_list = []
    closed_list = []

    # gets the heuristic value for the start location
    start_heuristic = h_values[ start ]

    # creates a node for finding a goal
    root_node = {
        "location": goals[ agent_num ],
        "heuristic_dist": h_values[ goals[ agent_num ] ],
        "start_dist": start_heuristic - h_values[ goals[ agent_num ] ]
    }

    # adds the node to the open list
    heapq.heappush( open_list, ( calc_goal_node_heuristic( root_node ), root_node ) )
    closed_list.append( root_node[ 'location' ] )

    best_goal = None

    while open_list != []:
        
        _, node = heapq.heappop( open_list )

        # checks if this goal is unqiue
        if node[ 'location' ] not in goals:
            # found unique goal
            best_goal = node[ 'location' ]
            break

        for dir in range( 4 ):
            new_location = move( node[ 'location' ], dir )

            if is_blocked( new_location, map ):
                # new location not on map, skip
                continue

            if new_location in closed_list:
                # new location already checked
                continue

            new_node = {
                'location': new_location,
                'heuristic_dist': h_values[ new_location ],
                'start_dist': start_heuristic - h_values[ new_location ]
            }

            # adds the node to the open list
            heapq.heappush( open_list, ( calc_goal_node_heuristic( new_node ), new_node ) )
            closed_list.append( new_node[ 'location' ] )

    return best_goal
        



def find_independant_goals( starts, goals, h_values, map ):
    start_heuristics = []

    # loops through each agent and finds its heuristic value for the start
    for i, start in enumerate( starts ):
        start_heuristics.append( {
            "agent": i,
            "heuristic": h_values[i][start]
        } )

    def sort_func( val ):
        return val[ "heuristic" ]
    
    # gets a list of agents sorted by how close they are to thier goals
    start_heuristics.sort( key=sort_func )
    agent_order = ( val["agent"] for val in start_heuristics )

    new_goals = goals.copy()

    # loops through all the agents in order of how close they are to the goal
    for agent in agent_order:
        other_goals = new_goals.copy()
        other_goals.pop( agent )

        new_goal = find_unqiue_location_for_goal( 
            starts[ agent ],
            other_goals,
            h_values[ agent ],
            agent,
            map
        )

        if new_goal is None:
            return None
        
        new_goals[ agent ] = new_goal
        

    goal_shifts = {}

    for agent, goal in enumerate( new_goals ):
        if goal != goals[ agent ]:
            goal_shifts[ agent ] = goal

    return goal_shifts

    