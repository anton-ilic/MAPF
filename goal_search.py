

def find_independant_goals( starts, goals, h_values ):
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
        pass

    goal_shifts = {}

    for agent, goal in enumerate( new_goals ):
        if goal != goals[ agent ]:
            goal_shifts[ agent ] = goal

    return goal_shifts

    