from single_agent_planner import move

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