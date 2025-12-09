from single_agent_planner import get_sum_of_cost
import time as timer

"""
A class for any solver used for multi-agent pathfinding problems
"""
class MAPFSolver(object):

    # initializes the MAPFSOlver with a MAPF problem
    def __init__(self, my_map, starts, goals, final):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        # creates a list of whether or not these goals are final
        self.final_goals = [ final ] * self.num_of_agents

        self.start_time = 0

    # finds a solution to the MPAF problem specifed by the map,
    # starts and goals in this solver. this function should be
    # overritten for each subclass of this solver
    def find_solution(self):
        pass

    # takes a given agent and updates thier goal location to the new passed goal
    # expands all paths from the passed timestep
    # assumes new goal is only temporary unless goal marked as final
    def update_goal(self, agent, goal, timestep, final):
        pass

    # prints the results of the search
    def print_results(self, paths):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.6f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(paths)))
        print( "proposed paths: {}".format( paths ) )
