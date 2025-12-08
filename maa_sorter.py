from multiagent_a_star import (
    weight_multi_a_star
)
from single_agent_planner import (
    compute_heuristics
)
from mpaf_solver import MAPFSolver

import time as timer

class MAASolver( MAPFSolver ):

    # initializes the MAASolver with a MAPF problem
    def __init__(self, my_map, starts, goals):
        # initalizes teh MAPF solver
        super().__init__( my_map, starts, goals)

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

        self.paths = []

    def find_solution(self):
        self.start_time = timer.time()

        paths = weight_multi_a_star(
            self.my_map,
            self.starts,
            self.goals,
            self.heuristics,
            list( range( self.num_of_agents ) ),
            []
        )

        if paths is None:
            raise BaseException(
                f"Failed to solve MPAF problem with starts {self.starts} and goals {self.goals}")
        
        self.print_results( paths )

        self.paths = paths
        
        return paths