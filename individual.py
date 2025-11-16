from mpaf_solver import MAPFSolver
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

class IndivSolver( MAPFSolver ):

    def __init__( self, my_map, starts, goals ):
        super().__init__( my_map, starts, goals )

        self.paths = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        self.start_time = timer.time()
        self.paths = []

        longestPath = 0

        # finds paths for each agent (disregarding other agents)
        for agent in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[agent], self.goals[agent], 
                          self.heuristics[agent], agent, [])
            if path is None:
                raise BaseException('No solutions')
            
            if len(path) > longestPath:
                longestPath = len(path)
            
            # lengthens the path by agent (delays later agents to reduce a few collisions)
            for i in range(agent):
                path.insert(0, path[0])
            self.paths.append(path)
        
        self.print_results( self.paths )
        return self.paths
    
    def update_goal(self, agent, goal, timestep):
        self.start_time = timer.time()
        # updates the agents goal
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

        return self.paths