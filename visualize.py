#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle, Polygon
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from mpaf_solver import MAPFSolver

Colors = ['green', 'blue', 'orange']

# run duration in seconds
DURATION = 120


class Animation:
    def __init__(self, my_map, starts, dropoffs, solver:MAPFSolver,
                 pickup=None, sequence=[]):
        self.my_map = np.flip(np.transpose(my_map), 1)

        # stores the start and dropoff points
        self.starts = []
        for start in starts:
            self.starts.append(self.convertToAnimationSpace(start))
        self.dropoffs = []
        for dropoff in dropoffs:
            self.dropoffs.append(self.convertToAnimationSpace(dropoff))

        # stores the pickup point
        if pickup is not None:
            self.pickup = self.convertToAnimationSpace( pickup )
        else:
            self.pickup = None

        self.sequence = sequence

        # calculates the paths needed for the current iteration
        self.solver = solver
        paths = solver.find_solution()

        self.currentPackage = [None] * len(starts)

        self.installPaths( paths )

        aspect = len(self.my_map) / len(self.my_map[0])

        # initializes the figure
        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch

        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

        # create agents:
        self.T = 0
        
        # adds the pick up point to the animation
        if pickup is not None:
            self.patches.append( Polygon([
                                            (self.pickup[0] - 0.25, self.pickup[1] - 0.05),
                                            (self.pickup[0] - 0.05, self.pickup[1] - 0.05),
                                            (self.pickup[0] - 0.05, self.pickup[1] - 0.25),
                                            (self.pickup[0] + 0.05, self.pickup[1] - 0.25),
                                            (self.pickup[0] + 0.05, self.pickup[1] - 0.05),
                                            (self.pickup[0] + 0.25, self.pickup[1] - 0.05),
                                            (self.pickup[0] + 0.25, self.pickup[1] + 0.05),
                                            (self.pickup[0] + 0.05, self.pickup[1] + 0.05),
                                            (self.pickup[0] + 0.05, self.pickup[1] + 0.25),
                                            (self.pickup[0] - 0.05, self.pickup[1] + 0.25),
                                            (self.pickup[0] - 0.05, self.pickup[1] + 0.05),
                                            (self.pickup[0] - 0.25, self.pickup[1] + 0.05)
                                         ], 
                                         closed=True, 
                                         edgecolor='black', 
                                         facecolor='red',
                                         alpha=0.5))

        # draw dropoffs first
        for i, dropoff in enumerate(self.dropoffs):
            self.patches.append(Rectangle((dropoff[0] - 0.25, dropoff[1] - 0.25), 0.5, 0.5,
                                          facecolor=Colors[i % len(Colors)],
                                          edgecolor='black', alpha=0.5))
                                          
            # adds a text label for which dropoff point this is
            dropoffText = self.ax.text( dropoff[0], dropoff[1], str(i) )
            dropoffText.set_horizontalalignment( 'center' )
            dropoffText.set_verticalalignment( 'center' )
            self.artists.append(dropoffText)
        # creates the agents
        for i in range(len(self.paths)):
            name = str(i)
            self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                    edgecolor='black')
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(paths[i]) - 1)
            self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=DURATION * 10,
                                                 interval=100,
                                                 blit=True)
        
    def convertToAnimationSpace(self, pos):
        return (pos[1], len(self.my_map[0]) - 1 - pos[0])
    
    def convertToSearchSpace(self, pos):
        return (len(self.my_map[0]) - 1 - pos[1], pos[0])
    
    def installPaths(self, paths):
         # stores the paths in the animation class
        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    # converts paths to animation space
                    self.paths[-1].append(self.convertToAnimationSpace(loc))

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"}, writer="html")

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        for k in range(len(self.paths)):
            pos = self.get_state(t / 10, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # checking for even timesteps if the agents goal should be updated
        if t % 10 == 0:
            step = int(t / 10)
            if self.pickup is not None:
                for a in range(len(self.paths)):
                    if self.currentPackage[a] is None:
                        if self.agents[a].center == self.pickup:

                            # agent should pick up package and go to its dropoff point
                            if len( self.sequence ) > 0:
                                self.currentPackage[a] = self.sequence.pop( 0 )
                                newGoal = self.dropoffs[ self.currentPackage[ a ] ]
                            else:
                                newGoal = self.starts[a]
                            newGoal = self.convertToSearchSpace( newGoal )
                            newPaths = self.solver.update_goal(a, newGoal, step + 1)
                            self.installPaths(newPaths)

                    else:
                        if self.agents[a].center == self.dropoffs[ self.currentPackage[ a ] ]:

                            # agent should drop off package and go to pickup point
                            self.currentPackage[a] = None
                            if len(self.sequence) > 0:
                                newGoal = self.pickup
                            else:
                                newGoal = self.starts[a]
                            newGoal = self.convertToSearchSpace( newGoal )
                            newPaths = self.solver.update_goal(a, newGoal, step + 1)
                            self.installPaths(newPaths)


        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos
