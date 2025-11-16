#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from visualize import Animation
from individual import IndivSolver

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, pickup, dropoffs):
    print('Start locations')
    print_locations(my_map, starts)
    print('pickup location')
    print_locations(my_map, [pickup])
    print('dropoff locations')
    print_locations(my_map, dropoffs)



def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: s to check for sorting instance
    line = f.readline()
    if line != "s\n":
        raise BaseException( filename + " is not a sorting instance." )
    # second line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start positions (goals determined later)
    starts = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
    # #dropoff points
    line = f.readline()
    num_dropoffs = int(line)
    # reads all of the dropoff locations
    dropoffs = []
    for a in range(num_dropoffs):
        line = f.readline()
        dx, dy = [int(x) for x in line.split(' ')]
        dropoffs.append( (dx, dy) )
    # reads the pickup location
    line = f.readline()
    px, py = [int(x) for x in line.split(' ')]
    pickup = (px, py)
    # reads the pickup sequence
    line = f.readline()
    sequence = [int(x) for x in line.split(' ')]
    #done with file
    f.close()
    return my_map, starts, pickup, dropoffs, sequence


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()


    #result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, pickup, dropoffs, sequence = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, pickup, dropoffs)

        """if args.solver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")"""
        
        if args.solver == "IND":
            print("***Run Independent***")
            solver = IndivSolver( my_map, starts, [pickup] * len(starts) )
        else:
            raise RuntimeError( "Unknown solver!!!" )
        


        """cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))"""


        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, dropoffs, solver, pickup=pickup )
        # animation.save("output.mp4", 1.0)
        animation.show()
    #result_file.close()
