# Dynamic Multi-Agent Pathfinding for Sorting and Delivery

Multiple agents deliver items to destination bins, reverse of Amazon Warehouse case. 

# Quick Start

Run an instance with the default CBS solver:

```bash
python run_sorting.py --instance "instances/SortingInstances/dual_lanes.txt" --solver CBS
```

## Running our instances

### Command-Line Arguments
- `--instance`: Path to instance file(s)
- `--solver`: Solver to use (CBS, LNS)
- `--search_type`: Search algorithm type (0=Standard A*, 1=Weighted A*, 2=Focal Search)
- `--weight`: Weight for weighted A* or focal search (must be >= 1.0)
- `--save`: Save animation as result.gif (True/False)

We benchmarked our algorithms on a subset of the instances in the `/instances` folder. To run those instances, run as follows:
`python run_sorting.py --instance "path-to-instance" --solver CBS -- search_type [1 for WA*, 2 for focal] --weight [>=1]`
For instance, to run the 16 agent case and output to a file, run the following. 
`python run_sorting.py --instance "instances/SortingInstances/large_16agents.txt" --solver CBS --search_type 1 --weight 1.5 >> 16.txt`


## Instance Format

Instance files are located in `instances/SortingInstances/`. See the report for detailed format specifications and instance descriptions.

## Documentation

For detailed algorithm descriptions, implementation details, and experimental results, see the project report in `report/CMPT417 Report.pdf`.