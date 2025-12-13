# Dynamic Multi-Agent Pathfinding for Sorting and Delivery

Multiple agents deliver items to destination bins, reverse of Amazon Warehouse case. 

# Running

We benchmarked our algorithms on a subset of the instances in the `/instances` folder. To run those instances, run as follows:
`python run_sorting.py --instance "path-to-instance" --solver CBS -- search_type [1 for WA*, 2 for focal] --weight [>=1]`
For instance, to run the 16 agent case and output to a file, run the following. 
`python run_sorting.py --instance "instances/SortingInstances/large_16agents.txt" --solver CBS --search_type 1 --weight 1.5 >> 16.txt`
