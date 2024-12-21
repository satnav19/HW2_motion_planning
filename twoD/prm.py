import time
import heapq
from collections import defaultdict
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
import math
import matplotlib.pyplot as plt
from .building_blocks import BuildingBlocks2D


class PRMController:
    def __init__(self, start, goal, bb):
        self.graph = nx.Graph()
        self.bb = bb
        self.start = start
        self.goal = goal
        self.coordinates_history = []
        self.kdtree = None
        # Feel free to add class variables as you wish

    def run_PRM(self, num_coords=100, k=5):
        """
            find a plan to get from current config to destination
            return-the found plan and None if couldn't find
        """
        path = []
        self.configs = []

        # Preprocessing
        configs = self.gen_coords(num_coords)
        configs.extend([self.start, self.goal])
        self.add_to_graph(configs, k)

        # Planning part
        path, distance = self.shortest_path()
        if path is None:
            print("No path found between start and goal configurations")
            return None

        self.coordinates_history = path
        return path

    def create_graph(self, base_number, how_many_to_add, num_searches):
        max_nodes = base_number + (num_searches - 1) * how_many_to_add
        all_configs = self.gen_coords(max_nodes)

        costs = {
            '5': [],
            '10': [],
            'log(n)': [],
            '10log(n)': [],
            'n/10': []
        }
        runtimes = {
            '5': [],
            '10': [],
            'log(n)': [],
            '10log(n)': [],
            'n/10': []
        }

        # For each value of n
        for i in range(num_searches):
            n = base_number + i * how_many_to_add
            configs = all_configs[:n]  # Use subset of pre-generated configs
            # Reset graph for new iteration
            self.graph.clear()
            self.configs = []
            # Test each k value
            k_values = {
                '5': 5,
                '10': 10,
                'log(n)': int(math.log(n)),
                '10log(n)': int(10 * math.log(n)),
                'n/10': n // 10
            }

            for k_name, k in k_values.items():
                # Reset graph for each k value
                self.graph.clear()
                self.configs = []

                # Time the graph construction and path finding
                start_time = time.time()

                # Add nodes and find path
                self.add_to_graph(configs + [self.start, self.goal], k)
                path = self.shortest_path()

                end_time = time.time()
                runtime = end_time - start_time
                # Calculate path cost if path exists
                if path is not None:
                    path_cost = self.bb.compute_path_cost(path)
                else:
                    path_cost = float('inf')

                # Store results
                costs[k_name].append(path_cost)
                runtimes[k_name].append(runtime)

        return costs, runtimes

    def gen_coords(self, n=5):
        # print("coords generated")
        """
        Generate 'n' random collision-free samples called milestones.
        n: number of collision-free configurations to generate
        """
        num = 0

        
        configs = []
        while num < n:
            config = np.random.uniform(low=-np.pi, high=np.pi, size=4)
            if self.bb.config_validity_checker(config):
                num += 1
                configs.append(config)
        return configs

    def add_to_graph(self, configs, k):
        # print("added to graph")
        """
            add new configs to the graph.
        """
        for config in configs:
            if config.tolist() not in [c.tolist() for c in self.configs]:
                self.configs.append(config)
                self.graph.add_node(tuple(config))

        self.kdtree = KDTree(self.configs)
        # For each config, find and connect to k nearest neighbors
        i = 0
        for config in configs:
            # print(f'config {i} out of len {len(configs)}')
            j = 0
            i += 1
            neighbors = self.find_nearest_neighbour(config, k)
            for neighbor in neighbors:
                # print(f'neighbor {j} out of {len(neighbors)} ')
                j += 1
                distance = self.bb.compute_distance(config, neighbor)
                if self.bb.edge_validity_checker(config, neighbor):
                    self.graph.add_edge(tuple(config), tuple(neighbor),
                                        weight=distance)

    def find_nearest_neighbour(self, config, k=5):

        # print("found neighbors")
        """
            Find the k nearest neighbours to config
        """
        if len(self.configs) == 0:
            return []

        if self.kdtree is None:
            self.kdtree = KDTree(self.configs)

        distances, indices = self.kdtree.query(
            config, k=min(k+1, len(self.configs)))

        neighbors = []
        for i, idx in enumerate(indices):
            if not np.array_equal(self.configs[idx], config):
                neighbors.append(self.configs[idx])
                if len(neighbors) == k:
                    break
        return neighbors

    def shortest_path(self):
        # print("did sp")
        """
            Find the shortest path from start to goal using Dijkstra's algorithm (you can use previous implementation from HW1)'
        """
        start_tuple = tuple(self.start)
        goal_tuple = tuple(self.goal)

        if start_tuple not in self.graph or goal_tuple not in self.graph:
            return None

        queue = [(0, start_tuple)]
        distances = {start_tuple: 0}
        previous = {start_tuple: None}
        visited = set()

        while queue:
            current_distance, current_node = heapq.heappop(queue)

            if current_node == goal_tuple:
                path = []
                while current_node is not None:
                    path.append(np.array(current_node))
                    current_node = previous[current_node]
                return path[::-1]

            if current_node in visited:
                continue

            visited.add(current_node)

            for neighbor in self.graph[current_node]:
                if neighbor in visited:
                    continue

                edge_weight = self.graph[current_node][neighbor]['weight']
                new_distance = current_distance + edge_weight

                if neighbor not in distances or new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (new_distance, neighbor))
                    
        return None

def plot_results(self, costs, runtimes, base_number=100, how_many_to_add=100, num_searches=7):
    """
    Plot the results of the PRM analysis.
    
    Args:
        costs (dict): Dictionary mapping k values to lists of path costs
        runtimes (dict): Dictionary mapping k values to lists of runtimes
        base_number (int): Starting number of nodes
        how_many_to_add (int): Number of nodes added in each iteration
        num_searches (int): Number of different n values tested
    """
    import matplotlib.pyplot as plt
    
    # Generate x-axis values (number of nodes)
    n_values = [base_number + i * how_many_to_add for i in range(num_searches)]
    
    # Plot path costs
    plt.figure(figsize=(10, 5))
    for k_name in costs.keys():
        plt.plot(n_values, costs[k_name], marker='o', label=f'k = {k_name}')
    
    plt.xlabel('Number of nodes (n)')
    plt.ylabel('Path Cost')
    plt.title('PRM Path Cost vs Number of Nodes')
    plt.legend()
    plt.grid(True)
    plt.savefig('path_costs.png')
    plt.close()
    
    # Plot runtimes
    plt.figure(figsize=(10, 5))
    for k_name in runtimes.keys():
        plt.plot(n_values, runtimes[k_name], marker='o', label=f'k = {k_name}')
    
    plt.xlabel('Number of nodes (n)')
    plt.ylabel('Runtime (seconds)')
    plt.title('PRM Runtime vs Number of Nodes')
    plt.legend()
    plt.grid(True)
    plt.savefig('runtimes.png')
    plt.close()