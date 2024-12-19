import time
import heapq
from collections import defaultdict
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
import math
import matplotlib.pyplot as plt
from building_blocks import BuildingBlocks2D

class PRMController:
    def __init__(self, start, goal, bb):
        self.graph = nx.Graph()
        self.bb = bb
        self.start = start
        self.goal = goal
        self.robot = BuildingBlocks2D()
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
        # TODO: HW2 4.3.5
        pass

    def gen_coords(self, n=5):
        """
        Generate 'n' random collision-free samples called milestones.
        n: number of collision-free configurations to generate
        """
        num = 0
        
        # TODO: HW2 4.3.1
        configs = []
        while num < n:
            config = np.random.uniform(low = -np.pi, high = np.pi , size= 4 )
            if self.robot.config_validity_checker(config):
                num += 1
            configs.append(config)
        return configs
    
  
    
    def add_to_graph(self, configs, k):
        """
            add new configs to the graph.
        """
        for config in configs:
            if config.tolist() not in [c.tolist() for c in self.configs]:
                self.configs.append(config)
                self.graph.add_node(tuple(config))

        self.kdtree = KDTree(self.configs)

        # For each config, find and connect to k nearest neighbors
        for config in configs:
            neighbors = self.find_nearest_neighbour(config, k)
            for neighbor in neighbors:
                distance = self.robot.compute_distance(config, neighbor)
                if self.robot.edge_validity_checker(config, neighbor):
                    self.graph.add_edge(tuple(config), tuple(neighbor), 
                                      weight=distance)

    def find_nearest_neighbour(self, config, k=5):
        """
            Find the k nearest neighbours to config
        """
        if len(self.configs) == 0:
            return []
        
        if self.kdtree is None:
            self.kdtree = KDTree(self.configs)
        
        distances, indices = self.kdtree.query(config, k=min(k+1, len(self.configs)))
    
        neighbors = []
        for i, idx in enumerate(indices):
            if not np.array_equal(self.configs[idx], config):
                neighbors.append(self.configs[idx])
                if len(neighbors) == k:
                    break
                
        return neighbors
    
    def shortest_path(self):
        """
            Find the shortest path from start to goal using Dijkstra's algorithm (you can use previous implementation from HW1)'
        """
        start_tuple = tuple(self.start)
        goal_tuple = tuple(self.goal)

        if start_tuple not in self.graph or goal_tuple not in self.graph:
            return None, float('inf')

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
                return path[::-1], current_distance

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

        return None, float('inf')
