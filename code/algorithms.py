import numpy as np
import random
from math import fsum

class AntColony:
    def __init__(self, distance_matrix, num_ants, num_iterations, evaporation_rate, alpha, beta):
        self.distance_matrix = distance_matrix
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.evaporation_rate = evaporation_rate
        self.alpha = alpha
        self.beta = beta
        self.num_centroids = distance_matrix.shape[0]
        self.pheromone_matrix = np.ones((self.num_centroids, self.num_centroids)) / self.num_centroids
        self.best_path = None
        self.best_distance = float('inf')

    def find_shortest_path(self):
        for _ in range(self.num_iterations):
            # print(f'iteration {_} \n')
            paths = self.construct_paths()
            self.update_pheromones(paths)
            self.update_best_path(paths)
            # print(f'paths at the end {paths} \n')

        return self.best_path, self.best_distance

    def construct_paths(self):
        paths = []
        for _ in range(self.num_ants):
            # print(f'ant {_} \n')
            path = self.construct_path()
            paths.append(path)
        return paths

    def construct_path(self):
        path = []
        visited_centroids = set()
        # Start from center point, 0th member of centroids
        current_centroid = 0
        path.append(current_centroid)
        visited_centroids.add(current_centroid)

        while len(visited_centroids) < self.num_centroids + 1:
            # If all centroids visited, last stop is back to center point
            if len(visited_centroids) == self.num_centroids:
                path.append(0)
                break
            next_centroid = self.select_next_city(current_centroid, visited_centroids)
            path.append(next_centroid)
            visited_centroids.add(next_centroid)
            current_centroid = next_centroid

        return path

    def select_next_city(self, current_centroid, visited_centroids):
        unvisited_centroids = list(set(range(self.num_centroids)) - visited_centroids)
        pheromone_values = [self.pheromone_matrix[current_centroid][centroid] 
                            for centroid in unvisited_centroids]
        attractiveness_values = [1.0 / self.distance_matrix[current_centroid][centroid] 
                                 for centroid in unvisited_centroids]
        probabilities = np.power(pheromone_values, self.alpha) * np.power(attractiveness_values, self.beta)
        probabilities /= np.sum(probabilities)
        next_centroid = random.choices(unvisited_centroids, probabilities)[0]
        return next_centroid

    def update_pheromones(self, paths):
        self.pheromone_matrix *= (1.0 - self.evaporation_rate)
        for path in paths:
            path_distance = self.calculate_distance(path)
            for i in range(self.num_centroids - 1):
                city_a = path[i]
                city_b = path[i + 1]
                self.pheromone_matrix[city_a][city_b] += 1.0 / path_distance
                self.pheromone_matrix[city_b][city_a] += 1.0 / path_distance

    def update_best_path(self, paths):
        for path in paths:
            path_distance = self.calculate_distance(path)
            if path_distance < self.best_distance:
                self.best_distance = path_distance
                self.best_path = path

    def calculate_distance(self, path):
        distance_list = []
        for i in range(self.num_centroids):
            city_a = path[i]
            city_b = path[i + 1]
            distance_list.append(self.distance_matrix[city_a][city_b])
        # Using fsum to fix the float sum python issue
        distance = fsum(distance_list)
        return distance

