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

class GeneticAlgorithm:
    def __init__(self, distance_matrix, population_size, mutation_rate, num_generations):
        self.distance_matrix = distance_matrix
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.num_generations = num_generations
        self.num_cities = distance_matrix.shape[0]
        self.population = self.initialize_population()

    def initialize_population(self):
        population = []
        for _ in range(self.population_size):
            individual = list(range(1, self.num_cities))  # Start from city 1 to num_cities - 1
            random.shuffle(individual)
            individual = [0] + individual + [0]  # Ensure the path starts and ends at city 0
            population.append(individual)
        return population

    def calculate_fitness(self, individual):
        distance = 0
        for i in range(self.num_cities):
            distance += self.distance_matrix[individual[i]][individual[i + 1]]
        return 1 / distance

    def select_parents(self):
        fitness_scores = [self.calculate_fitness(ind) for ind in self.population]
        parents = random.choices(self.population, weights=fitness_scores, k=self.population_size)
        return parents

    def crossover(self, parent1, parent2):
        child = [-1] * (self.num_cities + 1)
        start, end = sorted(random.sample(range(1, self.num_cities), 2))
        child[start:end] = parent1[start:end]

        p2_index = 1
        for i in range(1, self.num_cities):
            if child[i] == -1:
                while parent2[p2_index] in child:
                    p2_index += 1
                child[i] = parent2[p2_index]
        
        child[0] = child[-1] = 0  # Ensure it starts and ends at city 0
        return child

    def mutate(self, individual):
        if random.random() < self.mutation_rate:
            i, j = random.sample(range(1, self.num_cities), 2)
            individual[i], individual[j] = individual[j], individual[i]
        return individual

    def evolve_population(self):
        new_population = []
        parents = self.select_parents()
        for i in range(0, self.population_size, 2):
            parent1 = parents[i]
            parent2 = parents[i + 1]
            child1 = self.crossover(parent1, parent2)
            child2 = self.crossover(parent2, parent1)
            new_population.append(self.mutate(child1))
            new_population.append(self.mutate(child2))
        self.population = new_population

    def find_shortest_path(self):
        best_path = None
        best_distance = float('inf')

        for generation in range(self.num_generations):
            self.evolve_population()
            for individual in self.population:
                current_distance = self.calculate_fitness(individual)
                if 1 / current_distance < best_distance:
                    best_distance = 1 / current_distance
                    best_path = individual

        return best_path, best_distance
    
class BlackHoleAlgorithm:
    def __init__(self, distance_matrix, num_stars, num_iterations):
        self.distance_matrix = distance_matrix
        self.num_stars = num_stars
        self.num_iterations = num_iterations
        self.num_cities = distance_matrix.shape[0]
        self.stars = self.initialize_stars()

    def initialize_stars(self):
        stars = []
        for _ in range(self.num_stars):
            star = list(range(1, self.num_cities))  # Start from city 1 to num_cities - 1
            random.shuffle(star)
            star = [0] + star + [0]  # Ensure the path starts and ends at city 0
            stars.append(star)
        return stars

    def calculate_fitness(self, star):
        distance = 0
        for i in range(self.num_cities):
            distance += self.distance_matrix[star[i]][star[i + 1]]
        return distance

    def find_shortest_path(self):
        best_star = None
        best_fitness = float('inf')

        for _ in range(self.num_iterations):
            # Find the best star (black hole)
            for star in self.stars:
                current_fitness = self.calculate_fitness(star)
                if current_fitness < best_fitness:
                    best_fitness = current_fitness
                    best_star = star

            # Move stars towards the black hole
            new_stars = []
            for star in self.stars:
                if star != best_star:
                    new_star = self.move_towards_black_hole(star, best_star)
                    new_fitness = self.calculate_fitness(new_star)
                    if new_fitness < best_fitness:
                        best_fitness = new_fitness
                        best_star = new_star
                    new_stars.append(new_star)
                else:
                    new_stars.append(star)
            self.stars = new_stars

            # Absorb stars inside the event horizon and replace with new random stars
            event_horizon = best_fitness / np.sum([self.calculate_fitness(s) for s in self.stars])
            self.replace_absorbed_stars(best_star, event_horizon)

        return best_star, best_fitness

    def move_towards_black_hole(self, star, black_hole):
        new_star = star.copy()
        swap_probability = random.random()
        
        for i in range(1, self.num_cities):
            if random.random() < swap_probability:
                new_star[i] = black_hole[i]

        # Ensure the new star is a valid TSP path (no duplicates except for 0 at start and end)
        if len(set(new_star[1:-1])) < self.num_cities - 1:  # Check if there are duplicates
            remaining_cities = set(range(1, self.num_cities)) - set(new_star[1:-1])
            for i in range(1, self.num_cities):
                if new_star[i] in new_star[:i]:
                    new_star[i] = remaining_cities.pop()

        return new_star

    def replace_absorbed_stars(self, black_hole, event_horizon):
        new_stars = []
        for star in self.stars:
            if star != black_hole and self.calculate_fitness(star) > event_horizon:
                # Replace absorbed star with a new random star
                new_star = list(range(1, self.num_cities))
                random.shuffle(new_star)
                new_star = [0] + new_star + [0]
                new_stars.append(new_star)
            else:
                new_stars.append(star)
        self.stars = new_stars