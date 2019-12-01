import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import random
import copy
import sys
from NeuralNet import NeuralNetwork, activation_list
from NeuralNet import act_identity, act_sigmoid, act_tanh, act_relu, act_binary_step, act_cos, act_gaussian, act_null

# Used for testing changes to hyperparameters
test1 = []
test2 = []

class PSO:
    def __init__(self, filePath, networkShape, swarmSize, informants, alpha, beta, gamma, delta, epsilon):
        self.shape = networkShape #shape of neural network 
        self.data = self.load_data(filePath) #load neural network training data
        self.input_count = len(self.data[0][0]) #input count for neural network
        
        self.swarmSize = swarmSize #number of particles for the optimisation process
        self.informant_count = informants
        self.alpha = alpha #inertia weight 
        self.beta = beta #cognitive weight component
        self.gamma = gamma #social weight component 
        self.delta = delta #proportion of global best to be retained 
        self.epsilon = epsilon #step size

        self.input_activation_list = [act_null, act_sigmoid, act_tanh, act_cos, act_gaussian]
        self.hidden_activation_list = [act_null, act_sigmoid, act_tanh, act_cos, act_gaussian]
        self.output_activation_list = [act_null, act_sigmoid, act_tanh, act_cos, act_gaussian]

        self.particles = [] #list of particles (each containing positions, velocities, best positions, 
        #best fitness, informant indices)
        self.best_position = None #best position
        self.best_fitness = 0 # Best fitness
        self.max_vel = 1.0
        self.particle_bounds = (-15, 15)
        self.max_discrete_vel = 0.3
        # Limit bounds on a per-position basis, one bound per neuron
        self.discrete_particle_bounds = [len(self.input_activation_list)] * self.shape[0]
        for l, neurons in enumerate(self.shape[1:]):
            if l == len(self.shape[1:]) - 1:
                self.discrete_particle_bounds.extend([len(self.output_activation_list)]*neurons)
            else:
                self.discrete_particle_bounds.extend([len(self.hidden_activation_list)]*neurons)

        # Neural network used to calculate fitnesses
        self.neural_net = NeuralNetwork()
        self.create_neural_network()

        self.continuous_count = len(self.neural_net.get_genome()[0])
        self.final_run = []

    """
    This function will open the data file, append x input(s) and y output and return them in the form of a 2D list.
    """
    def load_data(self, file_path):
        inputs = []
        outputs = []
        file_name = os.path.split(file_path)[-1]
        print("[INFO] Opening {}...".format(file_name))
        with open(file_path) as f:
            lines = f.readlines()
            for line in lines:
                values = line.split()
                X = values[:-1]
                y = values[-1]
                inputs.append(X)
                outputs.append(y)
        outputs, inputs = np.asarray(outputs), np.asarray(inputs)
        outputs, inputs = outputs.astype('float64'), inputs.astype('float64')
        return [inputs, outputs]
    
    """
    This function will use a NeuralNet object to initialise an ANN.
    """
    def create_neural_network(self):
        self.neural_net = NeuralNetwork()
        #create input layer
        self.neural_net.create_input_layer(neuron_count=self.shape[0], input_count=self.input_count, 
             activations=self.input_activation_list, ranges=self.particle_bounds)
        #create hidden and output layers 
        for i, neurons in enumerate(self.shape[1:]):
            if i < len(self.shape[1:]) - 1:
                self.neural_net.create_hidden_layer(neuron_count=neurons, activations=self.hidden_activation_list,
                    ranges=self.particle_bounds)
            else:
                self.neural_net.create_hidden_layer(neuron_count=neurons, activations=self.output_activation_list,
                    ranges=self.particle_bounds)

    """
    This function initialises each particle with positions and velocities and then stores them in a swarm (array).
    """
    def initialise_state(self):
        #initialize swarm of particles
        for i in range(self.swarmSize):
            print("[INFO] Initialising particle number {}...".format(i + 1))
            self.create_neural_network()
            pos_discrete = [random.randrange(self.discrete_particle_bounds[j]) for j in range(len(self.neural_net.get_genome()[1]))]
            positions = np.array(copy.deepcopy(self.neural_net.get_genome()[0]) + pos_discrete) #generate random weights and biases 
            self.create_neural_network()
            vel_discrete = copy.deepcopy(self.neural_net.get_genome()[1])
            # Bound discrete velocities by max_discrete_vel
            vel2 = [(random.random() * 2 - 1.0) * self.max_discrete_vel for _ in vel_discrete]
            velocities = np.array(copy.deepcopy(self.neural_net.get_genome()[0]) + vel2) #generate random velocities 
            bestPositions, bestFitness = positions, self.neural_net.calc_fitness(self.data)
            informants = np.random.randint(self.swarmSize, size=self.informant_count)
            self.particles.append([positions, velocities, bestPositions, bestFitness, informants])
        self.particles = np.asarray(self.particles)

        #initialize best particle
        print("[INFO] Initialising best particle...")
        idx = np.argmax([p[3] for p in self.particles])
        self.best_position = copy.deepcopy(self.particles[idx][0])
        self.best_fitness = self.particles[idx][3]

    """
    This function was used for testing but it is not used in the final implementation. 
    """
    def initialise_test_state(self):
        pos_range = self.particle_bounds[1] - self.particle_bounds[0]
        for _ in range(self.swarmSize):
            # Create random positions, velocities
            px = random.random()*pos_range + self.particle_bounds[0]
            py = random.random()*pos_range + self.particle_bounds[0]
            pos = np.array([px, py])
            vx = random.random()*pos_range + self.particle_bounds[0]
            vy = random.random()*pos_range + self.particle_bounds[0]
            vel = np.array([vx, vy])
            informants = np.random.randint(self.swarmSize, size=5)
            fit = self.calculate_test_fitness(pos)
            self.particles.append([pos, vel, pos, fit, informants])
        self.particles = np.asarray(self.particles)

    """
    This function was used for testing but it is not used in the final implementation. 
    """
    def calculate_test_fitness(self, pos):
        return - (pos[0] ** 2) - (pos[1] ** 2)

    """
    This function uses the updated particle positions to update the neural network NeuralNet object.
    """
    def fill_network(self, particle_position):
        continuous, discrete_pos = particle_position[:self.continuous_count], particle_position[self.continuous_count:]
        discrete = []
        for i, d in enumerate(discrete_pos):
            if i < self.shape[0]:
                # Input layer
                discrete.append(activation_list.index(self.input_activation_list[int(d)]))
            elif i >= len(discrete_pos) - self.shape[-1]:
                # Output layer
                discrete.append(activation_list.index(self.output_activation_list[int(d)]))
            else:
                # Hidden layer
                discrete.append(activation_list.index(self.hidden_activation_list[int(d)]))
        genome = [continuous, discrete]
        self.neural_net = NeuralNetwork()
        self.neural_net.load_from_genome(self.shape, self.input_count, genome)

    """
    This function uses the NeuralNet function calc_fitness to calculate the fitness of the current particle positions.
    """
    def calculate_fitness(self, particle_position):
        self.fill_network(particle_position)
        return self.neural_net.calc_fitness(self.data)
    
    """
    PSO main function: performs the optimization algorithm, returns best particle position and plots results.
    """
    def optimize(self, iterations, display_results=True):
        #list of best fitnesses for each iteration
        fitnesses = []
        best_fitnesses = []
        avg_fitnesses = []
        worst_fitnesses = []

        #initialise state
        self.initialise_state()

        #perform training iterations to tune neural network parameters 
        for i in range(iterations):
            print("[INFO] Running iteration number {}...".format(i + 1))

            # Keep track of best, worst, and mean fitnesses
            best_fitness = -999999
            worst_fitness = 999999
            total_fitness = 0
            
            #update global best particle
            for particle in self.particles:
                #assign positions to genome continuous variables and calculate fitness
                fitness = self.calculate_fitness(particle[0])

                if i == (iterations - 1):
                    self.final_run.append(fitness)
                
                #if fitness of current particle is better than current best, assign the fitness as the best
                #fitness of the particle, and update best positions variable in particle tuple 
                if fitness > particle[3]:
                    particle[3] = fitness
                    particle[2] = copy.deepcopy(particle[0])

                #if particle has better fitness than the one of best particle, make it the best particle
                if self.best_position is None or fitness > self.best_fitness:
                    self.best_position = copy.deepcopy(particle[0])
                    self.best_fitness = fitness

                # Update current best, worst, and total fitnesses
                if fitness > best_fitness:
                    best_fitness = fitness
                if fitness < worst_fitness:
                    worst_fitness = fitness
                total_fitness += fitness

            for particle in self.particles:
                #gather information
                x_curr = particle[2] #best positions of current particle 
                informants = self.particles[np.array(particle[4])] #informant particles of current particle
                idx = np.argmax([inf[3] for inf in informants]) #index referring to informant with highest fitness
                x_informant = informants[idx][0] #best positions out of informants of current particle 
                x_global = self.best_position #positions of best particle (with highest fitness)

                #update particle velocity
                pos, vel, pos_curr, pos_info, pos_glob = particle[0], particle[1], x_curr, x_informant, x_global
                a = np.random.normal(loc=1.0) * self.alpha
                b = np.random.normal(loc=1.0) * self.beta
                c = np.random.normal(loc=1.0) * self.gamma
                d = np.random.normal(loc=1.0) * self.delta
                vel = a*vel + b*(pos_curr-pos) + c*(pos_info-pos) + d*(pos_glob-pos)
                #scaling factor for individual velocities
                #0.1 to avoid division by 0
                vel_magnitude = np.sqrt(np.sum(np.square(particle[1][:self.continuous_count]) + 0.000001))
                if vel_magnitude > self.max_vel:
                    vel_scale = self.max_vel / vel_magnitude
                    vel[:self.continuous_count] *= vel_scale
                for disc in range(self.continuous_count, len(vel)):
                    if vel[disc] > self.max_discrete_vel:
                        vel[disc] = self.max_discrete_vel
                    elif vel[disc] < -self.max_discrete_vel:
                        vel[disc] = -self.max_discrete_vel
                particle[1] = vel

            #update positions 
            #num_positions = len(particle[0])
            for particle in self.particles:
                for l, (position, velocity) in enumerate(zip(particle[0], particle[1])):
                    position += self.epsilon * velocity
                    if l < self.continuous_count:
                        # Continuous section
                        if self.particle_bounds[0] < position < self.particle_bounds[1]:
                            particle[0][l] = position
                    else:
                        # Discrete section
                        if 0 <= position < self.discrete_particle_bounds[l-self.continuous_count]:
                            particle[0][l] = position
            
            #update fitnesses list
            fitnesses.append(self.best_fitness)
            best_fitnesses.append(best_fitness)
            worst_fitnesses.append(worst_fitness)
            avg_fitnesses.append(total_fitness / float(self.swarmSize))
        
        if display_results == True:
            best_outputs = self.get_best_results()
            #make graph to display best fitness over iterations and network final output
            self.show_optimization_history(fitnesses, best_fitnesses, worst_fitnesses, avg_fitnesses, best_outputs)
            plt.show()
        return self.best_position
    
    """
    This function is used to retrieve results of the final neural network and final fitness values with 
    changing hyperparameter values. 
    """
    def hyperparameter_fitness_tests(self, iteration, file, results_name):
        file_name = results_name
        f = open(file_name, "a+")
        try:
            if os.path.getsize(file_name) == 0:
                f.write("Swarm size: {}\r\n".format(self.swarmSize))
                f.write("Function: {}\r\n".format(file))
        except OSError:
            sys.exit()        
        f.write("Test Number {}\r\n".format(iteration))
        f.write("Final Fitness: {}\r\n".format(self.calculate_fitness(self.best_position)))
        test1.append(self.calculate_fitness(self.best_position))
        f.write("Mean Final Iteration Fitness: {}\r\n".format(np.sum(self.final_run)/len(self.particles)))
        test2.append(np.sum(self.final_run)/len(self.particles))
        if iteration == 10:
            f.write("Mean of Test1: {}\r\n".format(np.sum(test1)/iteration))
            f.write("Mean of Test2: {}".format(np.sum(test2)/iteration))

    """
    This function uses the best current positions and calculates the output y values from the neural network.
    """
    def get_best_results(self, file_name=None):
        # Create network from best particle
        self.fill_network(self.best_position)
        outputs = self.neural_net.calc_outputs(self.data[0])
        # Print network info
        self.neural_net.print_network_info()
        # Save data and append fitness if given a file name
        if file_name is not None:
            self.neural_net.save_network_info(file_name)
            f = open(file_name, "a")
            f.write("Fitness of Network: {}".format(self.calculate_fitness(self.best_position)))
        return outputs

    """
    This function plots the PSO fitness results over the iterations and the estimated function vs ground-truth function.
    """
    def show_optimization_history(self, fitnesses, best_fitnesses, worst_fitnesses, avg_fitnesses, best_outputs, file_name=None):
        print("[INFO] Displaying results...")
        fig = plt.figure(figsize=(20, 15))
        fitnesses = np.asarray(fitnesses)
        iterations = np.array(range(len(fitnesses)))

        ax = fig.add_subplot(1, 3, 1)
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Best Particle Fitness")
        ax.set_xlim([0, np.max(iterations)])
        ax.plot(iterations, fitnesses, color="red", label="Best Overall Fitness")
        ax.plot(iterations, best_fitnesses, color="cyan", label="Best Fitness Per Iteration")
        ax.plot(iterations, worst_fitnesses, color="blue", label="Worst Fitness Per Iteration")
        ax.plot(iterations, avg_fitnesses, color="green", label="Average Fitness Per Iteration")
        ax.legend()

        ax = fig.add_subplot(1, 3, 2)
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Best Particle Fitness")
        ax.set_xlim([0, np.max(iterations)])
        ax.plot(iterations, fitnesses, color="red", label="Best Overall Fitness")
        ax.plot(iterations, best_fitnesses, color="cyan", label="Best Fitness Per Iteration")
        ax.legend()
        
        if np.asarray(self.data[0]).shape[1] == 2:
            x_input = self.data[0][:, 0]
            y_input = self.data[0][:, 1]
            z_output = best_outputs
            ax = fig.add_subplot(1, 3, 3, projection='3d')
            ax.scatter3D(x_input, y_input, z_output, color="violet", label="Ground Truth")
            ax.scatter3D(x_input, y_input, self.data[1], color="black", label="MLP and PSO Estimation")
            ax.legend()
        else:
            ax = fig.add_subplot(1, 3, 3)
            ax.plot(self.data[0], self.data[1], color="black", label="Ground Truth")
            ax.plot(self.data[0], best_outputs, color="violet", label="MLP and PSO Estimation")
            ax.legend()
        # If file name given, save plot to it
        if file_name is not None:
            plt.savefig(file_name)

if __name__ == '__main__':
    # Retrieve data set to be tested
    data_types = {
        "linear":1,
        "cubic":1,
        "sine":1,
        "tanh":1,
        "xor":2,
        "complex":2
        }
    input_type = input("Enter data type to test: ")
    while input_type not in data_types:
        print("Data type not found. Please enter one of the following:")
        print("linear, cubic, sine, tanh, xor, complex")
        input_type = input("Enter data type to test: ")
    input_size = data_types[input_type]

    # Data set path 
    file_path = "data/{}in_{}.txt".format(input_size, input_type)

    # ANN hyperparameters
    shape = (4, 4, 1)   # Network shape, default (4, 4, 1)

    # PSO hyperparameters 
    swarm_size = 100  # Number of particles in the swarm, default 100
    informants = 5     # Number of particle informants, default 5
    epsilon = 0.3    # Step size, default 0.3
    iterations = 500  # Number of PSO iterations , default 500
    alpha = 1.0    # Current velocity
    beta = 1.0          # Own best position
    gamma = 1.5        # Informants best position
    delta = 0.0         # Global best position

    # Create PSO object
    pso = PSO(file_path, shape, swarm_size, informants, alpha, beta, gamma, delta, epsilon)
    # Optimize neural network weights and display results 
    pso.optimize(iterations, display_results=True)
