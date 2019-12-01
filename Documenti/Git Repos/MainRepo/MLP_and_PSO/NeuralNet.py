import random
import math

"""
The following functions are available activation functions
"""
def act_identity(x):
    # Identity
    return x
def act_relu(x):
    # Rectified linear
    return max(0, x)
def act_sigmoid(x):
    # Sigmoid
    val = 0
    try:
        val = 1 / float(1 + math.e**-x)
    except:
        pass
    return val
def act_binary_step(x):
    # Binary step at x=0
    if x<0:
        return 0
    return 1
def act_tanh(x):
    # tanh - could be replacd with math.tanh directly, here more as a note
    return math.tanh(x)
def act_cos(x):
    # cos - could be replaced with math.cos directly
    return math.cos(x)
def act_sin(x):
    return math.sin(x)
def act_gaussian(x):
    val = 0
    try:
        val = math.pow(math.e, - (x**2 / 2.0))
    except:
        pass
    return val
def act_null(x):
    return 0

"""
The get_genome and load_from_genome methods use this list to assign integer values to functions.
If this list doesn't contain an activation function being used the get_genome and load_from_genome methods will fail.
"""
activation_list = [act_null,
                   act_binary_step,
                   act_relu,
                   act_identity,
                   act_sigmoid,
                   act_sin,
                   act_gaussian,
                   act_cos,
                   act_tanh
                   ]

class NeuralNetwork:

    """
    An empty network is initialised.
    create_input_layer and create_hidden_layer are used to set the actual shape of the network.
    """
    def __init__(self):
        # Initialise variables
        # List of layer sizes
        self.network_size = []
        # Number of inputs accepted when starting network calculation
        self.input_count = 0
        # Number of layers created
        self.layer_count = 0
        # List of weights. Accessed by [layer][neuron][weight]
        self.weights = []
        # List of biases. Accessed by [layer][neuron]
        self.bias = []
        # List of activation functions. Accessed by [layer][neuron]
        self.activation = []
        # List of results. Updated in calculation. Accessed by [output neuron]. Data sets given will only ever need 1 output neuron.
        self.calculated_results = None

    """
    This function creates the first layer of neurons
    """
    # Creates first layer given neuron and input count
    def create_input_layer(self, neuron_count, input_count, activations=activation_list, ranges=(-1.0, 1.0)):
        # Ensure no input layer exists
        if self.layer_count != 0:
            print("Network already has an input layer! Cancelling input layer operation.")
            return
        # Set input count
        self.input_count = input_count
        # Add first layer to network size
        self.network_size.append(neuron_count)
        # Increment layer count (to 1)
        self.layer_count += 1
        # Get random weights and biases
        weights, bias = self.get_random_weights_and_bias(input_count, neuron_count, ranges)
        # Add weights, bias, activation functions to relevant lists
        self.weights.append(weights)
        self.bias.append(bias)
        acts = [random.choice(activations) for _ in range(neuron_count)]
        self.activation.append(acts)

    """
    This function creates all subsequent layers. Number of weights is calculated as (neuron_count * neurons_in_previous_layer)
    """
    # Adds a new layer given neuron count
    def create_hidden_layer(self, neuron_count, activations=activation_list, ranges=(-1.0, 1.0)):
        # Ensure input layer exists
        if self.layer_count == 0:
            print("No input later! Cancelling hidden layer creation.")
            return
        # Add new layer size
        self.network_size.append(neuron_count)
        # Get randomised weights and biases
        weights, bias = self.get_random_weights_and_bias(self.network_size[self.layer_count-1], neuron_count, ranges)
        # Increment layer count
        self.layer_count += 1
        # Add weights, bias, activation functions to relevant lists
        self.weights.append(weights)
        self.bias.append(bias)
        acts = [random.choice(activations) for _ in range(neuron_count)]
        self.activation.append(acts)

    """
    This prints the details of the network, iterating through each layer and neuron, bringing all the weights, biases, and activation functions.
    """
    def print_network_info(self):
        # Print network info
        print("Number of layers:{}".format(self.layer_count))
        print("Network size:{}".format(self.network_size))
        print("Input count:{}".format(self.input_count))
        print("Neurons:")
        for l in range(len(self.weights)):
            # Print info for each layer
            print("  layer {}".format(l))
            for n in range(len(self.weights[l])):
                # Print info for each neuron
                print("    neuron {}".format(n))
                print("    w:{}".format(self.weights[l][n]))
                print("    b:{}".format(self.bias[l][n]))
                print("    a:{}".format(self.activation[l][n]))

    """
    Same as print_network_info but it saves it to a file instead
    """
    def save_network_info(self, file):
        f = open(file, "w+")
        f.write("Number of layers:{}\r\n".format(self.layer_count))
        f.write("Network size:{}\r\n".format(self.network_size))
        f.write("Input count:{}\r\n".format(self.input_count))
        f.write("Neurons:")
        for l in range(len(self.weights)):
            # Print info for each layer
            f.write("  layer {}\r\n".format(l))
            for n in range(len(self.weights[l])):
                # Print info for each neuron
                f.write("    neuron {}\r\n".format(n))
                f.write("    w:{}\r\n".format(self.weights[l][n]))
                f.write("    b:{}\r\n".format(self.bias[l][n]))
                f.write("    a:{}\r\n".format(self.activation[l][n]))

    """
    Given a set of inputs (1 or 2 values from the provided data sets), calculate the output of each neuron through each layer to get the final results.
    """
    def calculate(self, input_values, print_stages=False):
        # Ensure correct number of values given
        if len(input_values) != self.input_count:
            print("Given {} input values but need {}! Cancelling calculation.".format(len(input_values), self.input_count))
            return []
        values = input_values
        # For each layer
        for current_layer in range(len(self.network_size)):
            # Print values at each stage if requested
            if print_stages:
                print("Layer {}, input values: {}".format(current_layer, values))
            layer_size = self.network_size[current_layer]
            source_size = len(self.weights[current_layer][0])
            # Ensure correct number of weights given - shouldn't ever fail
            if source_size != len(values):
                print("Invalid input size during layer {}".format(current_layer))
                return
            # List of outputs from each neurons in a layer
            new_values = []
            # For each neuron in the current layer
            for n in range(layer_size):
                # Calculate value
                total = self.bias[current_layer][n]
                for i in range(len(values)):
                    # Calculate current value and add to total for neuron
                    v = values[i]
                    w = self.weights[current_layer][n][i]
                    total += v * w
                # Append new value to list after activation function
                total_after_activation = self.activation[current_layer][n](total)
                new_values.append(total_after_activation)
            # Replace values with new value
            values = new_values
        # Update calculated values and return
        self.calculated_results = values
        return values

    """
    This function is used to generate a list of random weights and biases between a given range.
    source_count is either the number of input values (for the input layer) or the number of neurons in the previous layer (for all subsequent layers)
    """
    def get_random_weights_and_bias(self, source_count, neuron_count, ranges):
        range_size = ranges[1]-ranges[0]
        weights = []
        bias = []
        for n in range(neuron_count):
            weights.append([])
            for _ in range(source_count):
                # Get random weight
                weights[n].append(random.random() * range_size + ranges[0])
            # Get random bias
            bias.append(random.random() * range_size + ranges[0])
        return weights, bias

    """
    This function sets the weights of a single neuron using the list of weights given
    """
    def set_weights(self, layer, neuron, weights):
        # Ensure layer valid
        if layer >= self.layer_count:
            print("Layer {} given, only {} layers exist.".format(layer, self.layer_count))
            print("Cancelling set_weights operation.")
            return
        # Ensure neuron valid
        if neuron >= self.network_size[layer]:
            print("Neuron {} given, only {} neurons exist for layer {}.".format(neuron, self.network_size[layer], layer))
            print("Cancelling set_weights operation.")
            return
        # Ensure number of weights as expected
        expected_source_size = self.input_count
        if layer > 0:
            expected_source_size = self.network_size[layer - 1]
        if len(weights) != expected_source_size:
            print("{} weights given, expected {} weights for layer {}.".format(len(weights), expected_source_size, layer))
            print("Cancelling set_weights operation.")
            return
        # Set weights to new values
        self.weights[layer][neuron] = weights

    """
    This function sets the bias of a single neuron from the value given
    """
    def set_bias(self, layer, neuron, bias):
        # Ensure layer valid
        if layer >= self.layer_count:
            print("Layer {} given, only {} layers exist.".format(layer, self.layer_count))
            print("Cancelling set_bias operation.")
            return
        # Ensure neuron valid
        if neuron >= self.network_size[layer]:
            print("Neuron {} given, only {} neurons exist for layer {}.".format(neuron, self.network_size[layer], layer))
            print("Cancelling set_bias operation.")
            return
        # Set bias to new value
        self.bias[layer][neuron] = bias

    """
    Similarly to set_bias, this function sets the activation function of a given neuron.
    """
    def set_activation(self, layer, neuron, activation):
        if layer >= self.layer_count:
        # Ensure layer valid
            print("Layer {} given, only {} layers exist.".format(layer, self.layer_count))
            print("Cancelling set_activation operation.")
            return
        # Ensure neuron valid
        if neuron >= self.network_size[layer]:
            print("Neuron {} given, only {} neurons exist for layer {}.".format(neuron, self.network_size[layer], layer))
            print("Cancelling set_activation operation.")
            return
        # Set activation to new function
        self.activation[layer][neuron] = activation

    """
    This function works alongside load_from_genome. A 2-dimensional list is created and returned.
    The first member contains continuous floating-point numbers, effectively the list of all weights and biases in the network.
    The second member contains discrete integers, each one corresponding to the index of a function in activation_list 
    """
    def get_genome(self):
        continuous = []
        discrete = []
        for layer in range(len(self.network_size)):
            for neuron in range(self.network_size[layer]):
                # Get all weights for current neuron
                current_weights = self.weights[layer][neuron]
                # Extend list
                continuous.extend(current_weights)
                # Get current neuron bias
                current_bias = self.bias[layer][neuron]
                # Append to list
                continuous.append(current_bias)
                # Get current activation function
                current_activation = self.activation[layer][neuron]
                # Find relevant index
                current_activation_index = activation_list.index(current_activation)
                # Append to list
                discrete.append(current_activation_index)
        # Return genome
        genome = [continuous, discrete]
        return genome

    """
    This function works alongside get_genome. A 2-dimensional list is accepted and the network parameters are set from it.
    The first member contains continuous floating-point numbers, each of which corresponds to a weight or bias in the network.
    The second member contains discrete integers, each corresponding to the index of a function in activation_list
    """
    def load_from_genome(self, shape, input_count, genome):
        # Unpack genome
        continuous, discrete = genome
        # Running index for weights, bias
        continuous_ind = 0
        # Running index for activation
        discrete_ind = 0
        # Create input layer
        self.create_input_layer(shape[0], input_count)
        # Set weight count for input layer
        weight_count = input_count
        for n in range(shape[0]):
            # Set weights
            self.set_weights(0, n, continuous[continuous_ind:continuous_ind+weight_count])
            # Move index to end of weights
            continuous_ind += weight_count
            # Set bias
            self.set_bias(0, n, continuous[continuous_ind])
            # Increment index
            continuous_ind += 1
            # Set activation
            self.set_activation(0, n, activation_list[discrete[discrete_ind]])
            # Increment index
            discrete_ind += 1
        for l in range(1, len(shape)):
            # Create hidden layer, the rest is the same as input layer
            self.create_hidden_layer(shape[l])
            weight_count = shape[l - 1]
            for n in range(shape[l]):
                self.set_weights(l, n, continuous[continuous_ind:continuous_ind+weight_count])
                continuous_ind += weight_count
                self.set_bias(l, n, continuous[continuous_ind])
                continuous_ind += 1
                self.set_activation(l, n, activation_list[discrete[discrete_ind]])
                discrete_ind += 1
            

    """
    This function calculates a value for the fitness of the current network given a set of data.
    Fitness is set as -1 * MSE of the error between expected output (from data) and calculated output (from network)
    Data takes the form [all_inputs, all_outputs]
    """
    def calc_fitness(self, data):
        # Unpack data
        all_inputs, outputs = data
        # Get all outputs
        net_outputs = self.calc_outputs(all_inputs)
        mse = 0
        for i in range(len(net_outputs)):
            current_error = net_outputs[i] - outputs[i]
            mse += current_error**2
        # 
        mse /= len(data[0])
        return -mse

    """
    Given a set of inputs, this function calculates the corresponding outputs
    Each member of inputs list contains a list (either 1- or 2-dimensional) of input values
    """
    def calc_outputs(self, inputs):
        outputs = []
        for i in range(len(inputs)):
            outputs.append(self.calculate(inputs[i])[0])
        return outputs


# This was purely used in testing to ensure the network was calculating results correctly.
if __name__ == "__main__":
    # Number of input values
    input_count = 2
    # Initialise network
    nn = NeuralNetwork()
    # Input layer with 3 neurons
    nn.create_input_layer(3, input_count)
    # Hidden layer with 2 neurons
    nn.create_hidden_layer(2)
    # Hidden/output layer with 1 neuron, sigmoid function
    nn.create_hidden_layer(1, activations=[act_sigmoid])
    # Print all network info (neuron weights, biases, functions, general network size, etc)
    nn.print_network_info()
    # Show output
    print("Output value: {}".format(nn.calculate([2, 2], print_stages=True)))
