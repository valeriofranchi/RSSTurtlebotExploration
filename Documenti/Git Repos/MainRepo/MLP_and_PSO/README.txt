This project was done in collaboration with Alexander Swift (https://github.com/aswift8)

==REQUIREMENTS==
ParticleSwarmOptimisation.py requires numpy for updating particle positions and matplotlib for creating graphs.
NeuralNet.py has no external module requirements.

==RUNNING==
To run the PSO and ANN on a dataset launch the ParticleSwarmOptimisation.py file. It will request a command line input for the data set.
It will print out progress as it starts each iteration.
After the final iteration two graphs are created showing the changes in fitness each iteration, and a third graph plots given data against best network output.

==HYPERPARAMETERS==
All of the hyperparameters that can be changed are in ParticleSwarmOptimisation.py.
Available hyperparameters are
-ANN shape
-PSO swarm size
-Informants per particle
-Step size
-Number of PSO iterations
-Weighting for current velocity
-Weighting for own best position
-Weighting for informants best position
-Weighting for global best position
Some hyperparameters are set in the PSO constructor. These are
-Search space range
-Maximum velocity for continuous dimensions
-Maximum velocity for discrete dimensions
