# RSSTurtlebotExploration #
Implementation of a ROS system on a Turtlebot-like robot to explore an unknown environment, mapping it and recognizing various danger signs. 

## Requirements ##
The ROS packages required to run the system, which are not included in the initial ROS full install, are *find_object_2d*, *gmapping* and *map_server*. 

## Running ##
In order to run the whole system, launch the *main.launch* file inside the *exploration_main* package. 
The *rviz* window will appear and display how the whole system works. The white and orange cells display the known free space and the frontiers to explore, the black cells display the inflated obstacles in the environment and the green cells show the generated trajectory to travel from the current robot position to one of the frontiers. The red text shows the position and name of the danger sign detected inside the map. 
The *find_object_2d* window will also appear, showing how the signs are detected from our trained set of objects.
When the exploration terminates, it will save the map to the *result* folder of the *exploration_main* package. Once that is done, simply kill the rest of the nodes either by typing *rosnode kill node1 node2 etc...* or by executing Ctrl-C on the terminal window.




