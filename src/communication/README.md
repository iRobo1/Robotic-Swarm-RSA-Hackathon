# Server and Robot Communication
This folder contains all the code for communicating between our robots, other swarms, and the server.


# Robot Responsibilities

## Intra-Swarm Communication (within our swarm)
Any information that the pioneer robots want to communicate with the rest of their own swarm is sent to the main Gripper robot <br>
The main Gripper robot then combines the information and broadcasts it to the Pioneer robots (one message per target robot) <br>
Reasoning for this over broadcasting:
- Reduces the number of sent HTTP messages (25 -> 8)
- Any information being sent from Pioneer robots to other Pioneer robots is non-essential (swarm will still function without it)
- Any information sent to the Gripper is essential because the Gripper is the only way to score points outside of identifying new April Tags

## Inter-Swarm Communication (to other swarms)
Pioneer robots that identify April Tags should send them directly to the target team (and also to the Gripper robot to inform our swarm)
Reasoning:
- This is essential for scoring points, so it is best to distribute the responsibility to send it to the target team among the swarm in case any single robot has issues