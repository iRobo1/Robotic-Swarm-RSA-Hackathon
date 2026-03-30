# A Swarming Algorithm for Emergency Response Robots by Team Thermites
We developed a custom swarming algorithm for emergency response robots during the Swarming Hackathon. The hackathon was organised by Delft's Robotics Student Association (RSA).

Our algorithm had the highest swarm performance out of all teams.

Emergency response operations often require multiple teams to act quickly in complex and unpredictable environments. Fires must be extinguished, injured people assisted, flooded areas drained, and debris cleared - all while navigating obstacles and coordinating limited resources. Our robotic swarm operated fully autonomously in this simulated disaster-response environment. Our robots explored the arena, detected objectives using computer vision in real-time, communicated dicoveries with other swarms, and delivered supply items to the corresponding objectives.

## Our Team
Our team consisted of Albert, Alexander, Franek, Nuria, Luis, Lukas, Mare, [Sandro](https://github.com/iRobo1), Steven, and Tomas.

## Implementation
We programmed our MIRTE Pioneer and Master robots in ROS. We utilise the RGB camera, depth camera, ultrasonic sensors, and LiDAR for robot, obstacle, and objective detection. We dynamically distribute map information across the robots in the swarm, such that each robot only stores a local map around themselves. This minimises memory consumption and allows the robot swarm(s) to theoretically be scaled to hundreds of robots. Information is transmitted only when necessary and only to specific target robots. The robot swarm is robust in the sense that if any robots fail, the remaining ones can continue working without human intervention. The swarm also shares relevant information with other robot swarms that are designed to solve different tasks.

## Documentation
Since this was a hackathon, writing code was prioritised over documentation. Thus, the documentation is very limited. 

Instructions for running the code on MIRTE robots can be found in [docs/README.md](./docs/README.md).

Unfortunately, some small parts of the codebase were also modified last minute and are not saved on github.

## Hackathon Sponsors
Thank you to the Robotics Student Association (RSA) for organising this amazing hackathon and to all the hackathon sponsors who made this possible:
- RoboHouse
- Robotics Student Association
- Even Groene Vrienden
- Beethoven
- Asimov
- TU Delft Science Centre Swarming Lab
- Lunar Zebro
