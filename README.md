Just an empty readme
# Team Thermites
We're team number `5` with the RED team colour. 

## Working With Github
To implement a new feature:
1. Create a new branch called `feature/<descriptive name>` from the `development` branch
2. Commit your changes frequently
3. Create a pull request when your feature is ready and have someone else merge it (the other person should review your code before merging it). Branches and frequent commits will help avoid complex merge conflicts

To update `docs/` or this `README.md` you may commit directly to the development branch.

## Code Structure
`src/` contains some folders for code. Feel free to expand the folder structure. Try to include a comment at the top of every file or a README.md in every folder that briefly describes what the file or folder contains.

`src/communication` - for all things related to communicating between the robots and the server (i.e., http requests all go there)<br>
`src/computer-vision` - for all the code that processes and analyses data from the robot cameras and LiDAR<br>
`src/gripper-robot` - the gripper robot code goes in here (there are several files, feel free to add more)<br>
`src/pioneer-robot` - the pioneer robot code goes in here (there are several files, feel free to add more)<br>
`src/arena.py` - contains a bunch of abstractions for objects inside the arena (robots, teams, baskets, obstacles, arena)<br>
`src/main.py` - main entry point to the program

## Documentation
Use `docs/` to store any plans, strategies, and ideas. Also use it to store pictures and other materials for the final presentation.

## Connecting to the MIRTE Pioneer Robots
Team number: 5
Team 5 password (WiFi & Login): uncertain-decoy-dandruff

Team 3 password (the team 5 robots are not ready yet): stagnant-attractor-companion

Manually moving the robot:
1. Flip the power switch (wait 1-2 min for startup)
2. Connect to WiFi (Swarming Team X)
3. Navigate to http://172.18.2.\<team\>\<robot\> e.g. Team 2, Robot 4 -> http://172.18.2.24
4. Login using the username "mirte" and the password (above)
5. Use the control panel to control the robots

Using VS Code:
1. Go to http:172.18.2.\<team\>\<robot\>:8000 to open VS Code
2. Copy over the small_mirte_library.py file to the workdir folder
(You can find the file at https://cloud.rsadelft.nl/s/swarming, then drag it into VS Code)
3. Create workshop.py int he workdir folder
4. Install dependencies with:
```pip install apriltag requests websockets opencv-python numpy```

## Connecting to the MIRTE Master Robot
