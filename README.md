# Swarm

Swarm is code from a bacheloursdegree focused on making a swarm platform, 
it is made using [ROS melodic](http://wiki.ros.org/melodic/Installation), utilizing many of its functions. mostly written in Python

```bash
wiki.ros.org
```

## Contents

This project contains:
- Autopilot for a marine vessel based on GPS and a PID-regulator
- Commmunication to send and recieve data over multicast
- Behaviour to calculate movement and control the vessel based on Boids or PSO

The main files that run as ROS nodes are:
- Autopilot/Swarmpilot.py
- Communication/Boat_TX.py
- Communication/Boat_RX.py
- Behaviour/Behaviour.py

# Bonus

A GCS, Ground Control Station was also made for this project in node-red the JSON file is named GCS_node-red.json

## Usage

```bash
roslaunch <pkg> <launch file> #General ROS command to start

roslaunch <swarm> <system.launch> #specific to start this system
```

### Current work

Nothing - this project was delivered 03.12.2019.


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

Contact me either here on github or over mail: anhellesnes@fhs.mil.no.

## Written by

Kim André Lyssand and Andreas Handal Hellesnes


## License
[MIT](https://choosealicense.com/licenses/mit/)
