# gym-iOTA
This is an Open-AI gym environment developed with a modular bot platform named '*iOTA*'. The main motive of this gym is to allow us to test out and develop Algorithms for such a MultiAgent System.
## IOTA
<img src="media/bRoll.gif" width=300 align="right"></img>

This is a cost effective modular robot platform developed by us, This contains two docking plates enabling it to dock with other robots at those locations. This has got a very antique WW1 tank inspired designed which allows the bot too with enough traction when used with catpillar tracks.

For more info on the bot hardware and designs please visit this page [here](/hardware_Designs)
<br/><br/>
## Installation
To install the latest changes one could clone and install like so.
```bash
git clone https://github.com/Robotics-Club-IIT-BHU/gym-iOTA
cd gym-iOTA
pip install -e gym-iOTA
```
else for stable releases. the below would work fine.
```bash
pip install gym-iOTA
```

## Usage
This environment can be accessed using gym api, and a small demo scipt is given below.
#### demo.py
```python
import gym
import gym_iOTA

env = gym.make('iOTA-v0',
                render=True,                  ## This runs the simulator in GUI mode
                no_of_modules=10,             ## This spawns so many no of robots
                no_of_clusters=10,            ## This is used to subdivide the total no of robots into groups named clusters More on it later.
                arena=(2,2),                  ## This sets the dimension of the forseeable space for the system
                low_control=True,             ## This flag makes the action space to be the desired velocity of each wheel touching the ground else is setpoint the bot must travel to.
                )
while True:
  action = np.ones((env.no_of_modules, 4))    
  ## Where for each row we have four velocities
  ## [right_forward_wheel, left_forward_wheel, right_back_wheel, left_back_wheel].
  dock = np.zeros((env.no_of_modules, 4))     ## This is the adjancy matrix storing all the docking relationships
  ## for dock[i][j] = 1 the simulator would dock bot[i] -> bot[j], dock[i][j] = 1 and dock[j][i] = 1 would result in the same joint so only fill one in the dock matrix
  observation, reward, done, info = env.step(action, dock)
  observation                                 ## This is the coordinates+orientation of each bot hence the shape is (no_of_modules, 6)
  ## This being (x_coor, y_coor, z_coor, roll, pitch, yaw) of the bot.
  env.render(mode='cluster')                                ## If the render mode was set 'cluster' then will return the image of the top view of the arena, else if was set to 'human' will plot the Image as well using PIL mostly is not desirable.
  if done:
    break
env.close()                                   ## simply removes all the docks and the bots and disconnects from the simulator.
```
