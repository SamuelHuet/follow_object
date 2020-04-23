# [Navigation Project](https://github.com/SamuelHuet/simple_navigation_goals)

_[![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-red)](http://wiki.ros.org/melodic/Installation/Ubuntu)_ _[![TurtleBot3](https://img.shields.io/badge/TurtleBot-3-brightgreen)](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)_ ![OpenCV](https://img.shields.io/badge/OpenCV-2-yellow) ![LICENSE](https://img.shields.io/badge/LICENSE-Apache%202.0-informational)

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/Ros_logo.svg/800px-Ros_logo.svg.png" width="110"> ![MelodicTurtle][melodic-turtle]

>The goal of this projet is to succeed a simplified version of the "Carry my luggage" test, imagined by the [RobotCup@Home](https://athome.robocup.org) contest.

- [Navigation Project](#navigation-project)
  - [Rules :](#rules)
        - [Phase one : Tracking](#phase-one--tracking)
        - [Phase two : Homecoming](#phase-two--homecoming)
        - [About](#about)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [How To](#how-to)
  - [Usage](#usage)
  - [Meta](#meta)
  - [Contributing](#contributing)


## Rules :

>At the beginning of the challenge, the starting positions of both the TurtleBot and the object to follow will be given.

##### Phase one : Tracking

>By default the object to follow is a cylinder, but it can be modified with permission of the professor. When the contest starts, the object will move with a sufficiently slow speed to his destination. It won't be the only moving object, it is the respnsability of the TurtleBot to track the same object without mistakes during this phase of the test. If the object remain stationnary during a period of 3 seconds, we can consider that the robot has arrived at its destination.
>
>Phase one validation conditions :
>  - No collision happened during the test
>  - The object has arrived at destination and the robot remains stationnary within 1.5m of the object for 3 seconds.

##### Phase two : Homecoming

>In this phase the goal of the robot is to return to his starting point. The main difficulty comes from the terrain modifications and obstacles that may appear. No collision is allowed.
>
>Phase two validation condition :
>  - The robot remains stationary within 0.3 meters of its starting point.
>
>Time will be used to break the equality between the teams that have validated the two phases.

##### About

>For more information about rules and regulations, please refer to [this document](https://robocupathome.github.io/RuleBook/rulebook/master.pdf).

## Prerequisites

- [Ubuntu 18.0.4 recommended](https://ubuntu.com/download/desktop)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)
- [OpenCV_2](https://opencv.org/)
- [Simple_navigation_goal](https://gitlab.com/catie_robotics/slam/tiago/simple_navigation_goals)

## Installation

First clone this repository on your computer and compile the project using catkin_make:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SamuelHuet/follow_object.git
```

Copy the map folder into your `home/` folder and `catkin_make`
```
$ cp -r follow_object/map/ ~/
$ cd ~/catkin_ws && catkin_make
```

Change the first line of `~/map/map.yaml` with your home directory
```
image: /home/user/map_friendly/map.pgm
```
## How To

Launch Gazebo et Rviz with the full stack navigation
```
$ roslaunch turtlebot3_gazebo turtlebot3_house_animated_friendly_no_plugin.launch
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map/map.yaml
$ rosrun follow_object follower.py
```
Enjoy

## Usage

The `follower.py` script read the data from the lidar and draw all the points in a PNG file.

## Meta

Distributed under the Apache 2.0 license. See ``LICENSE`` for more information.

## Contributing

1. Fork it (<https://github.com/SamuelHuet/follow_object/fork>)
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[melodic-turtle]: https://raw.githubusercontent.com/ros/ros_tutorials/melodic-devel/turtlesim/images/melodic.png
