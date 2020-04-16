# [Navigation Project](https://github.com/SamuelHuet/simple_navigation_goals)

_[![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-red)](http://wiki.ros.org/melodic/Installation/Ubuntu)_ _[![TurtleBot3](https://img.shields.io/badge/TurtleBot-3-brightgreen)](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)_ ![OpenCV](https://img.shields.io/badge/OpenCV-2-yellow) ![LICENSE](https://img.shields.io/badge/LICENSE-Apache%202.0-informational)

![MelodicTurtle](https://raw.githubusercontent.com/ros/ros_tutorials/melodic-devel/turtlesim/images/melodic.png)

>The goal of this projet is to succeed a simplified version of the "Carry my luggage" test, imagined by the [RobotCup@Home](https://athome.robocup.org) contest.

- [Navigation Project](#navigation-project)
  - [Rules](#rules)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [How To](#how-to)
  - [Usage](#usage)
  - [Meta](#meta)
  - [Contributing](#contributing)


## Rules

>At the beginning of the challenge, the starting positions of both the TurtleBot and the object to follow will be given.

#####Phase one : Tracking

>By default the object to follow is a cylinder, but it can be modified with permission of the professor. When the contest starts, the object will move with a sufficiently slow speed to his destination. It won't be the only moving object, it is the respnsability of the TurtleBot to track the same object without mistakes during this phase of the test. If the object remain stationnary during a period of 3 seconds, we can consider that the robot has arrived at his destination.
>
>Phase one validation conditions :
>  - No collision happened during the test
>  - The object has arrived at destination and the robot remains stationnary within 1.5m of the object for 3 seconds.

#####Phase two : Homecoming

>

>For more information about rules and regulations, please refer to [this document](https://robocupathome.github.io/RuleBook/rulebook/master.pdf).

## Prerequisites

- [Ubuntu 18.0.4 recommended](https://ubuntu.com/download/desktop)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)
- [Simple_navigation_goal](https://gitlab.com/catie_robotics/slam/tiago/simple_navigation_goals)

## Installation

Simply clone this repository on your computer and catkin_make:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SamuelHuet/follow_objet.git
$ cd ~/catkin_ws
$ catkin_make
```

## How To

## Usage

## Meta

Distributed under the Apache 2.0 license. See ``LICENSE`` for more information.

## Contributing

1. Fork it (<https://github.com/SamuelHuet/follow_object/fork>)
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request
