<div id="top"></div>

<a href="https://jderobot.github.io/"><img src="docs/assets/gif/logo.gif" width="150" align="right" /></a>

# BT Studio

![version](https://img.shields.io/badge/Version-0.1-blue)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![Forks][forks-shield]][forks-url]
[![License](http://img.shields.io/:license-gpl-green.svg)](http://opensource.org/licenses/GPL-3.0)

## Introduction

BT Studio is an **open-source** tool crafted for the development of robotic applications. Its primary objective is to facilitate the quick deployment of behavior tree-based robotic applications within ROS. In BehaviorTrees, a robotic app is defined as an XML tree coupled with actions scripted in Python, which the tool then translates into a ROS 2 package. This process circumvents the unnecessary complexities often associated with ROS-specific configurations, offering developers a more streamlined approach.

## Usage

### Installation

1. You need to ROS2 humble installed in your system. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you haven't. After that, source the ros underlay. 
```bash
source /opt/ros/humble/setup.bash
```

2. Clone the repo and enter it
```bash
git clone https://github.com/JdeRobot/BehaviorTrees
cd BehaviorTrees
```

3. Install the necessary packages
```bash
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
```
4. Install the tree_gardener library
```bash
cd modules/tree_gardener
python3 -m pip install .
```
This is a custom library that the apps use for tree related stuff, like accessing ports. Its installation is crucial!

### Generating your first app

1. Write actions
   
Go to the tree_translator section
```bash
cd ../../tree_translator
```
In the `actions/` folder, you can write actions using whatever dependencies you like, and with full ROS 2 integration. 

The actions must follow the following template:

```python
import py_trees

class TemplateAction(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports = None):

        """ Constructor, executed when the class is instantiated """

        # Configure the name of the behaviour
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports

        ...

    def setup(self, **kwargs: int) -> None:

        """ Executed when the setup function is called upon the tree """

        # Get the node passed from the tree (needed for interaction with ROS)
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "Couldn't find the tree node"
            raise KeyError(error_message) from e

        ...

    def initialise(self) -> None:

        """ Executed when coming from an idle state """

        ...

    def update(self) -> py_trees.common.Status:

        """ Executed when the action is ticked. Do not block! """

        ....
    
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behavior switches to a non-running state """
        ...

```

For examples of how to use ports in your actions, please refer to the [example actions](tree_translator/actions) provided with the repo in the actions folder. Feel free to study and adapt them!

2. Write an xml tree

This is where the magic of your app happens. With the actions you have written, you can define a behavior tree in an XML file. All the nodes of [Davide Faconti's BT.cpp library](https://www.behaviortree.dev/docs/category/nodes-library) are fully supported in BehaviorTrees. 

```xml
<root>
    <BehaviorTree ID="MainTree">
        <ReactiveSequence name="main_seq">
            <ReactiveFallback name="obs_fallback">
                <CheckObstacle name="check_obstacle" obs_port="{n_obs}" amplitude="20"/>
                <Turn name="avoid_obstacle"/>
            </ReactiveFallback>
            <Forward name="move_forward" speed="0.2" obs_port="{n_obs}"/>
        </ReactiveSequence>
    </BehaviorTree>
</root>
```

3. Generate a self-contained tree

```bash
python3 generate_tree.py trees/tree.xml actions/ trees/self_contained_tree.xml
```

Although it is not a common practice, self-contained trees allow greater portability and encapsulate all the complexity of a robotics app into a single file. 

4. Generate a ROS2 app

Given a self-contained tree, `tree_gardener` is already able to transform it into an executable tree to be executed within ROS. BehaviorTrees, to offer greater ease of use and reduce the interaction with ROS, comes with an integrated app generation script. This script takes the previously generated tree and prepares a complete ROS 2 app, that can be directly installed. This is simply a ROS 2 package template with a prepared executor script, which calls all the necessary tree_gardener translator functions. If you know what you are doing, you can directly modify and use the [provided package template](tree_translator/ros_template). 

```bash
python3 generate_app.py trees/self_contained_tree.xml you_app_name
```

This generates a zip file with all the necessary files called `your_app_name.zip`

## Executing your first app

1. Creating a ROS 2 ws and moving the app to the src folder

The previous steps have generated an executable app, that should be placed inside a ROS 2 workspace. 

```bash
mkdir -p your_ws/src/your_app_name
cp your_app_name.zip your_ws/src/your_app_name
cd your_ws/src/your_app_name
unzip your_app_name.zip
```

2. Install execution dependecies

#### BehaviorTrees dependecies

Move to the src of the ws and download BehaviorTrees depedencies
```bash
cd ..
vcs import < your_app/thirdparty.repos
```

A Thirdparty folder is created to store the dependecies. 

#### Dynamic app dependencies
Now, install the dependencies of your actions using rosdep. This is done this way as the dependencies of each app are variable and cannot be known at installation time. 
```bash
cd ..
rosdep install --from-paths src -y --ignore-src
```

3. Compile
4. 
Now, we call colcon build so it prepares the packages for execution.
```bash
colcon build --symlink-install
```

4. Source the ws
   
This is necessary for ros to detect the newly installed packages. 
```bash
source install/setup.bash
```

5. Execution!
   
Finally, we can execute the app. If you are using the recommended simulator:
```bash
ros2 launch tb4_sim tb4_launcher.py
py-trees-tree-viewer 
ros2 run your_app_name executor
```

If you execute these commands from different terminals, please, remember to source the ws in each of them. 

## Video

I have prepared this video to guide you through the installation process. Please, be aware this readme is the one and only source of truth of the project, the videos may not be updated. 

[Installation](https://www.youtube.com/watch?v=MObIPLqlmGM&feature=youtu.be)

## Roadmap

In the future, this project will have total integration with [RoboticsAcademy](https://github.com/JdeRobot/RoboticsAcademy), so developers will be able to develop trees in a graphical interface and deploy them in a dockerized environment. The expected roadmap is as follows: 

* Local execution
* Web-based tree designer with local execution
* Integration with a dockerized environment 

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/JdeRobot/BehaviorTrees
[contributors-url]: https://github.com/JdeRobot/BehaviorTrees/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JdeRobot/BehaviorTrees
[forks-url]: https://github.com/JdeRobot/BehaviorTrees/network/members
[stars-shield]: https://img.shields.io/github/stars/JdeRobot/BehaviorTrees
[stars-url]: https://github.com/JdeRobot/BehaviorTrees/stargazers
[issues-shield]: https://img.shields.io/github/issues/JdeRobot/BehaviorTrees
[issues-url]: https://github.com/JdeRobot/BehaviorTrees/issues
[license-shield]: https://img.shields.io/github/license/opensource.org/licenses/GPL-3.0
[license-url]: http://opensource.org/licenses/GPL-3.0
