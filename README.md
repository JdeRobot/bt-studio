<div id="top"></div>

<a href="https://jderobot.github.io/"><img src="docs/assets/gif/logo.gif" width="150" align="right" /></a>

# BT Studio

![version](https://img.shields.io/badge/Version-0.4-blue)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![Forks][forks-shield]][forks-url]
[![License](http://img.shields.io/:license-gpl-green.svg)](http://opensource.org/licenses/GPL-3.0)

## Introduction

BT Studio is an **open-source** tool crafted for the development of robotic applications. Its primary objective is to facilitate the quick deployment of behavior tree-based robotic applications within ROS 2. In BT Studio, a robotic app is defined as an graphical tree coupled with actions scripted in Python, which the tool then translates into a ROS 2 package. This process circumvents the unnecessary complexities often associated with ROS-specific configurations, offering developers a more streamlined approach.

## Usage

### Basic: No cloning repo

To save the code you will need to download the app. Everything will be reset every time the command below is launched.

```bash
curl -s https://raw.githubusercontent.com/JdeRobot/main/scripts/run.sh | sudo bash
```

### Developer

Here the code will be saved in 'backend/filesystem' and the current database and bt-studio will be used inside the docker.

1. Clone the repo and enter it
```bash
git clone --recurse-submodules https://github.com/JdeRobot/bt-studio.git -b <src-branch>
cd bt-studio
```

2. Launch the docker: it can be done in the following ways.

* **Recomended** Using the developer script (docker compose):
```bash
sh scripts/develop.sh
```

* Using docker run **(does not use the current database and bt-studio)**:
```bash
docker run --hostname my-postgres --name universe_db -d\
    -e POSTGRES_DB=universe_db \
    -e POSTGRES_USER=user-dev \
    -e POSTGRES_PASSWORD=bt-studio-dev \
    -e POSTGRES_PORT=5432 \
    -d -p 5432:5432 \
    jderobot/bt-studio-database:latest
docker run --rm -it $(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "") --device /dev/dri -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 --link universe_db jderobot/bt-studio:latest
```

### Generating apps

1. Create new app

You may create a new app clicking the **+** button in the upper right corner, or swap to the the demo project clicking the change button next to it and typing *demo*. In the demo project, you may find a reference implementation of a bump and go app for a differential robot using a LIDAR. Feel free to explore and use whatever you need!

2. Write actions

You may create as many actions as needed using the file browser. The actions must have a .py extension and follow this structure:

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

For examples of how to use ports in your actions, please refer to the [example actions](backend/filesystem/demo/actions/) provided with the repo in the actions folder. The structure is the same as [py_trees actions](https://py-trees.readthedocs.io/en/devel/behaviours.html), as this project uses the library underneath. Kudos to @stonier!

3. Define the tree structure

Using the existing blocks, you may define the logic of your app. Here you have available all the defined actions. This can be done previously to step 2 if you prefer to define the logic of the tree before its implementation!

4. Downloading the app

Clicking the green download button, a ROS 2 app is generated. This can be unziped in a ROS 2 workspace and installed as any other package. 

### Using the generated app

1. You need to ROS 2 humble installed in your system. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you haven't. After that, source the ros underlay. It is highly recommended to add this line at the end of your `.basrhc` file. 

```bash
source /opt/ros/humble/setup.bash
```

2. Moving the app to a ROS 2 ws

```bash
mkdir -p your_ws/src
mv your_app.zip your_ws/src
unzip your_ws/src/your_app.zip
```

3. Init rosdep

You may have to init the ROS dependency manager if you haven't done it before. 

```bash
sudo rosdep init
rosdep update
```

4. Downloading auxiliary tools and dependencies

These are a testing environment in the Webots simulator and a tree execution visualizer. 

```bash
cd src/your_app/
vcs import < demo/thirdparty.repos
cd ../..
rosdep install --from-paths src -r -y --ignore-src
```

5. Compiling

```bash
colcon build --symlink-install
source install/setup.bash
```

6. Launching a testing environment

You may choose whatever environment for your app to execute into, but we provide some prepared packages for convenience.

```bash
ros2 launch tb4_sim tb4_launcher.py
py-trees-tree-viewer
```

7. Launching the app

Now, using the app executor, you may launch the app itself. 

```bash
ros2 run your_app_name executor
```

## Showcase

You can see a running application as reference here:

[Showcase video](https://www.youtube.com/watch?v=Bo4SJEiBKGQ)

## Roadmap

In the future, this project will have total integration with [RoboticsAcademy](https://github.com/JdeRobot/RoboticsAcademy), so developers will be able to deploy the trees in a dockerized environment. 

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/JdeRobot/bt-studio
[contributors-url]: https://github.com/JdeRobot/bt-studio/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JdeRobot/bt-studio
[forks-url]: https://github.com/JdeRobot/bt-studio/network/members
[stars-shield]: https://img.shields.io/github/stars/JdeRobot/bt-studio
[stars-url]: https://github.com/JdeRobot/bt-studio/stargazers
[issues-shield]: https://img.shields.io/github/issues/JdeRobot/bt-studio
[issues-url]: https://github.com/JdeRobot/bt-studio/issues
[license-shield]: https://img.shields.io/github/license/opensource.org/licenses/GPL-3.0
[license-url]: http://opensource.org/licenses/GPL-3.0
