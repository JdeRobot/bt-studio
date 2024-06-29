---
title: About
layout: posts
permalink: /about/

collection: posts

classes: wide

sidebar:
  nav: "docs"
---

# BT Studio

![alt_text]({{ "assets/images/logo.png" | absolute_url }})

[![License](http://img.shields.io/:license-gpl-blue.svg)](http://opensource.org/licenses/GPL-2.0)

BT Studio is an open-source tool crafted for the development of robotic applications. Its primary objective is to facilitate the quick deployment of behavior tree-based robotic applications within ROS 2. In BT Studio, a robotic app is defined as an graphical tree coupled with actions scripted in Python, which the tool then translates into a ROS 2 package. This process circumvents the unnecessary complexities often associated with ROS-specific configurations, offering developers a more streamlined approach.

### Supported Platforms
 * Python-3 + ROS-Humble
 * Might support future ROS Distributions



## Installation


* **GNU/Linux**

  -- Follow [Installation Guide](https://jderobot.github.io/bt-studio/install/)

* **Windows**

  Coming soon.  

* **Mac OS**
  Coming soon.
  

Check the [Documentation](https://jderobot.github.io/bt-studio/documentation/) for more information.

## Development

### Download

```bash
git clone https://github.com/JdeRobot/bt-studio.git
cd bt-studio
```

### Install

1. Install the necessary packages
```bash
sudo apt update
sudo apt install npm python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions python3-autopep8 -y
```

2. Install the backend dependencies
```bash
cd backend
pip install virtualenv
virtualenv backend_env
source backend_env/bin/activate
pip install django djangorestframework
```

3. Launch the backend

```bash
python3 manage.py runserver
```

**Do not close the terminal where this is executing!**. Django provides the necessary backend funcionalities. For continuing the process, simply open a new terminal. 

4. Install the appropiate nodejs version

```bash
wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
source ~/.bashrc
nvm install 16
nvm use 16
```

5. Install Yarn

```bash
sudo npm install --global yarn
```

6. Setup the frontend

From the bt_studio folder:

```bash
cd frontend
yarn install
yarn start
```

### Documentation



### Troubleshooting


## Roadmap

 We use the GitHub [issues](https://github.com/JdeRobot/bt-studio/issues) to track the work and schedule our new features and improvements.

## Development Team

* **Óscar Martínez Martínez**, creator [Github page](https://github.com/OscarMrZ)
* **Javier Izquierdo Hernández**, developer [Github page](https://github.com/javizqh)
* **Jose Maria Cañas**,concepts and development [Github page](https://github.com/jmplaza)


## Credits


## License

Licensed under [GPL 3.0](http://opensource.org/licenses/GPL-3.0).
