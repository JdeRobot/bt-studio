---
permalink: /install/

title: "Installation"

sidebar:
  nav: "docs"
---

## Current Dependencies

##### Python 3.8 and above, pip3 and NPM:

```
sudo apt install npm python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions python3-autopep8 -y
```


## Setup


### Download Direct .zip Package:

You can download the .zip package from [here](https://github.com/JdeRobot/bt-studio/releases)


### Alternative Method:

1. Clone the repo and enter it
```bash
git clone https://github.com/JdeRobot/bt-studio
cd bt-studio
```

2. Install the necessary packages
```bash
sudo apt update
sudo apt install npm python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions python3-autopep8 -y
```

4. Install the backend dependencies
```bash
cd backend
pip install virtualenv
virtualenv backend_env
source backend_env/bin/activate
pip install django djangorestframework
```

5. Launch the backend

```bash
python3 manage.py runserver
```

**Do not close the terminal where this is executing!**. Django provides the necessary backend funcionalities. For continuing the process, simply open a new terminal. 

6. Install the appropiate nodejs version

```bash
wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
source ~/.bashrc
nvm install 16
nvm use 16
```

7. Install Yarn

```bash
sudo npm install --global yarn
```

8. Setup the frontend

From the bt_studio folder:

```bash
cd frontend
yarn install
yarn start
```

A new window in the browser should have opened. If not, go to [BT Studio](http://localhost:3000/)


## Running the Tool:

Remember to launch both the backend and the frontend in two different terminals as explained above