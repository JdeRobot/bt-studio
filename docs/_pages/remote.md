---
permalink: /remote/

title: "Remote Use"

sidebar:
  nav: "docs"
---

# Instructions for using Robotics Backend

## Minimum System Requirements

* **CPU**: A 4-cored processor.
* **RAM**: 2 gb RAM.
* **Memory**: 20 gb of disk space.

## Linux Users

1. Download [Docker](https://docs.docker.com/get-docker/) **(minimum version of docker-py: 5.0.3)**.

2. Pull the current distribution of Robotics Backend **(currently version 4.6.18)**:

```bash
docker pull jderobot/robotics-backend:latest
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of Robotics Backend can be found [here](https://hub.docker.com/r/jderobot/robotics-backend/tags).

## Windows Users

Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

1. Install WSL2. Here's a link to the [tutorial](https://learn.microsoft.com/en-us/windows/wsl/install) **(minimum version of docker hub: 4.30.0)**.

2. Install Docker Desktop. Docker made an app for Windows users to adapt the user experience. You can download it from this [link](https://www.docker.com/products/docker-desktop/).

3. Enable Docker Desktop WSL integration: In order to wsl2 to recognise Docker, you need to enable it. For that, go to Docker Desktop -> Settings -> Resources -> WSL integration. Click on the check box and the slider.

    ![WSL integration](/RoboticsAcademy/assets/images/user_guide/wsl-integration-docker.png)

4. Pull the current distribution of Robotics Backend **(currently version 4.6.18)**:

```bash
docker pull jderobot/robotics-backend:latest
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of Robotics Backend can be found [here](https://hub.docker.com/r/jderobot/robotics-backend/tags).

## MacOs (NOT TESTED!)

* Remember to add minium docker version to run a Bt Studio.

<a name="launch"></a>
# 2. How to launch a Robotics Backend container?

* Start a new docker container of the image and keep it running in the background:
* The priority order is: NVIDIA -> Intel -> Only CPU

```bash
docker run --rm -it $(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "") --device /dev/dri -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-backend:latest
```

## Advanced Instructions on Linux

- **NVIDIA:** For NVIDIA GPUs, acceleration can be achieved by [installing the nvidia-container-runtime package](https://docs.docker.com/config/containers/resource_constraints/#gpu), and then running the **auto** command above.

- **Integrated GPU**
```bash
docker run --rm -it --device /dev/dri -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-backend:latest

```

- **Only CPU:** 
```bash
docker run --rm -it -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-backend:latest

```

## Windows
For Windows machines, acceleration can be achieved for NVIDIA GPUs if a valid CUDA installation is available. Useful docs for proper installation of WSL2 + CUDA + Docker Desktop:
- [WSL2 + CUDA](https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl)
- [WSL2 + Docker Desktop](https://docs.docker.com/desktop/features/wsl/)

Once these requirements are ready, you should be able to run Bt Studio with GPU acceleration as follows:
```bash
docker run --rm -it --gpus all -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --device /dev/dri -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 --link academy_db jderobot/robotics-backend::latest
```

### Tips for Docker

Make sure to remove the container if you have problems with the exercise not loading with the following commands.

1. Locate the docker container used for Robotics Backend using

```bash
docker ps -a
```

2. Remove that container using

```bash
docker rm CONTAINER_ID
```
