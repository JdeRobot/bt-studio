#!/bin/bash

# Default branch if not specified
BT_STUDIO="main"
ROBOTICS_INFRASTRUCTURE="humble-devel"
RAM="humble-devel"
ROS_DISTRO="humble"
IMAGE_TAG="test"
FORCE_BUILD=false
FORCE_BUILD_NO_CACHE=false

Help()
{
   # Display Help
   echo "Syntax: build.sh [options]"
   echo "Options:"
   echo "  -h                        Print this Help."
   echo "  -f                        Force creation of the base image. If omitted, the base image is created only if "
   echo "                            it doesn't exist."
   echo "  -F                        Force creation of the base image without using docker cache."
   echo "  -bt,--bt-studio  <value>  Branch of Bt-Studio.                     Default: main"
   echo "  -i, --infra      <value>  Branch of RoboticsInfrastructure.        Default: humble-devel"
   echo "  -m, --ram        <value>  Branch of RoboticsApplicationManager.    Default: humble-devel"
   echo "  -r, --ros        <value>  ROS Distro (humble or noetic).           Default: humble"
   echo "  -t, --tag        <value>  Tag name of the image.                   Default: test"
   echo
   echo "Example:"
   echo "   ./build.sh -t my_image"
   echo "   ./build.sh -f -a master -i noetic-devel -m main -r noetic -t my_image" 
   echo "   ./build.sh -f --academy master --infra noetic-devel --ram main --ros noetic --tag my_image" 
   echo
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -bt | --bt-studio) 
            BT_STUDIO="$2"
            shift 2
            ;;
        -i | --infra)
            ROBOTICS_INFRASTRUCTURE="$2"
            shift 2
            ;;
        -m | --ram)
            RAM="$2"
            shift 2
            ;;
        -r | --ros)
            ROS_DISTRO="$2"
            shift 2
            ;;
        -t | --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        -f | --force)
            FORCE_BUILD=true
            shift
            ;;
        -F | --force-no-cache)
            FORCE_BUILD_NO_CACHE=true
            shift
            ;;
        -h | --help) # display Help
            echo "Generates Bt Studio image"
            echo
            Help
            exit 0
            ;;
        *)
            echo "Invalid Option: $1"
            Help
            exit 1
            ;;
   esac
done

echo "BT_STUDIO:-------------:$BT_STUDIO"
echo "ROBOTICS_INFRASTRUCTURE:------:$ROBOTICS_INFRASTRUCTURE"
echo "RAM:--------------------------:$RAM"
echo "ROS_DISTRO:-------------------:$ROS_DISTRO"
echo "IMAGE_TAG:--------------------:$IMAGE_TAG"
echo

# Determine Dockerfile based on ROS_DISTRO
if [[ $ROS_DISTRO == "humble" ]]; then
    DOCKERFILE_BASE="Dockerfile.dependencies_humble"
    DOCKERFILE="Dockerfile.humble"
else
    echo "Error: Unknown ROS_DISTRO ($ROS_DISTRO). Please set it to 'noetic' or 'humble'."
    exit 1
fi

if $FORCE_BUILD_NO_CACHE; then
  NO_CACHE="--no-cache"
else
  NO_CACHE=""
fi

# Build the Docker Base image
if $FORCE_BUILD_NO_CACHE || $FORCE_BUILD || [[ "$(docker images -q jderobot/robotics-applications:dependencies-$ROS_DISTRO 2> /dev/null)" == "" ]]; then
  echo "===================== BUILDING $ROS_DISTRO BASE IMAGE ====================="
  echo "Building base using $DOCKERFILE_BASE for ROS $ROS_DISTRO"
  docker build $NO_CACHE -f $DOCKERFILE_BASE -t jderobot/robotics-applications:dependencies-$ROS_DISTRO .
fi

if [ $? -eq 0 ]; then
    echo "Docker Base Image Build Successful"
else
    echo "Docker Base Image Build FAILED...exiting"
    exit
fi

# Build the Docker image
echo "===================== BUILDING $ROS_DISTRO RoboticsBackend ====================="
echo "Building RoboticsBackend using $DOCKERFILE for ROS $ROS_DISTRO"

docker build --no-cache -f $DOCKERFILE \
  --build-arg BT_STUDIO=$BT_STUDIO \
  --build-arg ROBOTICS_INFRASTRUCTURE=$ROBOTICS_INFRASTRUCTURE \
  --build-arg RAM=$RAM \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  --build-arg IMAGE_TAG=$IMAGE_TAG \
  -t jderobot/bt-studio:$IMAGE_TAG .
