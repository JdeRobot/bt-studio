#!/bin/bash

# Initialize variables with default values
gpu_mode="false"
nvidia="false"
compose_file="dev_humble_cpu"

# Function to display help message
show_help() {
  echo "Options:"
  echo "  -g  Enable GPU mode (default: false)"
  echo "  -n  Enable Nvidia support (default: false)"
  echo "  -h  Display this help message"
}

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$nvidia" = "true" ]; then
    docker compose --compatibility down
  else
    docker compose down
  fi
  rm docker-compose.yaml
  
  exit 0
}

while getopts ":g:n:h" opt; do
  case $opt in
    g) gpu_mode="true" ;; 
    n) nvidia="true" ;;
    h) show_help; exit 0 ;;  # Display help message and exit
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

# Install docker-compose if not installed
if ! command -v docker-compose &> /dev/null; then
  sudo apt install docker-compose
fi

# Prepare nvm
export NVM_DIR=$HOME/.nvm;
source $NVM_DIR/nvm.sh;
if ! command -v nvm &> /dev/null; then
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
  export NVM_DIR=$HOME/.nvm;
  source $NVM_DIR/nvm.sh;
fi

# Prepare yarn 
if ! command -v yarn --version &> /dev/null; then
  npm install --global yarn
fi

# Prepare the frontend
nvm install 20
nvm use 20
cd frontend/
yarn install
yarn build
# Rename main files
cd build/static/js && for filename in main.*; do mv "$filename" "main.js""$(echo "$filename" | sed -e 's/main.*.js//g')"; done && cd ..
cd css && for filename in main.*; do mv "$filename" "main.css""$(echo "$filename" | sed -e 's/main.*.css//g')"; done && cd ../../..
cd ..

# Prepare the compose file
if [ "$gpu_mode" = "true" ]; then
  compose_file="dev_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  compose_file="dev_humble_nvidia"
fi
cp compose_cfg/$compose_file.yaml docker-compose.yaml

# Proceed with docker-compose commands
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi 
