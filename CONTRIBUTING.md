<a href="https://mmg-ai.com/en/"><img src="https://jderobot.github.io/assets/images/logo.png" width="100 " align="right" /></a>

# Contributing to Bt Studio

First off, thanks for your interest in contributing to Bt Studio! All contributors are welcome, from commenting issues to reviewing or sending Pull Requests.

## How to contribute?

If you are new to GitHub, visit the [first-contributions instructions](https://github.com/firstcontributions/first-contributions/blob/master/README.md) to learn how to contribute on GitHub.

To find issues you can help with, go though the list of [good first issues](https://github.com/JdeRobot/bt-studio/issues?q=label%3A%22good+first+issue%22+is%3Aopen) or issues labeled with [help wanted](https://github.com/JdeRobot/bt-studio/issues?q=label%3A%22help+wanted%22+is%3Aopen).

Once found or created an issue, let us know that you want to work on it by commenting in the issue.

## Launching Bt Studio

To launch Bt Studio you must choose between this 2 ways. We recommend to just use the first one, because it will automatically update Bt Studio and the databases to the ones found locally on your machine.

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

## Creating a custom BTDI (Bt Studio Docker Image)

If you need to create a custom docker image instead of using the ones found in dockerhub you must use the next script.

You may need to use this if you have to use a specific Robotics Infrastructure or Robotics Application Manager branch.

### Usage

1. **Navigate to the scripts directory**

    ```bash
    cd /scripts/RADI
    ```

2. **Build the Docker image**

    Run the script using the following command:

    ```bash
    ./build.sh -bt [BT_STUDIO] -i [ROBOTICS_INFRASTRUCTURE] -m [RAM] -r [ROS_DISTRO] -t [IMAGE_TAG]

    ```

    Each of the parameters is explained below:

`BT_STUDIO`: This is the branch name of the Bt Studio repository to use. Default value is main.

`ROBOTICS_INFRASTRUCTURE`: This is the branch name of the Robotics Infrastructure repository to use. Default value is humble-devel.

`RAM`: This is the branch name of the RoboticsApplicationManager repository to use. Default value is humble-devel.

`ROS_DISTRO`: This is the ROS distribution to use. The script currently supports `humble`. Default value is humble.

`IMAGE_TAG`: This is the tag of the Docker image that will be created. Default value is `test`.

## Questions, suggestions or new ideas

Please don't open an issue to ask a question or suggestion. Use the [GitHub Discussions](https://github.com/JdeRobot/bt-studio/discussions) which are meant to it. New ideas and enhacements are also welcome as discussion posts.

## Issue reporting

Feel free to [create a new issue](https://github.com/JdeRobot/bt-studio/issues/new) if you have some issue to report. But first, make sure that the issue has not been reported yet.

Be sure to explain in details the context and the outcome that you are lookign for. If reporting bugs, provide basic information like you OS version, Bt Studio docker version and Bt Studio version.


## Submitting changes

* Please open a Pull Request with a clear description of what it contains.
* If there is no Issue related to what the Pull Request solves make sure to create ones and link them with the PR.
* Always write a clear log message for your commits.
* Make sure that the code is formated properly so that the actions executed when uploading a commit end succesfully. If the code is not formatted properly it will not be merged. See more in the **Formatting** section.

## Coding conventions

Try to make the code as readable as possible. Also follow the next things:

### Frontend
  
* All code must be in TypeScript and typed as much as possible.
* Formated using prettier. To format execute `yarn format` inside the frontend directory.
* All calls to the backend must be found inside the `frontend/src/api_helper/TreeWrapper.ts` file and following the schema found inside.
* Follow as much as possible the [React Guidelines](https://react.dev/reference/rules).

### Backend

* Uses Django and it is found inside the `backend/tree_api/` folder.
* Formatted using Black.

Thanks! :heart: :heart:
Bt Studio Team
