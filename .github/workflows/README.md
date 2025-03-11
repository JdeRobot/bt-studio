# Actions

## Github Action _Release Bt Studio_

This action should be used when releasing a new Bt Studio version and the automatic one failed to do so on the new version release. **You should not use this action when creating a beta or testing image**. For that purpose use _Generate Bt Studio_ or _Generate Bt Studio Database_.

This github action is prepared to call the actions _Generate Bt Studio_ and _Generate Bt Studio Database_. This github action needs some inputs:

* Docker Image tag: The id of the tag of the Bt Studio image to pull from Dockerhub. It must be the version number __(ex: 5.4.1)__.
* Upload as latest: The id of the tag of the Bt Studio image is also set to latest to pull from Dockerhub. Uncheck only if testing the action.
* Branch of Bt Studio: The branch of Bt Studio repo that will be packaged. It must be _main_.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo that will be packaged. It's default value is _humble-devel_.
* Branch of RoboticsInfrastructure for Universes Database: The branch of RoboticsInfrastructure repo where the universes database is stored. It's default value is _database_.
* Branch of RoboticsApplicationManager: The branch of RoboticsApplicationManager repo that will be packaged. It's default value is _humble-devel_.
* ROS Distro: ROS version that will be used on the simulations. From now on Bt Studio and Unibotics-webserver only use __ROS 2 (humble)__.

## Github Action _Generate Bt Studio_

This github action is prepared to package every image created by the Dockerfiles into a Bt Studio tagged docker image. This github action needs some inputs:

* Docker Image tag: The id of the tag of the Bt Studio image to pull from Dockerhub. It is usually a the version number __(ex: 0.7.1)__ unless it's a beta image, which should be tagged as __beta__
* Upload as latest: The id of the tag of the Bt Studio image is also set to latest to pull from Dockerhub. Check only if the image is desired to be used as the latest reference.
* Branch of Bt Studio: The branch of Bt Studio repo that will be packaged. It's default value is _main_. Note that If you're creating a beta image of BT, you must specify the branch of Bt Studio which has the desired changes.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo that will be packaged. It's default value is _humble-devel_.
* Branch of RoboticsApplicationManager: The branch of RoboticsApplicationManager repo that will be packaged. It's default value is _humble-devel_.
* ROS Distro: ROS version that will be used on the simulations. From now on Bt Studio and Unibotics-webserver only use __ROS 2 (humble)__.

  ### How does this github action works?

  1. All the necessary setups for github action is prepared.
  2. The github action logs in the Dockerhub webpage using the __github actions secrets__.
  3. Then builds a base image with all the deppendencies used on robotics applications.
  4. Finally starts compiling and packaging every repo to create the Bt Studio image.
  5. New Bt Studio image is pushed into [Dockerhub](https://hub.docker.com/r/jderobot/bt-studio).

## Github Action _Generate Bt Studio Database_

This github action is prepared to package every image created by the Dockerfiles into a Bt Studio Database tagged docker image. This github action needs some inputs:

* Docker Image tag: The id of the tag of the Bt Studio Database image to pull from Dockerhub. It is usually a the version number __(ex: 0.7.1)__ unless it's a beta image, which should be tagged as __beta__
* Upload as latest: The id of the tag of the Bt Studio Database image is also set to latest to pull from Dockerhub. Check only if the image is desired to be used as the latest reference.
* Branch of Bt Studio: The branch of Bt Studio repo that will be used to retrieve the django_auth databse. It's default value is _main_. Note that If you're creating a beta image of BT, you must specify the branch of Bt Studio which has the desired changes.
* Branch of RoboticsInfrastructure: The branch of RoboticsInfrastructure repo where the universe database is stored. It's default value is _database_.

  ### How does this github action works?

  1. All the necessary setups for github action is prepared.
  2. The github action logs in the Dockerhub webpage using the __github actions secrets__.
  3. Then builds a base image with all the deppendencies used on robotics applications.
  4. Finally starts compiling and packaging every repo to create the Bt Studio Database image.
  5. New Bt Studio image is pushed into [Dockerhub](https://hub.docker.com/r/jderobot/bt-studio-database).
