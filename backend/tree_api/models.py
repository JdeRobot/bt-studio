from django.db import models
from django.contrib.postgres.fields import ArrayField

StatusChoice = (
    ("ACTIVE", "ACTIVE"),
    ("INACTIVE", "INACTIVE"),
    ("PROTOTYPE", "PROTOTYPE"),
)

VisualizationType = (
    ("none", "None"),
    ("console", "Console"),
    ("gazebo_gra", "Gazebo GRA"),
    ("gazebo_rae", "Gazebo RAE"),
    ("gzsim_gra", "Gz Sim GRA"),
    ("gzsim_rae", "Gz Sim RAE"),
    ("physic_gra", "Physic GRA"),
    ("physic_rae", "Physic RAE"),
)

UniverseType = (
    ("none", "None"),
    ("gazebo", "Gazebo"),
    ("drones", "Gazebo Drones"),
    ("physical", "Physical"),
)

RosVersion = (("ROS", "ROS"), ("ROS2", "ROS2"))


class Robot(models.Model):
    """
    Modelo Robot para RoboticsAcademy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"robots"'


class World(models.Model):
    """
    Modelo World para RoboticsCademy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)
    visualization_config_path = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    visualization = models.CharField(
        max_length=50, choices=VisualizationType, default="none", blank=False
    )
    world = models.CharField(
        max_length=50, choices=UniverseType, default="none", blank=False
    )

    start_pose = ArrayField(
        ArrayField(
            models.DecimalField(
                decimal_places=4, max_digits=10, default=None, blank=False
            )
        )
    )

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"worlds"'


class Universe(models.Model):
    """
    Modelo Universe para Robotics Academy
    """

    name = models.CharField(max_length=100, blank=False, unique=True)
    world = models.OneToOneField(
        World, default=None, on_delete=models.CASCADE, db_column='"world_id"'
    )
    robot = models.OneToOneField(
        Robot, default=None, on_delete=models.CASCADE, db_column='"robot_id"'
    )

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"universes"'
