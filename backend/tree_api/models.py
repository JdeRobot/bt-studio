from django.db import models

# Create your models here.

import json
from django.db import models
import subprocess

StatusChoice = (
    ('ACTIVE', "ACTIVE"),
    ('INACTIVE', "INACTIVE"),
    ('PROTOTYPE', "PROTOTYPE")
)

VisualizationType = (
    ('none', "None"),
    ('console', "Console"),
    ('gazebo_gra', "Gazebo GRA"),
    ('gazebo_rae', "Gazebo RAE"),
    ('physic_gra', "Physic GRA"),
    ('physic_rae', "Physic RAE")
)

UniverseType = (
    ('none', "None"),
    ('gazebo', "Gazebo"),
    ('drones', "Gazebo Drones"),
    ('physical', "Physical")
)

RosVersion = (
    ('ROS1', "ROS1"),
    ('ROS2', "ROS2")
)


class Universe(models.Model):
    """
    Modelo Universe para RoboticsCademy
    """
    name = models.CharField(max_length=100, blank=False, unique=True)
    launch_file_path = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(
        max_length=4, choices=RosVersion, default="none")
    visualization = models.CharField(
        max_length=50,
        choices=VisualizationType,
        default="none",
        blank=False
    )
    world = models.CharField(
        max_length=50,
        choices=UniverseType,
        default="none",
        blank=False
    )

    def __str__(self):
        return str(self.name)
    
    class Meta:
        db_table = '"universes"'