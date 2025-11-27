from django.db import models
from django.contrib.postgres.fields import ArrayField
from django.utils import timezone
from django.contrib.auth.models import User

StatusChoice = (
    ("ACTIVE", "ACTIVE"),
    ("INACTIVE", "INACTIVE"),
    ("PROTOTYPE", "PROTOTYPE"),
)

UniverseType = (
    ("none", "None"),
    ("gazebo", "Gazebo"),
    ("gz", "Gazebo Harmonic"),
    ("physical", "Physical"),
)

RosVersion = (("ROS", "ROS"), ("ROS2", "ROS2"))


class Tool(models.Model):
    """
    Modelo Tool para Robotics Academy
    """

    name = models.CharField(max_length=50, blank=False, unique=True, primary_key=True)
    base_config = models.CharField(max_length=200, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"tools"'


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
    tools_config = models.CharField(max_length=200, blank=False)
    ros_version = models.CharField(max_length=4, choices=RosVersion, default="none")
    type = models.CharField(
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


class MyUser(User):
    size = -1
    max_size = -1

    projects = -1
    max_projects = -1

    def check_max_size(self, new_size, old_size=0):
        if self.max_size < 0:
            return True
        return (self.size + new_size - old_size) < self.max_size

    def update_size(self, fal, new_size, old_size=0, project_callback=None):
        if self.size < 0:
            if project_callback is not None:
                self.size += project_callback(self, fal)

        self.size += new_size - old_size
        self.save()


class Project(models.Model):
    """
    Project Model
    """

    id = models.SlugField(max_length=100, blank=False, unique=True, primary_key=True)
    name = models.CharField(max_length=100, blank=False, unique=True)
    creator = models.ForeignKey(MyUser, on_delete=models.CASCADE, db_column='"creator"')
    last_modified = models.DateTimeField(blank=False)
    size = models.BigIntegerField(default=-1, blank=False)

    def __str__(self):
        return str(self.name)

    class Meta:
        db_table = '"projects"'

    def update_size(self, fal, new_size=0, old_size=0):
        self.get_size(fal)
        self.size += new_size - old_size
        self.last_modified = timezone.now()
        self.save()

    def get_size(self, fal):
        if self.size < 0:
            self.size = fal.dir_size(fal.project_path(self.id, False))
            self.save()
        return self.size


def get_user_projects_size(user, fal):

    if user.projects < 0:
        user.projects = 0
        folder_path = fal.projects_path()
        old_projects = fal.listdirs(folder_path)
        for p in old_projects:
            Project.objects.create(
                id=p,
                name=p,
                creator=fal.user,
                last_modified=timezone.now(),
            )
            user.projects += 1
        user.save()

    size = 0
    for project in Project.objects.filter(creator=user):
        size += project.get_size(fal)

    return size
