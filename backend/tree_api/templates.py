import os
from django.conf import settings


def get_action_template(filename, template, template_path):

    with open(template_path, "r") as temp:
        file_data = temp.read()
        new_data = file_data.replace("ACTION", filename[:-3])
        return new_data
    return ""
