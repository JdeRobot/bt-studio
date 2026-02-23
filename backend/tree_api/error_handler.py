from rest_framework.response import Response
from rest_framework.decorators import api_view
from backend.tree_api.models import User, Project
import binascii
from functools import wraps
import json
from django.conf import settings
from .file_access import FAL_BT
from copy import copy
from .exceptions import (
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
)

CUSTOM_EXCEPTIONS = (
    ResourceNotExists,
    ResourceAlreadyExists,
    ParameterInvalid,
    InvalidPath,
)

local_fal = FAL_BT(settings.BASE_DIR)


def error_wrapper(type: str, param: list[str | tuple] = []):
    def decorated(func):
        @wraps(func)
        @api_view([type])
        def wrapper(request):
            try:
                fal = copy(local_fal)
                check_parameters(request.data if type == "POST" else request.GET, param)
                fal.set_user(User.objects.get(username="user"))

                # Set this to bypass user
                projects = Project.objects.all()
                if len(projects) > 0:
                    fal.user.projects = 0
                    for project in projects:
                        fal.user.projects += 1
                return func(fal, request)
            except CUSTOM_EXCEPTIONS as e:
                print(str(e))
                return Response({"error": f"{str(e)}"}, status=e.error_code)
            except json.JSONDecodeError as e:
                print(str(e))
                return Response({"error": f"Invalid JSON format: {str(e)}"}, status=422)
            except (binascii.Error, ValueError) as e:
                print(str(e))
                return Response({"error": f"Invalid B64 format: {str(e)}"}, status=422)
            except Exception as e:
                print(str(e))
                return Response({"error": f"An error occurred: {str(e)}"}, status=500)

        return wrapper

    return decorated


def check_parameters(request, param: list[str | tuple]):
    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data is None or len(data) <= min_len:
            raise ParameterInvalid(p)
