from rest_framework.response import Response
import binascii
from functools import wraps
import json
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


def error_wrapper(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except CUSTOM_EXCEPTIONS as e:
            return Response({"error": f"{str(e)}"}, status=e.error_code)
        except json.JSONDecodeError as e:
            return Response({"error": f"Invalid JSON format: {str(e)}"}, status=422)
        except (binascii.Error, ValueError) as e:
            return Response({"error": f"Invalid B64 format: {str(e)}"}, status=422)
        except Exception as e:
            return Response({"error": f"An error occurred: {str(e)}"}, status=500)

    return decorated

def check_post_parameters(request, param: list[str | tuple]):
    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data == None or len(data) <= min_len:
            raise ParameterInvalid(p)


def check_get_parameters(request, param: list[str | tuple]):
    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data == None or len(data) <= min_len:
            raise ParameterInvalid(p)
