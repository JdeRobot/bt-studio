from django.shortcuts import render
from django.views.decorators.cache import never_cache


@never_cache
def index(request, proj_id=None):
    return render(request, "frontend/index.html")
