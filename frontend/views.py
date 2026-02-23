from django.shortcuts import render


def index(request, proj_id=None):
    return render(request, "frontend/index.html")
