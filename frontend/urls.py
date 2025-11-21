from django.urls import path
from django.shortcuts import redirect
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("create_project/", lambda request: redirect("/home", permanent=True)),
    path(
        "create_project/<slug:proj_id>",
        RedirectView.as_view(url="/home", permanent=True),
    ),
    path("studio/<slug:proj_id>/", RedirectView.as_view(url="/home", permanent=True)),
]
