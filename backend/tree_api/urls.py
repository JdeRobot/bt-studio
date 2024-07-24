from django.urls import path
from . import views

urlpatterns = [
    path('create_project/', views.create_project, name='create_project'),
    path('delete_project/', views.delete_project, name='delete_project'),
    path('delete_universe/', views.delete_universe, name='delete_universe'),
    path('save_project/', views.save_project, name='save_project'),
    path('get_project_graph/', views.get_project_graph, name='get_project_graph'),
    path('get_project_list/', views.get_project_list, name='get_project_list'),
    path('get_file_list/', views.get_file_list, name='get_file_list'),
    path('get_universes_list/', views.get_universes_list, name='get_universes_list'),
    path('get_file/', views.get_file, name='get_file'),
    path('create_file/', views.create_file, name='create_file'),
    path('delete_file/', views.delete_file, name='delete_file'),
    path('save_file/', views.save_file, name='save_file'),
    path('translate_json/', views.translate_json, name='translate_json'),
    path('get_universe_configuration/', views.get_universe_configuration, name='get_universe_configuration'),
    path('generate_app/', views.generate_app, name='generate_app'),
    path('get_simplified_app/', views.get_simplified_app, name='generate_app'),
    path('get_simplified_universe/', views.get_simplified_universe, name='generate_app'),
]