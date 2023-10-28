from django.urls import path
from . import views

urlpatterns = [
    path('create_project/', views.create_project, name='create_project'),
    path('get_project_list/', views.get_project_list, name='get_project_list'),
    path('get_file_list/', views.get_file_list, name='get_file_list'),
    path('get_file/', views.get_file, name='get_file'),
    path('create_file/', views.create_file, name='create_file'),
    path('delete_file/', views.delete_file, name='delete_file'),
    path('save_file/', views.save_file, name='save_file'),
    path('translate_json/', views.translate_json, name='translate_json'),
    path('generate_app/', views.generate_app, name='generate_app')
]