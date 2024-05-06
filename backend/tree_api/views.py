import os
from django.conf import settings
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .serializers import FileContentSerializer
from . import app_generator
from . import tree_generator
from . import json_translator
from django.http import HttpResponse
from django.http import JsonResponse
import mimetypes
import json
import shutil
import zipfile

@api_view(['GET'])
def create_project(request):

    project_name = request.GET.get('project_name')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    
    if not os.path.exists(project_path):
        os.mkdir(project_path)
        os.mkdir(action_path)
        return Response({'success': True})
    else:
        return Response({'success': False, 'message': 'Project already exists'}, status=400)

@api_view(['GET'])
def get_project_list(request):

    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    
    try:
        # List all folders in the directory
        project_list = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]
        
        # Return the list of projects
        return Response({'project_list': project_list})
        
    except Exception as e:
        return Response({'error': f'An error occurred: {str(e)}'}, status=500)
    
@api_view(['POST'])
def save_project(request):

    # Get the app name and the graph
    project_name = request.data.get('project_name')
    graph_json = request.data.get('graph_json')

    # Generate the paths
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, project_name)
    graph_path = os.path.join(project_path, "graph.json")

    if project_path and graph_json:

        try:
            # Obtain pretty json
            graph = json.loads(graph_json)
            graph_formated = json.dumps(graph, indent=4)

            with open(graph_path, 'w') as f:
                f.write(graph_formated)

            return JsonResponse({'success': True})
        
        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error deleting file: {str(e)}'}, status=500)
    else:
        return Response({'error': 'app_name parameter is missing'}, status=400)

@api_view(['GET'])
def get_project_graph(request):

    project_name = request.GET.get('project_name')

    # Generate the paths
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, project_name)
    graph_path = os.path.join(project_path, "graph.json")

    # Check if the project exists
    if os.path.exists(graph_path):
        try:
            with open(graph_path, 'r') as f:
                graph_data = json.load(f)
            return JsonResponse({'success': True, 'graph_json': graph_data})
        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error reading file: {str(e)}'}, status=500)
    else:
        return Response({'error': 'The project does not have a graph definition'}, status=404)


@api_view(['GET'])
def get_file_list(request):
    
    project_name = request.GET.get('project_name')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')

    try:
        # List all files in the directory
        file_list = [f for f in os.listdir(action_path) if os.path.isfile(os.path.join(action_path, f))]
        
        # Return the list of files
        return Response({'file_list': file_list})
        
    except Exception as e:
        return Response({'error': f'An error occurred: {str(e)}'}, status=500)

@api_view(['GET'])
def get_file(request):

    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)
    
    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    
    if filename:
        file_path = os.path.join(action_path, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                content = f.read()
            serializer = FileContentSerializer({'content': content})
            return Response(serializer.data)
        else:
            return Response({'error': 'File not found'}, status=404)
    else:
        return Response({'error': 'Filename parameter is missing'}, status=400)

@api_view(['GET'])
def create_file(request):

    # Get the file info
    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)
    template = request.GET.get('template', None)
    print(template)
    
    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    templates_folder_path = os.path.join(settings.BASE_DIR, 'templates')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)
    template_path = os.path.join(templates_folder_path, template)

    replacements = {'ACTION': filename[:-3]}
    
    if not os.path.exists(file_path):
        if template == 'empty':
            with open(file_path, 'w') as f:
                f.write('')  # Empty content
            return Response({'success': True})
        elif template == 'action':
            with open(file_path, 'w') as f:
                with open(template_path,'r') as temp:
                    for line in temp:
                        for src, target in replacements.items():
                            line = line.replace(src, target)
                        f.write(line)
                    return Response({'success': True})
        else:
            return Response({'success': False, 'message': 'Template does not exist'}, status=400)
    else:
        return Response({'success': False, 'message': 'File already exists'}, status=400)

@api_view(['GET'])
def delete_file(request):
    
    # Get the file info
    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)
    
    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)
    
    if os.path.exists(file_path):
        try:
            os.remove(file_path)
            return JsonResponse({'success': True})
        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error deleting file: {str(e)}'}, status=500)
    else:
        return JsonResponse({'success': False, 'message': 'File does not exist'}, status=404)

@api_view(['POST'])
def save_file(request):

    project_name = request.data.get('project_name')
    filename = request.data.get('filename')
    content = request.data.get('content')
    
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)

    try:
        with open(file_path, 'w') as f:
            f.write(content)
        return Response({'success': True})
    except Exception as e:
        return Response({'success': False, 'message': str(e)}, status=400)

@api_view(['POST'])
def translate_json(request):

    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')

    try:
        content = request.data.get('content')
        if content is None:
            return Response({'success': False, 'message': 'Content is missing'}, status=400)
        
        # Pass the JSON content to the translate function
        json_translator.translate(content, folder_path + "/tree.xml")
        
        return Response({'success': True})
    except Exception as e:
        return Response({'success': False, 'message': str(e)}, status=400)

@api_view(['POST'])
def generate_app(request):

    # Get the app name
    app_name = request.data.get('app_name')
    content = request.data.get('content')

    # Make folder path relative to Django app
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, app_name)
    action_path = os.path.join(project_path, 'actions')
    tree_path = os.path.join('/tmp/tree.xml')
    self_contained_tree_path = os.path.join('/tmp/self_contained_tree.xml')
    template_path = os.path.join(settings.BASE_DIR, 'ros_template')
    tree_gardener_src = os.path.join(settings.BASE_DIR, 'tree_gardener')

    if app_name and content:

        try:
            # Generate a basic tree from the JSON definition 
            json_translator.translate(content, tree_path)

            # Generate a self-contained tree 
            tree_generator.generate(tree_path, action_path, self_contained_tree_path)

            # Using the self-contained tree, package the ROS 2 app
            zip_file_path = app_generator.generate(self_contained_tree_path, app_name, template_path, action_path, tree_gardener_src)

            # Confirm ZIP file exists
            if not os.path.exists(zip_file_path):
                return Response({'success': False, 'message': 'ZIP file not found'}, status=400)

            # Prepare file response
            zip_file = open(zip_file_path, 'rb')
            mime_type, _ = mimetypes.guess_type(zip_file_path)
            response = HttpResponse(zip_file, content_type=mime_type)
            response['Content-Disposition'] = f'attachment; filename={os.path.basename(zip_file_path)}'

            return response

        except Exception as e:
            return Response({'success': False, 'message': str(e)}, status=400)
    else:
        return Response({'error': 'app_name parameter is missing'}, status=500)

@api_view(['POST'])
def get_simplified_app(request):

    # Get the app name
    app_name = request.data.get('app_name')
    tree_graph = request.data.get('content')

    # Make folder path relative to Django app
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, app_name)
    action_path = os.path.join(project_path, 'actions')

    working_folder = "/tmp/wf"
    tree_path = "/tmp/tree.xml"
    self_contained_tree_path = os.path.join(working_folder, 'self_contained_tree.xml')
    tree_gardener_src = os.path.join(settings.BASE_DIR, 'tree_gardener')
    template_path = os.path.join(settings.BASE_DIR, 'ros_template')

    if app_name and tree_graph:

        try:
            # 1. Create the working folder
            if os.path.exists(working_folder):
                shutil.rmtree(working_folder)
            os.mkdir(working_folder)

            # 2. Generate a basic tree from the JSON definition 
            json_translator.translate(tree_graph, tree_path)

            # 3. Generate a self-contained tree 
            tree_generator.generate(tree_path, action_path, self_contained_tree_path)

            # 4. Copy necessary files from tree_gardener and ros_template
            factory_location = tree_gardener_src + "/tree_gardener/tree_factory.py"
            tools_location = tree_gardener_src + "/tree_gardener/tree_tools.py"
            entrypoint_location = template_path + "/ros_template/execute_docker.py"
            shutil.copy(factory_location, working_folder)
            shutil.copy(tools_location, working_folder)
            shutil.copy(entrypoint_location, working_folder)

            # 5. Generate the zip
            zip_path = working_folder + ".zip"
            with zipfile.ZipFile(zip_path, 'w') as zipf:
                for root, dirs, files in os.walk(working_folder):
                    for file in files:
                        zipf.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), working_folder))

            # 6. Return the zip
            zip_file = open(zip_path, 'rb')
            mime_type, _ = mimetypes.guess_type(zip_path)
            response = HttpResponse(zip_file, content_type=mime_type)
            response['Content-Disposition'] = f'attachment; filename={os.path.basename(zip_path)}'         

            return response
        except Exception as e:
            return Response({'success': False, 'message': str(e)}, status=400)
    else:
        return Response({'error': 'app_name parameter is missing'}, status=500)
    