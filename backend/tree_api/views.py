import os
from django.conf import settings
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .serializers import FileContentSerializer
from . import generate_app
from . import generate_tree
from django.http import HttpResponse
from django.http import JsonResponse
import mimetypes

@api_view(['GET'])
def get_file_list(request):
    
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    
    try:
        # List all files in the directory
        file_list = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
        
        # Return the list of files
        return Response({'file_list': file_list})
        
    except Exception as e:
        return Response({'error': f'An error occurred: {str(e)}'}, status=500)

@api_view(['GET'])
def get_file(request):

    filename = request.GET.get('filename', None)
    
    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    
    if filename:
        file_path = os.path.join(folder_path, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                content = f.read()
            serializer = FileContentSerializer({'content': content})
            return Response(serializer.data)
        else:
            return Response({'error': 'File not found'}, status=404)
    else:
        return Response({'error': 'Filename parameter is missing'}, status=400)

@api_view(['POST'])
def create_file(request):

    filename = request.data.get('filename')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    file_path = os.path.join(folder_path, filename)
    
    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            f.write('')  # Empty content
        return Response({'success': True})
    else:
        return Response({'success': False, 'message': 'File already exists'}, status=400)

@api_view(['POST'])
def delete_file(request):
    filename = request.data.get('filename')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    file_path = os.path.join(folder_path, filename)
    
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

    filename = request.data.get('filename')
    content = request.data.get('content')
    
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    file_path = os.path.join(folder_path, filename)

    try:
        with open(file_path, 'w') as f:
            f.write(content)
        return Response({'success': True})
    except Exception as e:
        return Response({'success': False, 'message': str(e)}, status=400)
    

@api_view(['GET'])
def download_app(request):

    # Get the app name
    app_name = request.GET.get('app_name', None)

    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    tree_path = os.path.join(folder_path, 'tree.xml')
    result_path = os.path.join(folder_path, '/tmp/self_contained_tree.xml')
    template_path = os.path.join(settings.BASE_DIR, 'ros_template')

    if app_name:

        try:
            generate_tree.generate(tree_path, folder_path, result_path)
            zip_file_path = generate_app.generate(result_path, app_name, template_path)

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
        return Response({'error': 'app_name parameter is missing'}, status=400)



    