import os
from django.conf import settings

def create_file_from_template(file_path, filename, template):
	
	templates_folder_path = os.path.join(settings.BASE_DIR, 'templates')
	template_path = os.path.join(templates_folder_path, template)

	replacements = {'ACTION': filename[:-3]}

	if template == 'empty':
		with open(file_path, 'w') as f:
			f.write('')  # Empty content
		return True
	elif template == 'action':
		with open(file_path, 'w') as f:
			with open(template_path,'r') as temp:
				for line in temp:
					for src, target in replacements.items():
						line = line.replace(src, target)
					f.write(line)
				return True
	elif template == 'io':
		with open(file_path, 'w') as f:
			with open(template_path,'r') as temp:
				for line in temp:
					for src, target in replacements.items():
						line = line.replace(src, target)
					f.write(line)
				return True
	else:
		return False