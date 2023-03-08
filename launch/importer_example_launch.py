import launch

from classifier_importer.classifier_importer import cimporter

def generate_launch_description():

    # Classifier importer
	classifiers = []
	classifiers.append({
	    "import_string": "from smap_classifiers.classifier_1 import classifier_1",
	    "class_name": "classifier_1",
	    "args": "\'yolov1\'"
	})
	classifiers.append({
	    "import_string": "from smap_classifiers.classifier_2 import classifier_2",
	    "class_name": "classifier_2",
	    "args": "\'yolov2\'"
	})

	cimporter(classifiers)
	
	return launch.LaunchDescription([
        #im
    ])