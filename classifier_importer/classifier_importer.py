#!/usr/bin/env python3

import os
import re
import rclpy.logging

def replace_values(text,s_marker,e_marker,values,padding=''):

    text = re.sub(
        r'{}(\n|.)*?{}'.format(
            s_marker,
            e_marker
        ),
        s_marker+'\n'+values+'\n'+padding+e_marker,
        text
    )

    return text

def cimporter(classifiers=None):

        if classifiers == None or classifiers == []:
            print("Missing input arguments.")
            # TODO Insert url example
            print("The classifiers shoud be define using a launch file. Example: #url#")
            return
        
        print("Importing Classifiers...")

        t_imports = ""
        t_nodes = ""
        
        cpath = os.path.realpath(__file__)
        poccurrences = [i for i in range(len(cpath)) if cpath.startswith('/', i)]
        dpath = cpath[:poccurrences[-2]+1] + 'classifier_importer/classification_node_template.py'
        epath = cpath[:poccurrences[-2]+1] + 'src/classification_node.py'
        n_cl = len(classifiers)
        tab_4 = '    '
        for i in range(n_cl):
            if i == n_cl-1:
                t_imports += classifiers[i]['import_string']
                t_nodes+="{tab}node_list.append({class_name}({args}))".format(
                    tab=tab_4,
                    class_name=classifiers[i]['class_name'],
                    args=classifiers[i]['args']
                )
            else:
                t_imports += classifiers[i]['import_string'] + '\n'
                t_nodes+="{tab}node_list.append({class_name}({args}))\n".format(
                    tab=tab_4,
                    class_name=classifiers[i]['class_name'],
                    args=classifiers[i]['args']
                )


        with open(dpath, 'r') as file:
            file_data = file.read()

        file_data = replace_values(
            file_data,
            "### <import_classifiers> ###",
            "### </import_classifiers> ###",
            t_imports
        )

        file_data = replace_values(
            file_data,
            "### <append_classifiers> ###",
            "### </append_classifiers> ###",
            t_nodes,
            padding='\t'
        )

        with open(epath, 'w') as file:
            file_data = file.write(file_data)

        
        txtcls = ""
        for i in range(n_cl):
            txtcls+='\t'+classifiers[i]['class_name']+'\n'
        print("Classifiers imported: \n{}".format(txtcls))