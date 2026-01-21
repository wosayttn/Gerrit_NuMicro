import os
import sys
import fnmatch
import xml.etree.ElementTree as ET
import fileinput
from shutil import copyfile
import copy

sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

PATH_SCRIPT = os.getcwd()
BLANK_PRJ_NAME='blank'
BLANK_UVOPTX_PATH=os.path.join(PATH_SCRIPT , BLANK_PRJ_NAME+'.uvoptx')
BLANK_UVOPROJX_PATH=os.path.join(PATH_SCRIPT , BLANK_PRJ_NAME+'.uvprojx')

def uvprojx_get(blank, path_parent, tag_name):

    # Locate Parent
    TParent = blank.find(path_parent)
    if TParent is None:
        raise RuntimeError("TParent section not found")

    # Locate Tag under Parent
    TTag = TParent.find(tag_name)
    if TTag is None:
        raise RuntimeError("TTag not found inside Cads")

    # Return value
    return TTag.text

def uvprojx_sync(prj, blank, path_parent, tag_name):

    blank_value = uvprojx_get(blank, path_parent, tag_name)

    # Locate Parent
    TParent = prj.find(path_parent)
    if TParent is None:
        raise RuntimeError("TParent section not found")

    # Locate Tag under Parent
    TTag = TParent.find(tag_name)
    if TTag is None:
        raise RuntimeError("TTag not found inside Cads")

    # Modify value
    old = TTag.text
    TTag.text = str(blank_value)

if __name__ == "__main__":
    root = os.getcwd()
    os.chdir(root)

    blank_tree = ET.parse(BLANK_UVOPROJX_PATH)
    blank_tree_root = blank_tree.getroot()

    blank_tree_target = blank_tree_root.find("./Targets/Target")
    if blank_tree_target is None:
        print("[WARN] No blank_tree_target.<Target> found")
        exit (1)

    prj_counter=0

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.uvprojx'):

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:

                curName = os.path.splitext(file)[0]
                print(dirPath + '/' + file + ' upgrading ...')
                os.chdir(dirPath)

                tree = ET.parse(file)
                prj_tree_root = tree.getroot()

                prj_tree_targets = prj_tree_root.findall("./Targets/Target")
                if not prj_tree_targets:
                    print("[WARN] No prj_tree_targets.<Target> found")
                    exit (1)


                for t in prj_tree_targets:
                    uvprojx_sync(t, blank_tree_target, 'TargetOption/TargetCommonOption', 'PackID')
                    uvprojx_sync(t, blank_tree_target, 'TargetOption/TargetCommonOption', 'Device')
                    uvprojx_sync(t, blank_tree_target, 'TargetOption/TargetCommonOption', 'RegisterFile')
                    uvprojx_sync(t, blank_tree_target, 'TargetOption/TargetCommonOption', 'SFDFile')
                    uvprojx_sync(t, blank_tree_target, 'TargetOption/TargetArmAds/Cads', 'wLevel')
                    uvprojx_sync(t, blank_tree_target, '.', 'pCCUsed')

                xml_str = ET.tostring(prj_tree_root, encoding='utf-8', short_empty_elements=False)
                with open(file, 'wb') as f:
                    f.write(b'<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n')
                    f.write(xml_str)

                copyfile( os.path.join(root, 'Nu_Link_Driver.ini'), os.path.join(os.getcwd(), 'Nu_Link_Driver.ini') )
                os.chdir(root)

                prj_counter=prj_counter+1

    print(str(prj_counter) + ' project(s) upgraded')
