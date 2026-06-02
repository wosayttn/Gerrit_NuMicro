import os
import sys
import fnmatch
import xml.etree.ElementTree as ET
import fileinput

sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

PATH_SCRIPT = os.getcwd()
BLANK_PRJ_NAME='blank'
BLANK_EWD_PATH=os.path.join(PATH_SCRIPT , BLANK_PRJ_NAME+'.ewd')
BLANK_EWP_PATH=os.path.join(PATH_SCRIPT , BLANK_PRJ_NAME+'.ewp')

def ewp_group_copy(PRJ_EWP):
    global BLANK_EWP_PATH

    # Copy sources
    prj_ewp_tree = ET.parse(PRJ_EWP)
    prj_ewp_groups = prj_ewp_tree.findall('group')

    blank_ewp_tree = ET.parse(BLANK_EWP_PATH)
    blank_ewp_root = blank_ewp_tree.getroot()

    for group in prj_ewp_groups:
        #dupe = copy.deepcopy(group)
        blank_ewp_root.append(group)

    # Copy ICCARM/AARM/OBJCOPY in project to blank.
    blank_ewp_settings = blank_ewp_tree.findall('./configuration/settings')
    prj_ewp_settings = prj_ewp_tree.findall('./configuration/settings')

    blank_ewp_configuration = blank_ewp_tree.find('./configuration')
    for setting in blank_ewp_settings:
        name = setting.find('name')
        if name.text.find('ICCARM') >= 0 or name.text.find('AARM') >= 0 or name.text.find('OBJCOPY') >= 0:
            blank_ewp_configuration.remove(setting)
            for prj_setting in prj_ewp_settings:
                prj_name = prj_setting.find('name')
                if prj_name.text.find(name.text) == 0:
                    #dupe = copy.deepcopy(prj_setting)
                    blank_ewp_configuration.append(prj_setting)

    # Restore blank to project.
    blank_ewp_tree.write(PRJ_EWP, encoding='UTF-8',  xml_declaration=True)

def ewp_outfile_rename(PRJ_EWP, prjName):
    # Find Project.bin and Project.out
    with fileinput.FileInput(PRJ_EWP, inplace = True) as f:
        for line in f:
            if line.find('<state>Project.bin</state>') >= 0:
                print('                    <state>'+prjName+'.bin</state>',end ='\n')
            elif line.find('<state>Project.out</state>') >= 0:
                print('                    <state>'+prjName+'.out</state>',end ='\n')
            else:
                print(line, end ='')
        f.close()

def ewd_copy(PRJ_EWD):
    global BLANK_EWD_PATH

    blank_ewd_tree = ET.parse(BLANK_EWD_PATH)
    blank_ewd_root = blank_ewd_tree.getroot()
    blank_ewd_tree.write(PRJ_EWD, encoding='UTF-8',  xml_declaration=True)

if __name__ == "__main__":
    root = os.getcwd()
    os.chdir(root)

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.ewp'):

            runit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    runit = 1
                    break

            if runit == 1:
                curName = os.path.splitext(file)[0]
                print(dirPath + '/' +file + 'upgrading ...')
                os.chdir(dirPath)
                ewp_group_copy(file)
                ewp_outfile_rename(file, curName)
                ewd_copy(curName+'.ewd')
                os.chdir(root)
