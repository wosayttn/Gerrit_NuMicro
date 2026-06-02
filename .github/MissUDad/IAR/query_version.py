import os
import sys
import fnmatch
import traceback
import xml.etree.ElementTree as ET
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST=missudad.IP_LIST

def get_version(PRJ_EWP):

    tree = ET.parse(PRJ_EWP)
    root = tree.getroot()

    # Search for <option> tags
    for option in root.findall(".//option"):
        name_elem = option.find("name")
        state_elem = option.find("state")
        if name_elem is not None and name_elem.text == "OGLastSavedByProductVersion":
            if state_elem is not None:
                version = state_elem.text.strip()
                if not version.startswith('8.4'):
                    print(PRJ_EWP,'-->',version )

if __name__ == "__main__":

    root = os.getcwd()
    os.chdir(root)
    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.ewp'):

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:
                get_version(os.path.join(dirPath, file))
