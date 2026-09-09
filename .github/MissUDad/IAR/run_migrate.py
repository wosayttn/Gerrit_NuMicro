import os
import sys
import fnmatch
import traceback
import shutil
import subprocess
import xml.etree.ElementTree as ET
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

ROOT = missudad.PROJ_FOLDER_NAME
OUT = "list.txt"
TARGET_VERSION = "8.42.1.23678"

ewp_list = []

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
                if not version.startswith(TARGET_VERSION ):
                    print(PRJ_EWP,'-->',version )
                    ewp_list.append(PRJ_EWP)

def fix_version(PRJ_EWP):
    tree = ET.parse(PRJ_EWP)
    root = tree.getroot()
    updated = False

    for option in root.findall(".//option"):
        name_elem = option.find("name")
        state_elem = option.find("state")

        if name_elem is not None and name_elem.text == "OGLastSavedByProductVersion":
            if state_elem is not None:
                version = state_elem.text.strip()

                if version != TARGET_VERSION:
                    print(f"{PRJ_EWP} : {version}  →  {TARGET_VERSION}")
                    state_elem.text = TARGET_VERSION
                    updated = True

    if updated:
        tree.write(PRJ_EWP, encoding="UTF-8", xml_declaration=True)

if __name__ == "__main__":

    root = os.getcwd()
    os.chdir(root)
    for dirPath, dirNames, fileNames in os.walk(ROOT):
        for file in fnmatch.filter(fileNames, '*.ewp'):
            get_version(os.path.join(dirPath, file))

    with open(OUT, "w", encoding="utf-8") as f:
        for p in ewp_list:
            f.write(p + "\n")

    print(f"Found {len(ewp_list)} IAR projects")

    print(f"Run VBS")
    vbs_path = os.path.abspath(os.path.join(root, migrate_all.vbs))
    subprocess.run(["cscript.exe", vbs_path], check=True)

    os.chdir(root)
    for dirPath, dirNames, fileNames in os.walk(ROOT):
        for file in fnmatch.filter(fileNames, '*.ewp'):
            fix_version(os.path.join(dirPath, file))

    print(f"Migrate down.")
