import os
import sys
import subprocess
import fnmatch
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

PATH_ARM_CMSIS_SOLUTION_VSCODE_TEMPLATE='arm_cmsis_solution_1.54_.vscode'

if __name__ == "__main__":
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    err = 0
    root = os.getcwd()
    os.chdir(root)

    f = open('update.vscode.txt', "w")

    prj_count = 1
    
    PATH_ARM_CMSIS_SOLUTION_VSCODE_TEMPLATE=os.path.join(root, PATH_ARM_CMSIS_SOLUTION_VSCODE_TEMPLATE)

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, 'vcpkg-configuration.json'):

            CSolPrjPath = os.path.abspath(dirPath)
            CSolPrjPath_DotVScode = os.path.join(CSolPrjPath, '.vscode')

            try:

                missudad.mirror_file(os.path.join(PATH_ARM_CMSIS_SOLUTION_VSCODE_TEMPLATE, 'launch.json'), os.path.join(CSolPrjPath_DotVScode, 'launch.json'))
                missudad.mirror_file(os.path.join(PATH_ARM_CMSIS_SOLUTION_VSCODE_TEMPLATE, 'tasks.json'), os.path.join(CSolPrjPath_DotVScode, 'tasks.json'))

                missudad.remove_file(os.path.join(CSolPrjPath_DotVScode, 'settings.json'))

            except subprocess.TimeoutExpired:
                err += 1

            except Exception as e:
                err += 1

            except OSError:
                pass #Silently ignore

            prj_count += 1

    f.write("Update " + str(prj_count-1) + " projects successfully.\n")
    f.close()

    os.chdir(root)

    if err == 0:
        sys.exit(0)
    else:
        sys.exit(1)
