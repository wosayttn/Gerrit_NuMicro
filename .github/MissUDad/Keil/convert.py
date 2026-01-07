import os
import sys
import subprocess
import fnmatch
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST
UV4_EXE=missudad.UV4_EXE

if __name__ == "__main__":
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    err = 0
    root = os.getcwd()
    f = open('uv4_convert.txt', "w+")

    os.chdir(root)

    UV2CSOLUTION_EXE=os.path.join(root, 'uv2csolution.exe')

    prj_count = 1

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.uvproj'):

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:
                os.chdir(dirPath)
                try:

                    BUILDLOG = file + ".covert.log"
                    covertcommnd = UV4_EXE + " -b -5 " + file + " -l " + BUILDLOG
                
                    # https://www.keil.com/support/man/docs/uv4cl/uv4cl_commandline.htm
                    # -j0   Hides the µVision GUI. Messages are suppressed. Use this option for batch testing.
                    # -z    Re-builds all targets of a project or multiple-project.
                    #       Ensure that each target has another object output folder.
                    #       Use the menu Projects - Options for Target - Output - Select Folder for Objects.
                    # -b    Builds the last current target of a project and exits after the build process finished.
                    #       Refer to option -t to change the target.
                    #       For multi-projects, the command builds the targets as defined in the dialog Project - Batch Build.
                    # -o outputfile
                    #f.write("[" + str(prj_count) + "] Build " + os.path.abspath(file) +  "\n")

                    print("[" + str(prj_count) + "] "+ os.getcwd() + "\\" + file +  " coverting.\n")
                    p = subprocess.Popen(covertcommnd, startupinfo=si, stdout=f, stderr=f)
                    p.wait(30)

                except subprocess.TimeoutExpired:
                    p.kill()
                    err += 1

                except Exception as e:
                    err += 1

                except OSError:
                    pass #Silently ignore

                os.chdir(root)
                prj_count += 1

    f.write("Converted " + str(prj_count-1) + " projects successfully.\n")
    f.close()

    os.chdir(root)

    if err == 0:
        sys.exit(0)
    else:
        sys.exit(1)
