import os
import sys
import subprocess
import fnmatch
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

if __name__ == "__main__":
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    err = 0
    root = os.getcwd()
    f = open('uv2solution.txt', "w+")

    os.chdir(root)

    UV2CSOLUTION_EXE=os.path.join(root, 'uv2csolution.exe')

    prj_count = 1

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.uvprojx'):

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:
                os.chdir(dirPath)
                try:

                    UVPrjPath = os.path.abspath(os.getcwd())
                    command = UV2CSOLUTION_EXE + ' ' + os.path.join(UVPrjPath, file)
                    p = subprocess.Popen(command, startupinfo=si, stdout=f, stderr=f)
                    p.wait(30)

                    CSolutionPrjPath = os.path.join(os.path.dirname(UVPrjPath), 'VSCode')
                    PrjName = file.split('.')[0]
                    cproject_file = PrjName + '.cproject.yml'
                    csolution_file = PrjName + '.csolution.yml'
                    vcpkg_configuration = 'vcpkg-configuration.json'

                    missudad.move_file(os.path.join(UVPrjPath, cproject_file), os.path.join(CSolutionPrjPath, cproject_file))
                    missudad.move_file(os.path.join(UVPrjPath, csolution_file), os.path.join(CSolutionPrjPath, csolution_file))
                    missudad.move_file(os.path.join(UVPrjPath, vcpkg_configuration), os.path.join(CSolutionPrjPath, vcpkg_configuration))

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
