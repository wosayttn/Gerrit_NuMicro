import os
import sys
import subprocess
import shutil
import fnmatch
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

ECLIPSE_EXE=missudad.ECLIPSE_EXE
PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

if __name__ == "__main__":
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    err = 0
    root = os.getcwd()
    f = open('gcc.txt', "w")

    prj_count = 1

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.cproject'):

            basename = os.path.basename(os.path.dirname(dirPath))
            prjName = basename

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or prjName.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:

                if os.path.isdir('Temp'):
                    shutil.rmtree('Temp')

                os.mkdir('Temp')
                buildcommnd = ECLIPSE_EXE + " -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data Temp -import " + dirPath + " -build all"

                BUILDLOG = os.path.join(dirPath, basename) + '.log'

                try:
                    print(dirPath + " building ...")
                    fp = open(BUILDLOG, "w")
                    subprocess.check_call(buildcommnd, startupinfo=si, stdout=fp, stderr=fp)
                    fp.flush()
                    fp.close()

                    # Find any error/warning
                    fp = open(BUILDLOG, "r")
                    lines = fp.readlines()
                    fp.close()

                    # Skip 3 lines at first
                    # For ignore warning. OpenJDK 64-Bit Server VM warning: Options -Xverify:none and -noverify were deprecated in JDK 13 and will likely be removed in a future release.
                    lines.pop(0)
                    lines.pop(1)

                    found = 0
                    for line in lines:
                        if line.find('[Fatal Error]') >= 0 or line.find('An error has occurred.') >=0:
                            found = 1
                            break
                        elif line.find(" error: ") >= 0 or line.find(" warning: ") >= 0:
                            found = 1

                    if found == 1:
                        err += 1
                        f.write("[" + str(prj_count) + "] "+ dirPath +  " has error or warning.\n")
                        #print("Build " + basename + " has error or warning...\n")
                    #else:
                        #f.write("[" + str(prj_count) + "] "+ os.path.abspath(file) +  " pass...\n")
                        #print("\t" + basename +  " pass.\n")
                    pass
                except Exception as e:
                    f.write("[" + str(prj_count) + "] "+ dirPath +  " has Exception.\n")
                    print("Build" + file +  "has Exception.\n")
                    err += 1
                except OSError:
                    print("Build" + file +  "has Ooops...\n")
                    f.write("[" + str(prj_count) + "] "+ dirPath +  " has Ooops.\n")
                    err += 1
                    pass                # Silently ignore

                prj_count += 1
                f.flush()

    if os.path.isdir('Temp'):
        shutil.rmtree('Temp')

    if err == 0:
        f.write("Build " + str(prj_count-1) + " projects successfully.\n")

    f.close()

    if err == 0:
        sys.exit(0)
    else:
        sys.exit(1)
