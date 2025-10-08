import os
import sys
import subprocess
import fnmatch
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST=missudad.IP_LIST
IARBUILD_EXE=missudad.IARBUILD_EXE

if __name__ == "__main__":

    si = subprocess.STARTUPINFO()

    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    err = 0

    root = os.getcwd()
    f = open('failed_logs.txt', "w+")
    os.chdir(root)

    prj_count = 1

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.ewp'):

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:
                os.chdir(dirPath)
                try:
                    BUILDLOG = file + ".log"
                    fp = open(BUILDLOG, "w")

                    prjName = os.path.splitext(file)[0]
                    buildcommnd = IARBUILD_EXE + " " + file + " -build * -log warnings"
                    subprocess.call(buildcommnd, startupinfo=si, stdout=fp, stderr=fp)
                    fp.flush()
                    fp.close()

                    # Find any error/warning
                    tmp = open(BUILDLOG, "r")
                    lines = tmp.readlines()
                    tmp.close()

                    found = 0
                    prjNamePat = prjName + " - "
                    for line in lines:
                        if line.find(" ERROR, ") == 0:
                            break
                        elif line.find(prjNamePat) >=0:
                            total_conf += 1
                        elif line.find("Total number of errors: 0") >=0:
                            found += 1
                        elif line.find("Total number of warnings: 0") >=0:
                            found += 1

                    if total_conf == 0 or (total_conf > 0 and found != 2*total_conf):
                        err += 1
                        f.write(os.path.abspath(BUILDLOG) + "\n")
                        print("[" + str(prj_count) + "] " + os.getcwd() + "\\" + file +  " has error or warning.", flush=True)
                    else:
                        print("[" + str(prj_count) + "] " + os.getcwd() + "\\" + file +  " pass.", flush=True)

                except Exception as e:
                    print("[" + str(prj_count) + "] "+ " Build " + file +  " has other exception.", flush=True)
                    err += 1

                except OSError:
                    print("[" + str(prj_count) + "] " + os.path.abspath(file) + " Oops.", flush=True)
                    pass #Silently ignore

                prj_count += 1

                #f.flush()
                os.chdir(root)

    if err == 0:
        print("Build " + str(prj_count-1) + " projects successfully.", flush=True)

    f.close()

    sys.exit(err)
