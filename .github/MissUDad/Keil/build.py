import os
import sys
import subprocess
import fnmatch
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST=missudad.IP_LIST
UV4_EXE=missudad.UV4_EXE

if __name__ == "__main__":

    si = subprocess.STARTUPINFO()

    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    err = 0

    root = os.getcwd()
    f = open('attachments.txt', "w+")
    os.chdir(root)

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
                    BUILDLOG = file + ".log"
                    buildcommnd = UV4_EXE + " -b -j0 -z -o " + BUILDLOG + " " + file
                    #cleancommnd = UV4_EXE + " -j0 -c " + file

                    # https://www.keil.com/support/man/docs/uv4cl/uv4cl_commandline.htm
                    # -j0   Hides the µVision GUI. Messages are suppressed. Use this option for batch testing.
                    # -z    Re-builds all targets of a project or multiple-project.
                    #       Ensure that each target has another object output folder.
                    #       Use the menu Projects - Options for Target - Output - Select Folder for Objects.
                    # -b    Builds the last current target of a project and exits after the build process finished.
                    #       Refer to option -t to change the target.
                    #       For multi-projects, the command builds the targets as defined in the dialog Project - Batch Build.
                    # -o outputfile
                    #print("[" + str(prj_count) + "] Build " + os.path.abspath(file) +  "\n")

                    #print("[" + str(prj_count) + "] "+ os.getcwd() + "\\" + file +  " cleaning.\n")
                    #p = subprocess.Popen(cleancommnd, startupinfo=si, stdout=f, stderr=f)
                    #p.wait(120)

                    #print("[" + str(prj_count) + "] "+ os.getcwd() + "\\" + file +  " building.\n")
                    p = subprocess.Popen(buildcommnd, startupinfo=si, stdout=f, stderr=f)
                    p.wait(300)

                    # It's a bit strange keil report error code as 0 even build failed. so parse k.log
                    tmp = open(BUILDLOG, "r")
                    lines = tmp.readlines()
                    tmp.close()
 
                    found = 0
                    for line in lines:
                        if line.find("0 Error(s), 0 Warning(s)") >= 0:
                            found = 1

                    if found == 0:
                        if err > 0:
                            f.write(",")
                        f.write(os.path.abspath(BUILDLOG))                            
                        print("❌ Build failed: " + os.path.abspath(file), flush=True)
                        err += 1
                    else:
                        print("✅ Build success: " + os.path.abspath(file), flush=True)

                except subprocess.TimeoutExpired:
                    p.kill()
                    print("❌ Build TimeoutExpired: " + os.path.abspath(file), flush=True)

                except Exception as e:
                    print("❌ Build Exception: " + os.path.abspath(file), flush=True)
                    traceback.print_exc()
                    err += 1
                    pass #Silently ignore

                except OSError:
                    print("❌ Build OSError: " + os.path.abspath(file), flush=True)
                    err += 1
                    pass #Silently ignore

                prj_count += 1

                #f.flush()
                os.chdir(root)

    if err == 0:
        print("🎉 Build " + str(prj_count-1) + " projects successfully.", flush=True)

    f.close()

    sys.exit(err)