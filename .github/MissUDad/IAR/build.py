import os
import sys
import subprocess
import fnmatch
import traceback
import xml.etree.ElementTree as ET
import sys
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST=missudad.IP_LIST

def get_toolchain_path(PRJ_EWP):
    '''
    Parse OGLastSavedByProductVersion value in EWP project file.
    <option>
        <name>OGLastSavedByProductVersion</name>
        <state>9.50.2.71646</state>
    </option>
    '''

    tree = ET.parse(PRJ_EWP)
    root = tree.getroot()

    # Search for <option> tags
    for option in root.findall(".//option"):
        name_elem = option.find("name")
        state_elem = option.find("state")
        if name_elem is not None and name_elem.text == "OGLastSavedByProductVersion":
            if state_elem is not None:
                version = state_elem.text.strip()
                if version.startswith('9'):
                    return missudad.IAR9BUILD_EXE
                else if version.startswith('8'):
                    return missudad.IAR8BUILD_EXE
                break

    return missudad.IAR8BUILD_EXE

if __name__ == "__main__":

    si = subprocess.STARTUPINFO()

    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    err = 0

    root = os.getcwd()
    f = open('attachments.txt', "w+")
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

                try:
                    os.chdir(dirPath)
                    BUILDLOG = file + ".log"

                    fp = open(BUILDLOG, "w")

                    prjName = os.path.splitext(file)[0]
                    buildcommnd = get_toolchain_path(file) + " " + file + " -build * -log warnings"
                    subprocess.call(buildcommnd, startupinfo=si, stdout=fp, stderr=fp)

                    fp.flush()
                    fp.close()

                    # Find any error/warning
                    tmp = open(BUILDLOG, "r")
                    lines = tmp.readlines()
                    tmp.close()

                    found = 0
                    total_conf = 0
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
                        if err > 0:
                            f.write(",")
                        f.write(os.path.abspath(BUILDLOG))
                        print("❌ Build failed: " + os.path.abspath(file), flush=True)
                        err += 1
                    else:
                        print("✅ Build success: " + os.path.abspath(file), flush=True)

                except Exception:
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
