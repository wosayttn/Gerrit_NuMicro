import os
import sys
import subprocess
import shutil
import fnmatch
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

ECLIPSE_EXE=missudad.ECLIPSE_EXE
PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST

def blacklist_check(BUILDLOG):
    Blacklist = ['[Fatal Error]', 'An error has occurred.', ' error: ', ' warning: ', ' Error: ', ' Warning: ', 'warning csolution: ']

    # Find any error/warning
    fp = open(BUILDLOG, "r")
    lines = fp.readlines()
    fp.close()

    new_lines = ''

    found = 0
    for l in lines:
        new_lines += l
        for br in Blacklist:
            pos = l.find(br)
            if pos >= 0:
                space_string = ' ' * pos
                arrow_string = '^' * len(br)
                new_lines += space_string + arrow_string + ' <- Received a blacklist rule.\n\n'
                found += 1

    if found > 0:
        # Update BUILDLOG
        fp = open(BUILDLOG, "w")
        fp.writelines(new_lines)
        fp.close()

    return found

def get_memory_info():
    cmd = 'cmd /c \"systeminfo | find /i \"Available Physical Memory\"\"'
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    subprocess.check_call(cmd, startupinfo=si)
    sys.stdout.flush()

def export_build_ps1(DirPath, prjBuildScript, csolution_file):

    fps1 = open(os.path.join(DirPath, prjBuildScript), "w")

    script_str = f"""\

if (Test-Path "C:\\ArmPacks") {{
    $env:CMSIS_PACK_ROOT = "C:\\ArmPacks"
}}

. ~/.vcpkg/vcpkg-init.ps1
vcpkg activate

$targets=@(cbuild list contexts \"{csolution_file}\")
if ($LASTEXITCODE -ne 0) {{
    exit 1
}}
$numberOfElements = $targets.Count

$okay = 0
foreach ($t in $targets) {{
    cbuild \"{csolution_file}\" --context $t -S
    cbuild \"{csolution_file}\" -S --rebuild --update-rte -d -v
    if ($LASTEXITCODE -eq 0) {{
        $okay+=1
    }}
}}

Write-Output "BuildMachine: built $okay target in $numberOfElements configuration."
if ($okay -eq $numberOfElements) {{
    exit 0
}}
else {{
    exit 1
}}

"""

    fps1.write(script_str)
    fps1.flush()
    fps1.close()

if __name__ == "__main__":
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    err = 0
    root = os.getcwd()
    f = open('vscode.txt', "w")

    prj_count = 1
    bsp_root = os.getcwd()

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):

        for file in fnmatch.filter(fileNames, '*.csolution.y*ml'):

            prjName = os.path.basename(os.path.dirname(dirPath))
            prjFileAbsPath = os.path.join(bsp_root, dirPath, file)

            buildit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or prjName.find(ip) == 0:
                    buildit = 1
                    break

            if buildit == 1:

                prjDirAbsPath = os.path.dirname(prjFileAbsPath)
                BUILDLOG = prjFileAbsPath + '.log'
                prjBuildScript = 'build_' + file + '.ps1'

                buildcommand = "cd \"" + prjDirAbsPath + "\" & PowerShell -File ./" + prjBuildScript + " > \"" + BUILDLOG + "\""

                print(buildcommand)
                print(BUILDLOG)
                try:
                    print(dirPath + " building ...")
                    export_build_ps1(prjDirAbsPath, prjBuildScript, file)
                    found = os.system(buildcommand)

                    found += blacklist_check(BUILDLOG)
                    if found != 0:
                        err += 1
                        f.write("[" + str(prj_count) + "] "+ dirPath +  " has error or warning.\n")
                        #print("Build " + basename + " has error or warning...\n")
                    #else:
                        #f.write("[" + str(prj_count) + "] "+ os.path.abspath(file) +  " pass...\n")
                        #print("\t" + basename +  " pass.\n")
                    pass
                except Exception as e:
                    f.write("[" + str(prj_count) + "] "+ dirPath +  " has Exception.\n")
                    print("Build " + file +  " has Exception.\n")
                    err += 1
                except OSError:
                    print("Build " + file +  " has Ooops...\n")
                    f.write("[" + str(prj_count) + "] "+ dirPath +  " has Ooops.\n")
                    err += 1
                    pass                # Silently ignore

                prj_count += 1
                f.flush()

    os.chdir(root)

    if err == 0:
        f.write("Build " + str(prj_count-1) + " projects successfully.\n")

    f.close()

    if err == 0:
        sys.exit(0)
    else:
        sys.exit(1)
