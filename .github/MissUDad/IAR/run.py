import os
import sys
import subprocess
import fnmatch
import sys
import serial
import threading
import time

sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
RUN_LIST = missudad.RUN_LIST
COMX=missudad.COMX
CSPYBAT_EXE=missudad.CSPYBAT_EXE
IARIDEPM_EXE=missudad.IARIDEPM_EXE

if __name__ == "__main__":

    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    root = os.getcwd()
    os.chdir(root)

    # Replace 'COMx' with the actual serial port on your system, e.g., 'COM1' or '/dev/ttyUSB0'
    serial_port = serial.Serial(COMX, 115200, timeout=1)

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):
        for file in fnmatch.filter(fileNames, '*.eww'):
            runit = 0
            for ip in RUN_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    runit = 1
                    break

            if runit == 1:
                os.chdir(dirPath)
                prjName = os.path.splitext(file)[0]
                print(prjName+' log:\n')

                try:
                    buildcommnd = IARIDEPM_EXE + " " + file
                    p = subprocess.Popen(buildcommnd, startupinfo=si)
                    p.wait(10)
                except subprocess.TimeoutExpired:
                    p.kill()
                except OSError:
                    pass    #Silently ignore

                try:
                    generalXclPath = os.path.abspath(os.path.join('settings', prjName + '.Release.general.xcl'))
                    driverXclPath  = os.path.abspath(os.path.join('settings', prjName + '.Release.driver.xcl'))
                    cspybatCmd = CSPYBAT_EXE + ' -f \"' + generalXclPath + '\" --backend -f \"' + driverXclPath + '\"'
                    p = subprocess.Popen(cspybatCmd, startupinfo=si)
                    p.wait(missudad.RUNTIME)

                    # Create and start the read threads
                    read_thread = threading.Thread(target=missudad.read_serial_port, args=(serial_port,))
                    read_thread.start()
                    read_thread.join()
                except subprocess.TimeoutExpired:
                    p.kill()
                except OSError:
                    pass    #Silently ignore
                os.chdir(root)

    sys.exit(0)
