import os
import sys
import time
import subprocess
import shutil
import fnmatch
import tempfile
import glob
import re
import sys
import datetime
import serial
import threading
import time

sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.RUN_LIST
COMX=missudad.COMX
UV4_EXE=missudad.UV4_EXE

if __name__ == "__main__":

    # Replace 'COMx' with the actual serial port on your system, e.g., 'COM1' or '/dev/ttyUSB0'
    serial_port = serial.Serial(COMX, 115200, timeout=1)

    # Create and start the read and write threads
    read_thread = threading.Thread(target=missudad.read_serial_port, daemon=True, args=(serial_port,))
    write_thread = threading.Thread(target=missudad.write_to_serial_port, daemon=True, args=(serial_port,))

    read_thread.start()
    write_thread.start()

    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    root = os.getcwd()
    os.chdir(root)

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):
        for file in fnmatch.filter(fileNames, '*.uvprojx'):
            runit = 0
            for ip in RUN_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    runit = 1
                    break

            if runit == 1:
                os.chdir(dirPath)
                prjName = os.path.splitext(file)[0]
                print('#################################### '+prjName+' ####################################')
                try:
                    RunCmd = UV4_EXE + ' -j0 -f ' + file
                    #p = subprocess.Popen(RunCmd, startupinfo=si, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                    p = subprocess.Popen(RunCmd, startupinfo=si)
                    p.wait(missudad.RUNTIME)
                    time.sleep(missudad.RUNTIME)
                except subprocess.TimeoutExpired:
                    p.kill()
                except OSError:
                    pass    #Silently ignore
                print('************************************ ' + prjName+' ************************************')
                os.chdir(root)

    sys.exit(0)
