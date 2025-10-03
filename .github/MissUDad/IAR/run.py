import os
import sys
import subprocess
import fnmatch
import sys
import serial
import threading
sys.path.append(os.path.join(os.path.dirname(os.getcwd())))
import missudad

PROJ_FOLDER_NAME = missudad.PROJ_FOLDER_NAME
IP_LIST = missudad.IP_LIST
COMX=missudad.COMX
CSPYBAT_EXE=missudad.CSPYBAT_EXE
IARIDEPM_EXE=missudad.IARIDEPM_EXE

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
        for file in fnmatch.filter(fileNames, '*.eww'):
            runit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or file.find(ip) == 0:
                    runit = 1
                    break

            if runit == 1:
                os.chdir(dirPath)
                prjName = os.path.splitext(file)[0]

                try:
                    buildcommnd = IARIDEPM_EXE + " " + file
                    #print(buildcommnd)
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
                    #print(cspybatCmd)
                    print('#################################### '+prjName+' ####################################')
                    #p = subprocess.Popen(cspybatCmd, startupinfo=si, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                    p = subprocess.Popen(cspybatCmd, startupinfo=si)
                    p.wait(missudad.RUNTIME)
                except subprocess.TimeoutExpired:
                    print('************************************ ' + prjName+' ************************************')
                    p.kill()
                except OSError:
                    pass    #Silently ignore

                os.chdir(root)

    print('Bye')

    sys.exit(0)
