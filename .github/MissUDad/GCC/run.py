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
OPENOCD_EXE=missudad.OPENOCD_EXE
GDB_EXE=missudad.GDB_EXE

if __name__ == "__main__":

    # Replace 'COMx' with the actual serial port on your system, e.g., 'COM1' or '/dev/ttyUSB0'
    serial_port = serial.Serial(COMX, 115200, timeout=1)

    # Create and start the read and write threads
    read_thread = threading.Thread(target=missudad.read_serial_port, daemon=True, args=(serial_port,))
    write_thread = threading.Thread(target=missudad.write_to_serial_port, daemon=True, args=(serial_port,))

    read_thread.start()
    write_thread.start()

    siocd = subprocess.STARTUPINFO()
    siocd.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    sigdb = subprocess.STARTUPINFO()
    sigdb.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    root = os.getcwd()
    os.chdir(root)

    for dirPath, dirNames, fileNames in os.walk(PROJ_FOLDER_NAME):
        for file in fnmatch.filter(fileNames, '*.cproject'):
            basename = os.path.basename(os.path.dirname(dirPath))
            prjName = basename
            runit = 0
            for ip in IP_LIST:
                if ip.find('*') == 0 or prjName.find(ip) == 0:
                    runit = 1
                    break

            if runit == 1:
                RelPath = os.path.join(dirPath, 'Release')
                os.chdir(RelPath)
                PrjElfName = prjName + '.elf'

                try:
                    print('#################################### '+prjName+' ####################################')
                    OpenOcdCmd = OPENOCD_EXE + ' -f ../scripts/interface/cmsis-dap.cfg -f ../scripts/target/numicroM55_nulink2.cfg'
                    #print(OpenOcdCmd)
                    p_openocd = subprocess.Popen(OpenOcdCmd, startupinfo=siocd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                    #p_openocd = subprocess.Popen(OpenOcdCmd, startupinfo=siocd)

                    GdbCmd = GDB_EXE + ' -q -ex \"set confirm off\" -ex \"target remote localhost:3333\" -ex \"set remotetimeout 5\" -ex \"load ' + PrjElfName + ' \" -ex \"continue\" -ex \"detach\" '
                    #print(GdbCmd)
                    p_gdb = subprocess.Popen(GdbCmd, startupinfo=sigdb, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                    #p_gdb = subprocess.Popen(GdbCmd, startupinfo=sigdb)

                    p_gdb.wait(missudad.RUNTIME)

                except subprocess.TimeoutExpired:

                    print('************************************ ' + prjName+' ************************************')
                    p_gdb.kill()

                except OSError:
                    pass    #Silently ignore

                p_openocd.kill()
                os.chdir(root)

    print('Bye')

    sys.exit(0)
