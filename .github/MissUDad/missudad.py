import os
import sys
import shutil
import time

# Define prefix string of IP name you preferred.
# EX: IP_LIST=[ 'UART', 'SPI', 'CLK', 'SYS' ]
# Or append '*' symbol to build all projects.
IP_LIST=[ '*' ]
#IP_LIST=[ 'Template', 'Project' ]
RUN_LIST=[ 'Template', 'Project' ]

# Define VCOM port number of NuMaker board.
COMX='COM4'

# Define running interval between SampleCodes. (Default: 20 seconds)
# Notice: The value depends on firmware flash time and SampleCode running time.
RUNTIME=20

# Define searching root directory
PROJ_FOLDER_NAME='../../../SampleCode'

'''
Define your NuEclipse/GDB/OpenOCD installation path.
'''
OPENOCD_EXE='C:\\Program Files (x86)\\Nuvoton Tools\\OpenOCD\\bin\\openocd_cmsis-dap.exe'
GDB_EXE='C:\\Program Files (x86)\\GNU Arm Embedded Toolchain\\10 2021.10\\bin\\arm-none-eabi-gdb.exe'
ECLIPSE_EXE="C:\\Program Files (x86)\\Nuvoton Tools\\NuEclipse\\V1.02.025c\\NuEclipse\\eclipse\\eclipsec.exe"

'''
Define your IAR embedded workbench installation path.
'''
IAR7_INSTALLATION_PATH="C:\\Program Files (x86)\\IAR Systems\\Embedded Workbench 7.5\\"
IAR8_INSTALLATION_PATH="C:\\Program Files (x86)\\IAR Systems\\Embedded Workbench 8.4\\"
IAR9_INSTALLATION_PATH="C:\\Program Files\\IAR Systems\\Embedded Workbench 9.2\\"

CSPY7BAT_EXE=IAR7_INSTALLATION_PATH + "common\\bin\\cspybat"
IAR7IDEPM_EXE=IAR7_INSTALLATION_PATH + "common\\bin\\IarIdePm.exe"
IAR7BUILD_EXE=IAR7_INSTALLATION_PATH + "common\\bin\\iarbuild.exe"

CSPY8BAT_EXE=IAR8_INSTALLATION_PATH + "common\\bin\\cspybat"
IAR8IDEPM_EXE=IAR8_INSTALLATION_PATH + "common\\bin\\IarIdePm.exe"
IAR8BUILD_EXE=IAR8_INSTALLATION_PATH + "common\\bin\\iarbuild.exe"

CSPY9BAT_EXE=IAR9_INSTALLATION_PATH + "common\\bin\\cspybat"
IAR9IDEPM_EXE=IAR9_INSTALLATION_PATH + "common\\bin\\IarIdePm.exe"
IAR9BUILD_EXE=IAR9_INSTALLATION_PATH + "common\\bin\\iarbuild.exe"

CSPYBAT_EXE=CSPY9BAT_EXE
IARIDEPM_EXE=IAR9IDEPM_EXE
IARBUILD_EXE=IAR9BUILD_EXE

'''
Define your Keil v5 installation path.
'''
UV4_EXE="C:\\Keil_v5\\UV4\\Uv4.exe"


'''
Don't touch me. Just collect module function here.
'''
def read_serial_port(serial_port, stop_event=None, eot_event=None):
    """
    Continuously read lines from the serial port and print them.
    - stop_event: threading.Event to exit the loop gracefully
    - eot_event: threading.Event to signal that EOT (\x04) was received
    """
    buffer = ""
    start_time = time.time()
    while True:
        if stop_event and stop_event.is_set():
            break
        try:
            if serial_port.in_waiting > 0:
                # Read one byte at a time for EOT detection
                byte = serial_port.read(1)
                if not byte:
                    continue

                char = byte.decode('utf-8', errors='replace')

                if char == '\x04':  # End-of-Transmission detected
                    print("[EOT received]")
                    if eot_event:
                        eot_event.set()
                    # Optionally, break loop if you want to stop reading
                    break

                buffer += char
                if char == '\n':
                    # Complete line received
                    line = buffer.strip()
                    print(line)
                    buffer = ""

            #Check elapsed time
            elif time.time() - start_time >= RUNTIME:
                print("Thread timeout reached")
                break

            else:
                time.sleep(0.01)  # Avoid busy-waiting

        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            break

def write_to_serial_port(serial_port):
    while True:
        if sys.stdin.isatty():
            message = input('$ ')
            serial_port.write(message.encode('utf-8'))
            print(message)

def mirror_file(src_file, dst_file):
    print('Copy ' + src_file+ ' to ' + dst_file)
    dst_folder = os.path.dirname(dst_file)
    if not os.path.exists(dst_folder):
       os.makedirs(dst_folder)

    shutil.copyfile(src_file, dst_file)

def move_file(src_file, dst_file):
    print('Move ' + src_file+ ' to ' + dst_file)
    dst_folder = os.path.dirname(dst_file)
    if not os.path.exists(dst_folder):
       os.makedirs(dst_folder)

    shutil.move(src_file, dst_file)

def remove_file(file_path):
    if os.path.isfile(file_path):
        try:
            os.remove(file_path)
            print(f"Removed: {file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to remove {file_path}: {e}")
    else:
        print(f"[INFO] File not found: {file_path}")
