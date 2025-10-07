import os
import sys
import shutil

# Define prefix string of IP name you preferred.
# EX: IP_LIST=[ 'UART', 'SPI', 'CLK', 'SYS' ]
# Or append '*' symbol to build all projects.
#IP_LIST=[ '*' ]
IP_LIST=[ 'Template', 'Project' ]

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
CSPYBAT_EXE='C:\\Program Files\\IAR Systems\\Embedded Workbench 9.4\\common\\bin\\cspybat'
IARIDEPM_EXE='C:\\Program Files\\IAR Systems\\Embedded Workbench 9.4\\common\\bin\\IarIdePm.exe'
IARBUILD_EXE="C:\\Program Files\\IAR Systems\\Embedded Workbench 9.4\\common\\bin\\iarbuild.exe"

'''
Define your Keil v537 installation path.
'''
UV4_EXE="C:\\Keil_v5\\UV4\\Uv4.exe"


'''
Don't touch me. Just collect module function here.
'''
def read_serial_port(serial_port):
    while True:
        if serial_port.in_waiting > 0:
            data = serial_port.readline().decode('utf-8').strip()
            print(data)

def write_to_serial_port(serial_port):
    while True:
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

import os

def remove_file(file_path):
    if os.path.isfile(file_path):
        try:
            os.remove(file_path)
            print(f"Removed: {file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to remove {file_path}: {e}")
    else:
        print(f"[INFO] File not found: {file_path}")
