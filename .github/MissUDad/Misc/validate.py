import sys
import os
import re
import fnmatch
import json
import yaml
import xml.etree.ElementTree as ET
import requests
import glob
import shutil
import subprocess
from packaging import version  # standard tool for version comparison

def to_int(hex_val):
    """Convert hex string to integer for comparison."""
    try:
        if isinstance(hex_val, str):
            return int(hex_val, 16)
    except (ValueError, TypeError):
        return None
    return None

def download_pdsc(pack_id, pack_url, download_dir="."):
    """Download PDSC if not present locally."""
    parts = pack_id.split('.')
    if len(parts) < 2: return None
    pdsc_filename = f"{parts[0]}.{parts[1]}.pdsc"
    save_path = os.path.join(download_dir, pdsc_filename)
    
    if os.path.exists(save_path):
        return save_path

    base_url = pack_url if pack_url.endswith('/') else pack_url + '/'
    full_url = f"{base_url}{pdsc_filename}"
    try:
        response = requests.get(full_url, timeout=15)
        response.raise_for_status()
        with open(save_path, 'wb') as f:
            f.write(response.content)
        return save_path
    except:
        return None

def parse_pdsc_memory(pdsc_path, device_name):
    """Extract all memory layouts (IROM1-3, IRAM1-3) from PDSC."""
    if not pdsc_path or not os.path.exists(pdsc_path): return None
    try:
        tree = ET.parse(pdsc_path)
        root = tree.getroot()
        parent_map = {c: p for p in root.iter() for c in p}
        device_node = None
        for device in root.findall(".//device"):
            if device.get('Dname', '').upper() == device_name.upper():
                device_node = device
                break
        if not device_node: return None
        pdsc_mem = {}
        curr = device_node
        while curr is not None:
            for mem in curr.findall("memory"):
                m_id = mem.get('id') or mem.get('name')
                if m_id in ['IROM1', 'IROM2', 'IROM3', 'IRAM1', 'IRAM2', 'IRAM3']:
                    if m_id not in pdsc_mem:
                        pdsc_mem[m_id] = {
                            "val_start": to_int(mem.get('start')),
                            "val_size": to_int(mem.get('size')),
                            "raw_start": mem.get('start'),
                            "raw_size": mem.get('size')
                        }
            curr = parent_map.get(curr)
        return pdsc_mem
    except:
        return None

def process_single_project(uvprojx_path):
    """Inspect and automatically fix memory mismatches in .uvprojx."""
    try:
        tree = ET.parse(uvprojx_path)
        root = tree.getroot()
    except:
        return

    check_list = ['IROM1', 'IROM2', 'IROM3', 'IRAM1', 'IRAM2', 'IRAM3']

    return_errors = []

    project_header_printed = False
    for target in root.findall(".//Target"):
        t_name = target.find("TargetName").text
        device_node = target.find(".//Device")
        pack_id_node = target.find(".//PackID")
        pack_url_node = target.find(".//PackURL")
        cpu_node = target.find(".//Cpu")

        if any(x is None for x in [device_node, pack_id_node, pack_url_node, cpu_node]):
            continue

        device_name = device_node.text
        pdsc_file = download_pdsc(pack_id_node.text, pack_url_node.text)
        if not pdsc_file: continue

        truth = parse_pdsc_memory(pdsc_file, device_name)
        if not truth: continue

        matches = re.findall(r"(\w+)\((0x[0-9a-fA-F]+),(0x[0-9a-fA-F]+)\)", cpu_node.text)
        proj_mem = {}
        for m in matches:
            m_id = m[0]
            if m_id == "IROM": m_id = "IROM1"
            if m_id == "IRAM": m_id = "IRAM1"
            proj_mem[m_id] = {
                "v_s": to_int(m[1]), "v_z": to_int(m[2]), 
                "r_s": m[1], "r_z": m[2]
            }

        target_errors = []
        for m_id in check_list:
            p_val = truth.get(m_id)
            u_val = proj_mem.get(m_id)
            
            if not p_val: continue 
            
            if not u_val:
                target_errors.append(f"    [MISSING] {m_id}: Expected {p_val['raw_start']} in <{t_name}>")
            elif u_val['v_s'] != p_val['val_start'] or u_val['v_z'] != p_val['val_size']:
                target_errors.append(
                    f"    [MISMATCH] {m_id} in Target <{t_name}>!\n"
                    f"               Proj: {u_val['r_s']}, {u_val['r_z']}\n"
                    f"               Pack: {p_val['raw_start']}, {p_val['raw_size']}"
                )

        if target_errors:
            # Insert Target info at the very beginning (index 0)
            target_errors.insert(0, f"  Target: {t_name} ({device_name})")

            # Insert File header only if it hasn't been printed/added yet
            if not project_header_printed:
                target_errors.insert(0, f"\n[!] ISSUES FOUND IN: {uvprojx_path}")
                project_header_printed = True            

            return_errors += target_errors

    return return_errors

def xml_get_value(node, path_parent, tag_name):
    # Locate Parent
    TParent = node.find(path_parent)
    if TParent is None:
        raise RuntimeError("TParent section not found")

    # Locate Tag under Parent
    TTag = TParent.find(tag_name)
    if TTag is None:
        raise RuntimeError("TTag not found inside Cads")

    # Return value
    return TTag.text

def keil_uvprojx(file_path):

    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:

        # 1. Get the directory name of the file
        parent_dir = os.path.basename(os.path.dirname(file_path))

        # 2. Check if the parent directory is NOT strictly 'Keil'
        if parent_dir.lower() == "keil":
            if parent_dir != "Keil":
                errors.append(f"Error: '{file_path}' Directory name must be strictly 'Keil' (case-sensitive). Found: '{parent_dir}'")
                return errors

        # 3. Check if file exists
        try:
            tree = ET.parse(file_path)
            prj_tree_root = tree.getroot()
        except ET.ParseError as e:
            errors.append(f"Error: Failed to parse JSON -> {file_path}")
            errors.append(f"Details: {e}")
            return errors

        prj_tree_targets = prj_tree_root.findall("./Targets/Target")
        if not prj_tree_targets:
            errors.append(f"Error: No prj_tree_targets.<Target> found")
            return errors

        for t in prj_tree_targets:
            target_name = xml_get_value(t, '.', 'TargetName')

            value = xml_get_value(t, 'TargetOption/TargetCommonOption', 'PackID')
            if not fnmatch.fnmatch(value, "Nuvoton.NuMicroM*_*"):
                errors.append(f"{target_name} - Invalid PackID version: {value} (should be Nuvton.NuMicroM*_*)")


            value = xml_get_value(t, '.', 'pCCUsed')
            # Example value format: "6240000::V6.24::ARMCLANG"
            try:
                # Extract the version part, e.g. "V6.24"
                version_str = value.split("::")[1].lstrip("V")
                version_num = float(version_str)
            except (IndexError, ValueError):
                errors.append(f"{target_name} - Invalid pCCUsed format: {value}")
            else:
                if version_num <= 6.22:
                    errors.append(f"{target_name} - Invalid pCCUsed version: {value} (should be > V6.22)")

        # pdsc checking
        errors += process_single_project(file_path)

    return errors


def vcpkg_configuration(file_path):

    errors = []

    if not os.path.exists(file_path):

        errors.append(f"vcpkg-configuration.json not found -> {file_path}")

    else:

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

                # check registries.name is only arm
                registries = data.get("registries", [])
                for reg in registries:
                    if reg.get("name") != "arm":
                        errors.append(f"Invalid registry name: {reg.get('name')} (should be kept 'arm' only)")

                # check requires versions
                requires = data.get("requires", {})
                for key, value in requires.items():
                    try:
                        if "arm-none-eabi-gcc" in key:
                            if version.parse(value) < version.parse("14.3.1"):
                                errors.append(f"Invalid arm:compilers/arm/arm-none-eabi-gcc: {value} (should be >= 14.3.1)")

                        if "cmsis-toolbox" in key:
                            if version.parse(value) < version.parse("2.9.0"):
                                errors.append(f"Invalid arm:tools/open-cmsis-pack/cmsis-toolbox version: {value} (should be >= 2.9.0)")

                        if "cmake" in key:
                            if version.parse(value) != version.parse("3.31.5"):
                                errors.append(f"Invalid arm:tools/kitware/cmake version: {value} (should be >= 3.31.5)")

                    except Exception as e:
                        errors.append(f"Invalid version format for {key}: {value} ({e})")

        except json.JSONDecodeError as e:
            errors.append(f"Error: Failed to parse JSON -> {file_path}")
            errors.append(f"Details: {e}")

    return errors


def vcpkg_csolution_convert(file_path):
    """
    Run 'csolution convert <solution_file>' and print its output.
    
    :param solution_file: Path to your .csolution.yml file
    """
    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:
        try:
            result = subprocess.run(
                ["csolution", "convert", file_path],
                check=True,
                capture_output=True,
                text=True
            )
        except subprocess.CalledProcessError as e:
            errors.append(f"Error while running csolution convert")

    return errors


def vcpkg_cbuild(yaml_file):
    with open(yaml_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    errors = []

    compiler = data.get("build", {}).get("compiler", "")
    if compiler == "AC6":
        misc_list = data.get("build", {}).get("misc", [])
        if isinstance(misc_list, dict): 
            misc_list = [misc_list]

        for misc in misc_list:
            if "C" in misc:
                c_flags = " ".join(misc.get("C", []))
                c_cpp_flags = " ".join(misc.get("C-CPP", []))
                cpp_flags = " ".join(misc.get("CPP", []))
                all_flags = f"{c_flags} {cpp_flags} {c_cpp_flags}"

                if "-Wall" not in all_flags.split():
                    errors.append("C for AC6 should contain -Wall (missing)")

                if "-w" in all_flags.split():
                    errors.append("C for AC6 contains -w which is forbidden")

    device_pack = data.get("build", {}).get("device-pack", "")
    if device_pack:
        if not re.search(r"Nuvoton::NuMicroM\d+_DFP(@[\w\.\-]+)?", device_pack):
            errors.append(f"device-pack should match Nuvoton::NuMicroM*_DFP* (got: {device_pack})")

    return errors


def vscode_csolution(file_path):

    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:

        # 1. Get the directory name of the file
        parent_dir = os.path.basename(os.path.dirname(file_path))

        # 2. Check if the parent directory is NOT strictly 'VSCode'
        if parent_dir.lower() == "vscode":
            if parent_dir != "VSCode":
                errors.append(f"Error: '{file_path}' Directory name must be strictly 'VSCode' (case-sensitive). Found: '{parent_dir}'")
                return errors

    return errors

def nueclipse_project(file_path):

    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:

        # 1. Get the directory name of the file
        parent_dir = os.path.basename(os.path.dirname(file_path))

        # 2. Check if the parent directory is NOT strictly 'GCC'
        if parent_dir.lower() == "gcc":
            if parent_dir != "GCC":
                errors.append(f"Error: '{file_path}' Directory name must be strictly 'GCC' (case-sensitive). Found: '{parent_dir}'")
                return errors

    return errors

def iar_project(file_path):

    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:

        # 1. Get the directory name of the file
        parent_dir = os.path.basename(os.path.dirname(file_path))

        # 2. Check if the parent directory is NOT strictly 'IAR'
        if parent_dir.lower() == "iar":
            if parent_dir != "IAR":
                errors.append(f"Error: '{file_path}' Directory name must be strictly 'IAR' (case-sensitive). Found: '{parent_dir}'")
                return errors

    return errors


VALIDATION_FUNCTIONS = {
    '*.cbuild.yml': vcpkg_cbuild,
    'vcpkg-configuration.json': vcpkg_configuration,
    '*.uvproj*': keil_uvprojx,
    '*.csolution': vscode_csolution,
    '.project': nueclipse_project,
    '*.eww' : iar_project
}

def validate_project_file(file_abs_path, update_function):
    errors=update_function(file_abs_path)
    if errors:
        print(file_abs_path,"validation failed:")
        for e in errors:
            print("- ", e)
        print(" ")
        return 1
    else:
        return 0


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} CHECK_FOLDER")
        sys.exit(1)

    for dirPath, dirNames, fileNames in os.walk(sys.argv[1]):
        for file in fnmatch.filter(fileNames, '*.csolution.yml'):
            vcpkg_csolution_convert(os.path.join(dirPath, file))

    err = 0

    for dirPath, dirNames, fileNames in os.walk(sys.argv[1]):
        for pattern, update_function in VALIDATION_FUNCTIONS.items():
            for file in fnmatch.filter(fileNames, pattern):
                prjFileAbsPath = os.path.join(dirPath, file)
                err |= validate_project_file(prjFileAbsPath, update_function)

    sys.exit(err)
