import sys
import os
import re
import fnmatch
import json
import yaml
import xml.etree.ElementTree as ET

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

            value = xml_get_value(t, 'TargetOption/TargetArmAds/Cads', 'wLevel')
            if value != "2":
                errors.append(f"{target_name} - Invalid wLevel setting: {value} (should be 2, all warnings)")

            value = xml_get_value(t, '.', 'pCCUsed')
            if value != "6240000::V6.24::ARMCLANG":
                errors.append(f"{target_name} - Invalid pCCUsed version: {value} (should be 6240000::V6.24::ARMCLANG)")

    return errors


def vcpkg_configuration(file_path):

    errors = []

    if not os.path.exists(file_path):

        errors.append(f"vcpkg-configuration.json not found -> {file_path}")

    else:

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

                # 1. check registries.name is only arm
                registries = data.get("registries", [])
                for reg in registries:
                    if reg.get("name") != "arm":
                        errors.append(f"Invalid registry name: {reg.get('name')} (should be kept 'arm' only)")

                # 2. Check requires.arm-none-eabi-gcc is 14.3.1?
                requires = data.get("requires", {})
                for key, value in requires.items():
                    if "arm-none-eabi-gcc" in key:
                        if value != "14.3.1":
                            errors.append(f"Invalid arm:compilers/arm/arm-none-eabi-gcc: {value} (should be 14.3.1)")

                # 3. check requires.cmsis-toolbox is 2.9.0
                for key, value in requires.items():
                    if "cmsis-toolbox" in key:
                        if value != "2.9.0":
                            errors.append(f"Invalid arm:tools/open-cmsis-pack/cmsis-toolbox version: {value} (should be 2.9.0)")

                # 4. check requires.cmake is 3.31.5
                for key, value in requires.items():
                    if "cmake" in key:
                        if value != "3.31.5":
                            errors.append(f"Invalid arm:tools/kitware/cmake version: {value} (should be 3.31.5)")

        except json.JSONDecodeError as e:
            errors.append(f"Error: Failed to parse JSON -> {file_path}")
            errors.append(f"Details: {e}")

    return errors

def vcpkg_cproject(file_path):

    errors = []

    if not os.path.exists(file_path):
        errors.append(f"Error: File not found -> {file_path}")
    else:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            errors.append(f"Error: Failed to parse YAML -> {file_path}")
            errors.append(f"Details: {e}")
            return errors

        isLibBuilding = 0

        # (1) Check AC6 C-CPP compiler flags
        setups = data.get("project", {}).get("setups", [])
        for setup in setups:

            output = setup.get("output")
            if isinstance(output, dict):
                if output.get("type") == "lib":
                    isLibBuilding = 1
                    break

            misc_list = setup.get("misc", [])
            for misc in misc_list:
                if misc.get("for-compiler") == "AC6":
                    c_flags = " ".join(misc.get("C", []))
                    c_cpp_flags = " ".join(misc.get("C-CPP", []))  # Corrected key
                    cpp_flags = " ".join(misc.get("CPP", []))  # Corrected key
                    all_flags = c_flags + " " + cpp_flags + " " + c_cpp_flags
                    if " -Wall" not in all_flags:
                        errors.append(f"C/C-CPP/CPP for AC6 does not contain -Wall")
                    if " -w" in all_flags:
                        errors.append(f"C/C-CPP/CPP for AC6 contains -w which is forbidden")
                    break # Only check the first misc with for-compiler AC6

            break # Only check the first setup

        # (2) Check if _syscalls.c exists under GCC compiler condition
        '''
        if isLibBuilding != 1:
            syscalls_found = False
            groups = data.get("project", {}).get("groups", [])
            for group in groups:
                files = group.get("files", [])
                for f in files:
                    filename = f.get("file", "")
                    compiler = f.get("for-compiler", None)
                    if filename.endswith("_syscalls.c") and compiler == "GCC":
                        syscalls_found = True

            if not syscalls_found:
                errors.append("_syscalls.c not found under GCC compiler condition in groups")
        '''


        '''
        # (3) Check if at least one pack matches Nuvoton::NuMicro*_DFP
        packs = data.get("project", {}).get("packs", [])
        pattern = re.compile(r"Nuvoton::NuMicroM.*_DFP")
        pack_found = any(pattern.match(p.get("pack", "")) for p in packs)

        if not pack_found:
            errors.append("No pack found matching 'Nuvoton::NuMicroM*_DFP'")
        '''

    return errors


VALIDATION_FUNCTIONS = {
    '*.uvproj*': keil_uvprojx,
    '*.cproject.yml': vcpkg_cproject,
    'vcpkg-configuration.json': vcpkg_configuration,
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

    err = 0

    for dirPath, dirNames, fileNames in os.walk(sys.argv[1]):
        for pattern, update_function in VALIDATION_FUNCTIONS.items():
            for file in fnmatch.filter(fileNames, pattern):
                prjFileAbsPath = os.path.join(dirPath, file)
                err |= validate_project_file(prjFileAbsPath, update_function)

    sys.exit(err)
