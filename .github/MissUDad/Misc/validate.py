import sys
import os
import re
import fnmatch
import json
import yaml
import xml.etree.ElementTree as ET
import subprocess
from packaging import version  # standard tool for version comparison

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

VALIDATION_FUNCTIONS = {
    '*.cbuild.yml': vcpkg_cbuild,
    'vcpkg-configuration.json': vcpkg_configuration,
    '*.uvproj*': keil_uvprojx,
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
