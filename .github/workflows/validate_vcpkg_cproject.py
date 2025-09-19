import yaml
import sys
import os
import re

def validate_project_yaml(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File not found -> {file_path}")
        return(1)

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML -> {file_path}")
        print(f"Details: {e}")
        return(1)

    errors = []

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
                # Check C flags
                print(f"Global misc -->")
                print(misc)
                print(f" ")
                c_flags = " ".join(misc.get("C", []))
                c_cpp_flags = " ".join(misc.get("C-CPP", []))  # Corrected key
                cpp_flags = " ".join(misc.get("CPP", []))  # Corrected key
                all_flags = c_flags + " " + cpp_flags + " " + c_cpp_flags
                print(f"--> Global AC6 C/C++ flags: {all_flags}")
                if " -Wall" not in all_flags:
                    errors.append("C/C-CPP/CPP for AC6 does not contain -Wall")
                if " -w" in all_flags:
                    errors.append("C/C-CPP/CPP for AC6 contains -w which is forbidden")
                break # Only check the first misc with for-compiler AC6

        break # Only check the first setup

    # (2) Check if _syscalls.c exists under GCC compiler condition
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
    # (3) Check if at least one pack matches Nuvoton::NuMicro*_DFP
    packs = data.get("project", {}).get("packs", [])
    pattern = re.compile(r"Nuvoton::NuMicroM.*_DFP")
    pack_found = any(pattern.match(p.get("pack", "")) for p in packs)

    if not pack_found:
        errors.append("No pack found matching 'Nuvoton::NuMicroM*_DFP'")
    '''

    # Show results
    if errors:
        print(file_path,"validation failed:")
        for e in errors:
            print(" -", e)
        return(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <yaml_file>")
        sys.exit(1)

    sys.exit(validate_project_yaml(sys.argv[1]))
