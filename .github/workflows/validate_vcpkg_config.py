import json
import sys
import os

def validate_json(file_path):
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

    if errors:
        print("vcpkg-configuration.json validation failed:")
        for e in errors:
            print(" -", e)
        return 1
    else:
        return 0

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <json_file>")
        sys.exit(1)

    sys.exit(validate_json(sys.argv[1]))
