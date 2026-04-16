"""Append no-unaligned-access flags to MDK5 .uvprojx targets based on AC5/AC6 compiler type."""

import xml.etree.ElementTree as ET
import os
import glob
import sys

COMPILER_FLAGS = {
    "AC5": "--no_unaligned_access",
    "AC6": "-mno-unaligned-access"
}

def ensure_child(parent, tag):
    node = parent.find(tag)
    if node is None:
        node = ET.SubElement(parent, tag)
    return node

def get_target_compiler(target):
    ac6_node = target.find("./uAC6")
    if ac6_node is not None and (ac6_node.text or "").strip() == "1":
        return "AC6"
    return "AC5"

def append_misc_controls(file_path):
    if not os.path.exists(file_path):
        print(f"Error: not found {file_path}")
        return

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()
            if not first_line.startswith('<?xml'):
                first_line = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'

        tree = ET.parse(file_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing {file_path}: {e}")
        return

    is_modified = False

    for target in root.findall(".//Target"):
        target_name_node = target.find("./TargetName")
        target_name = target_name_node.text if target_name_node is not None else "Unknown"
        compiler = get_target_compiler(target)
        extra_flag = COMPILER_FLAGS[compiler]

        target_option = ensure_child(target, "TargetOption")
        target_arm_ads = ensure_child(target_option, "TargetArmAds")
        cads = ensure_child(target_arm_ads, "Cads")
        various_controls = ensure_child(cads, "VariousControls")
        misc_node = ensure_child(various_controls, "MiscControls")

        current_val = misc_node.text if misc_node.text else ""
        if extra_flag not in current_val:
            new_val = f"{current_val} {extra_flag}".strip()
            misc_node.text = new_val
            print(f"  [{target_name}] ({compiler}): Appended '{extra_flag}'")
            is_modified = True
        else:
            print(f"  [{target_name}] ({compiler}): Flag exists, skipping")

    if is_modified:
        xml_content = ET.tostring(root, encoding='utf-8', method='xml',
                                  xml_declaration=False,
                                  short_empty_elements=False).decode('utf-8')

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(first_line + "\n" + xml_content)
        print(f"  Saved changes to: {file_path}")

def scan_directory(base_dir):
    print(f"Scanning/Fixing 'SampleCode' projects under: {os.path.abspath(base_dir)}...")
    search_pattern = os.path.join(base_dir, "**", "*.uvprojx")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]
    if not target_files:
        print("No SampleCode projects found.")
        return
    for f in target_files:
        print(f"\nChecking: {f}")
        append_misc_controls(f)

    print("\nScan and Append complete.")

if __name__ == "__main__":
    base_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    scan_directory(base_dir)