import xml.etree.ElementTree as ET
import os
import glob
import shutil
import sys

# Flag to append for Keil/ARM Compiler
EXTRA_MISCCONTROLS = '-Wconditional-uninitialized'

def append_misc_controls(file_path, extra_flag):
    if not os.path.exists(file_path):
        print(f"Error: not found {file_path}")
        return

    try:
        # Preserve original header
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

    # Path: Target -> TargetOption -> TargetArmAds -> Cads -> VariousControls -> MiscControls
    for target in root.findall(".//Target"):
        target_name_node = target.find("./TargetName")
        target_name = target_name_node.text if target_name_node is not None else "Unknown"
        
        # Look for the specific MiscControls node
        misc_node = target.find("./TargetOption/TargetArmAds/Cads/VariousControls/MiscControls")
        
        if misc_node is not None:
            current_val = misc_node.text if misc_node.text else ""
            
            # Only append if the flag isn't already there
            if extra_flag not in current_val:
                new_val = f"{current_val} {extra_flag}".strip()
                misc_node.text = new_val
                print(f"  [{target_name}]: Appended '{extra_flag}'")
                is_modified = True
            else:
                print(f"  [{target_name}]: Flag exists, skipping")

    # If changes were made, backup and overwrite
    if is_modified:
        # Create backup
        #shutil.copy2(file_path, file_path + ".bak")
        
        # Keil requires long tags (short_empty_elements=False) for compatibility
        xml_content = ET.tostring(root, encoding='utf-8', method='xml', 
                                  xml_declaration=False, 
                                  short_empty_elements=False).decode('utf-8')
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(first_line + "\n" + xml_content)
        print(f"  Saved changes to: {file_path}")

def scan_directory(base_dir):

    """Recursively search and fix projects within 'SampleCode' folders."""
    print(f"Scanning/Fixing 'SampleCode' projects under: {os.path.abspath(base_dir)}...")
    search_pattern = os.path.join(base_dir, "**", "*.uvprojx")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]
    if not target_files:
        print("No SampleCode projects found.")
        return
    for f in target_files:
        print(f"\nChecking: {f}")
        append_misc_controls(f, EXTRA_MISCCONTROLS)

    print("\nScan and Append complete.")


if __name__ == "__main__":
    scan_directory('.')