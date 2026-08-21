"""Append '-Wmaybe-uninitialized' to Eclipse .cproject compiler options."""

import xml.etree.ElementTree as ET
import os
import glob
import uuid
import sys

# GCC warning flag for potential uninitialized variable usage
EXTRA_CFLAGS = '-Wmaybe-uninitialized'

def append_misc_controls(file_path, extra_flag):
    if not os.path.exists(file_path):
        print(f"Error: not found {file_path}")
        return

    try:
        # Preserve Eclipse .cproject headers, including <?fileVersion ...?> and <?xml ...?>
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract all declarations before <cproject> (xml version and fileVersion)
        idx = content.find('<cproject')
        if idx != -1:
            header = content[:idx].strip()
        else:
            header = '<?xml version="1.0" encoding="UTF-8" standalone="no"?>'
            
        tree = ET.parse(file_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing {file_path}: {e}")
        return

    is_modified = False

    # In Eclipse CDT structure, find all C/C++ compiler <tool> nodes
    for tool in root.findall(".//tool"):
        super_class = tool.get("superClass", "")
        # Check whether this node is a compiler tool
        if "tool.c.compiler" in super_class or "tool.cpp.compiler" in super_class:
            
            # Determine C or C++ to select the matching 'other flags' superClass
            is_c = "tool.c.compiler" in super_class
            opt_super = "ilg.gnuarmeclipse.managedbuild.cross.option.c.compiler.other" if is_c else "ilg.gnuarmeclipse.managedbuild.cross.option.cpp.compiler.other"
            
            # Check if the "Other compiler flags" option already exists
            other_opt = None
            for opt in tool.findall("option"):
                if opt.get("superClass") == opt_super:
                    other_opt = opt
                    break
            
            if other_opt is not None:
                # If present, append the flag when missing
                current_val = other_opt.get("value", "")
                if extra_flag not in current_val:
                    new_val = f"{current_val} {extra_flag}".strip()
                    other_opt.set("value", new_val)
                    is_modified = True
                    print(f"  Appended '{extra_flag}' to {super_class}")
                else:
                    print(f"  Flag exists in {super_class}, skipping")
            else:
                # If missing, create a new option node for extra compiler flags
                new_id = f"{opt_super}.{uuid.uuid4().hex[:8]}"
                ET.SubElement(tool, "option", {
                    "id": new_id,
                    "name": "Other compiler flags",
                    "superClass": opt_super,
                    "useByScannerDiscovery": "false",
                    "value": extra_flag,
                    "valueType": "string"
                })
                is_modified = True
                print(f"  Added '{extra_flag}' to {super_class}")

    # Save file only when changes were made
    if is_modified:
        # Eclipse prefers short empty elements, e.g. <option ... />
        xml_content = ET.tostring(root, encoding='utf-8', method='xml', 
                                  xml_declaration=False, 
                                  short_empty_elements=True).decode('utf-8')
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(header + "\n" + xml_content)
        print(f"  Saved changes to: {file_path}")

def scan_directory(base_dir):
    """Recursively search and fix GCC projects within the provided folder."""
    print(f"Scanning/Fixing GCC projects under: {os.path.abspath(base_dir)}...")
    
    # For GCC projects, search hidden .cproject files
    search_pattern = os.path.join(base_dir, "**", ".cproject")
    target_files = glob.glob(search_pattern, recursive=True)
    
    if not target_files:
        print("No .cproject projects found.")
        return
        
    for f in target_files:
        print(f"\nChecking: {f}")
        append_misc_controls(f, EXTRA_CFLAGS)

    print("\nScan and Append complete.")

if __name__ == "__main__":
    base_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    scan_directory(base_dir)
