import xml.etree.ElementTree as ET
import os
import glob
import sys

# IAR Compiler flag for no-unaligned-access
EXTRA_FLAG = '--no_unaligned_access'

def append_extra_options(file_path, extra_flag):
    if not os.path.exists(file_path):
        print(f"Error: not found {file_path}")
        return

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing {file_path}: {e}")
        return

    is_modified = False

    # Find the ICCARM compiler settings
    # Path: project/configuration/settings[name='ICCARM']/data/option[name='IExtraOptions']
    for config in root.findall(".//configuration"):
        config_name = config.find("name")
        config_name_text = config_name.text if config_name is not None else "Unknown"

        for settings in config.findall("settings"):
            settings_name = settings.find("name")
            if settings_name is not None and settings_name.text == "ICCARM":
                # Found ICCARM section
                for data in settings.findall("data"):
                    for option in data.findall("option"):
                        option_name = option.find("name")
                        if option_name is not None and option_name.text == "IExtraOptions":
                            # Found IExtraOptions
                            state_elem = option.find("state")
                            if state_elem is not None:
                                current_val = state_elem.text if state_elem.text else ""
                                if extra_flag not in current_val:
                                    new_val = f"{current_val} {extra_flag}".strip()
                                    state_elem.text = new_val
                                    is_modified = True
                                    print(f"  [{config_name_text}]: Appended '{extra_flag}'")
                                else:
                                    print(f"  [{config_name_text}]: Flag exists, skipping")

    if is_modified:
        # IAR projects are typically in XML format without declaration
        xml_content = ET.tostring(root, encoding='utf-8', method='xml',
                                  xml_declaration=False,
                                  short_empty_elements=False).decode('utf-8')

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n' + xml_content)
        print(f"  Saved changes to: {file_path}")

def scan_directory(base_dir):
    print(f"Scanning/Fixing 'SampleCode' IAR projects under: {os.path.abspath(base_dir)}...")
    search_pattern = os.path.join(base_dir, "**", "*.ewp")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]
    if not target_files:
        print("No SampleCode IAR projects found.")
        return
    for f in target_files:
        print(f"\nChecking: {f}")
        append_extra_options(f, EXTRA_FLAG)

    print("\nScan and Append complete.")

if __name__ == "__main__":
    base_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    scan_directory(base_dir)
