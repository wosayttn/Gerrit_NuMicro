import os
import glob
import sys
import importlib

FLAGS = {
    "AC6": "-mno-unaligned-access",
    "GCC": "-mno-unaligned-access"
}

def get_yaml_loader():
    try:
        yaml_module = importlib.import_module("ruamel.yaml")
    except ImportError:
        print("    [ERROR] Missing dependency: ruamel.yaml")
        return None
    return yaml_module.YAML()

def update_cproject_yml(file_path):
    yaml = get_yaml_loader()
    if yaml is None:
        return False
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 1000

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.load(f)

        if not data:
            return False

        modified = False
        root = data.get('project') if 'project' in data else data

        if 'setups' not in root or not isinstance(root['setups'], list):
            return False

        for setup in root['setups']:
            if 'misc' not in setup:
                continue

            misc_list = setup['misc']
            if not isinstance(misc_list, list):
                continue

            for compiler, target_flag in FLAGS.items():
                entry = next((item for item in misc_list if item.get('for-compiler') == compiler), None)

                if entry is None:
                    misc_list.append({
                        'for-compiler': compiler,
                        'C-CPP': [target_flag]
                    })
                    modified = True
                    print(f"    [+] {compiler}: Created C-CPP entry")
                    continue

                key = 'C-CPP'
                if key not in entry:
                    entry[key] = []

                if isinstance(entry[key], list):
                    if target_flag not in entry[key]:
                        entry[key].append(target_flag)
                        modified = True
                        print(f"    [+] {compiler}: Appended to {key}")
                else:
                    if target_flag not in str(entry[key]):
                        entry[key] = f"{entry[key]} {target_flag}".strip()
                        modified = True
                        print(f"    [+] {compiler}: Appended to {key}")

        if modified:
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f)
            return True

    except Exception as e:
        print(f"    [ERROR] {file_path}: {e}")
    return False

def scan_directory(base_dir):
    print(f"Force-Scanning projects under: {os.path.abspath(base_dir)}")
    search_pattern = os.path.join(base_dir, "**", "*.cproject.y*ml")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f]

    for f in target_files:
        print(f"\nChecking: {f}")
        if not update_cproject_yml(f):
            print("    Status: Already up to date.")

if __name__ == "__main__":
    base_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    scan_directory(base_dir)