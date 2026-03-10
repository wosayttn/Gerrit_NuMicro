import xml.etree.ElementTree as ET
import re
import os
import requests
import glob
import shutil

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

    project_header_printed = False
    check_list = ['IROM1', 'IROM2', 'IROM3', 'IRAM1', 'IRAM2', 'IRAM3']

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
            if not project_header_printed:
                print(f"\n[!] ISSUES FOUND IN: {uvprojx_path}")
                project_header_printed = True
            print(f"  Target: {t_name} ({device_name})")
            for err in target_errors:
                print(err)

def scan_directory(base_dir):
    """Recursively search and fix projects within 'SampleCode' folders."""
    print(f"Scanning for issues in 'SampleCode' folders under: {os.path.abspath(base_dir)}...")
    search_pattern = os.path.join(base_dir, "**", "*.uvprojx")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]

    if not target_files:
        print("No SampleCode projects found.")
        return

    for f in target_files:
        process_single_project(f)
    print("\nScan complete.")

if __name__ == "__main__":
    scan_directory(".")