import xml.etree.ElementTree as ET
import re
import os
import requests
import glob
import shutil

def to_int(hex_val):
    """Convert hex string to integer for accurate comparison."""
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
    """Extract all memory layouts from PDSC, handling hierarchy inheritance."""
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
        
        # FIX: Explicit check to avoid DeprecationWarning
        if device_node is None: return None
        
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
        # Preserve original header
        with open(uvprojx_path, 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()
            if not first_line.startswith('<?xml'):
                first_line = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'
        
        tree = ET.parse(uvprojx_path)
        root = tree.getroot()
    except:
        return

    project_header_printed = False
    is_modified = False
    check_list = ['IROM1', 'IROM2', 'IROM3', 'IRAM1', 'IRAM2', 'IRAM3']

    for target in root.findall(".//Target"):
        t_name_node = target.find("TargetName")
        t_name = t_name_node.text if t_name_node is not None else "Unknown"
        
        device_node = target.find(".//Device")
        pack_id_node = target.find(".//PackID")
        pack_url_node = target.find(".//PackURL")
        cpu_node = target.find(".//Cpu")

        # FIX: Explicit None checks
        if any(x is None for x in [device_node, pack_id_node, pack_url_node, cpu_node]):
            continue

        device_name = device_node.text
        pdsc_file = download_pdsc(pack_id_node.text, pack_url_node.text)
        if not pdsc_file: continue

        truth = parse_pdsc_memory(pdsc_file, device_name)
        if not truth: continue

        # Current project memory parsing
        cpu_text = cpu_node.text or ""
        matches = re.findall(r"(\w+)\((0x[0-9a-fA-F]+),(0x[0-9a-fA-F]+)\)", cpu_text)
        proj_mem = {}
        for m in matches:
            m_id = m[0]
            if m_id == "IROM": m_id = "IROM1"
            if m_id == "IRAM": m_id = "IRAM1"
            proj_mem[m_id] = {"v_s": to_int(m[1]), "v_z": to_int(m[2]), "r_s": m[1], "r_z": m[2]}

        target_errors = []
        new_cpu_text = cpu_text

        for m_id in check_list:
            p_val = truth.get(m_id)
            if not p_val: continue 
            
            u_val = proj_mem.get(m_id)
            
            # Use shorter ID for Cpu string matching (IROM1 -> IROM)
            search_id = m_id if m_id not in ["IROM1", "IRAM1"] else m_id.replace("1", "")
            
            if not u_val:
                target_errors.append(f"    [MISSING] {m_id}: Added {p_val['raw_start']}")
                new_cpu_text += f" {search_id}({p_val['raw_start']},{p_val['raw_size']})"
                is_modified = True
            elif u_val['v_s'] != p_val['val_start'] or u_val['v_z'] != p_val['val_size']:
                target_errors.append(
                    f"    [FIXED] {m_id} mismatch!\n"
                    f"            From: {u_val['r_s']}, {u_val['r_z']}\n"
                    f"            To:   {p_val['raw_start']}, {p_val['raw_size']}"
                )
                # Regex replace specific entry
                pattern = rf"{search_id}\({u_val['r_s']},{u_val['r_z']}\)"
                replacement = f"{search_id}({p_val['raw_start']},{p_val['raw_size']})"
                new_cpu_text = re.sub(pattern, replacement, new_cpu_text)
                is_modified = True

        if target_errors:
            cpu_node.text = new_cpu_text
            # Insert Headers at the START of the list
            target_errors.insert(0, f"  Target: {t_name} ({device_name})")
            if not project_header_printed:
                target_errors.insert(0, f"\n[!] ISSUES FOUND & FIXED IN: {uvprojx_path}")
                project_header_printed = True
            
            for err in target_errors:
                print(err)

    if is_modified:
        shutil.copy2(uvprojx_path, uvprojx_path + ".bak")
        # Ensure long tags for Keil compatibility
        xml_content = ET.tostring(root, encoding='utf-8', method='xml', 
                                  xml_declaration=False, 
                                  short_empty_elements=False).decode('utf-8')
        with open(uvprojx_path, 'w', encoding='utf-8') as f:
            f.write(first_line + "\n" + xml_content)

def scan_directory(base_dir):
    """Recursively search and fix projects within 'SampleCode' folders."""
    print(f"Scanning/Fixing 'SampleCode' projects under: {os.path.abspath(base_dir)}...")
    search_pattern = os.path.join(base_dir, "**", "*.uvprojx")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]

    if not target_files:
        print("No SampleCode projects found.")
        return

    for f in target_files:
        process_single_project(f)
    print("\nScan and Auto-Fix complete.")

if __name__ == "__main__":
    scan_directory(".")