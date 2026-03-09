import xml.etree.ElementTree as ET
import re
import os
import requests
import glob
import shutil

def to_int(hex_val):
    """將十六進位字串轉換為整數以進行準確比對"""
    try:
        if isinstance(hex_val, str):
            return int(hex_val, 16)
    except (ValueError, TypeError):
        return None
    return None

def download_pdsc(pack_id, pack_url, download_dir="."):
    """下載 PDSC 檔案"""
    parts = pack_id.split('.')
    if len(parts) < 2: return None
    pdsc_filename = f"{parts[0]}.{parts[1]}.pdsc"
    save_path = os.path.join(download_dir, pdsc_filename)
    if os.path.exists(save_path): return save_path
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
    """解析 PDSC 配置"""
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
                        pdsc_mem[m_id] = {"s": mem.get('start'), "z": mem.get('size')}
            curr = parent_map.get(curr)
        return pdsc_mem
    except:
        return None

def process_single_project(uvprojx_path):
    """修復專案並強制執行標籤完整格式"""
    try:
        # 讀取第一行以保留原始 Header
        with open(uvprojx_path, 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()
            if not first_line.startswith('<?xml'):
                first_line = '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>'
        
        tree = ET.parse(uvprojx_path)
        root = tree.getroot()
    except:
        return

    is_modified = False
    check_list = ['IROM1', 'IROM2', 'IROM3', 'IRAM1', 'IRAM2', 'IRAM3']

    for target in root.findall(".//Target"):
        t_name = target.find("TargetName").text
        device_node = target.find(".//Device")
        pack_id_node = target.find(".//PackID")
        pack_url_node = target.find(".//PackURL")
        cpu_node = target.find(".//Cpu")

        if any(x is None for x in [device_node, pack_id_node, pack_url_node, cpu_node]):
            continue

        pdsc_file = download_pdsc(pack_id_node.text, pack_url_node.text)
        truth = parse_pdsc_memory(pdsc_file, device_node.text)
        if not truth: continue

        new_cpu_text = cpu_node.text

        for m_id in check_list:
            p_val = truth.get(m_id)
            if not p_val: continue

            search_id = m_id if m_id not in ["IROM1", "IRAM1"] else m_id.replace("1", "")
            pattern = rf"({search_id})\((0x[0-9a-fA-F]+),(0x[0-9a-fA-F]+)\)"
            match = re.search(pattern, new_cpu_text)

            if match:
                if to_int(match.group(2)) != to_int(p_val['s']) or to_int(match.group(3)) != to_int(p_val['z']):
                    print(f"  [FIX] {uvprojx_path} | Target: {t_name} | Updating {m_id}")
                    replacement = f"{search_id}({p_val['s']},{p_val['z']})"
                    new_cpu_text = new_cpu_text.replace(match.group(0), replacement)
                    is_modified = True
            else:
                print(f"  [ADD] {uvprojx_path} | Target: {t_name} | Adding {m_id}")
                new_cpu_text += f" {search_id}({p_val['s']},{p_val['z']})"
                is_modified = True

        if is_modified:
            cpu_node.text = new_cpu_text

    if is_modified:
        shutil.copy2(uvprojx_path, uvprojx_path + ".bak")
        
        # 關鍵：使用 short_empty_elements=False 強制展開標籤
        # xml_declaration=False 是為了手動控制 header
        xml_content = ET.tostring(root, 
                                  encoding='utf-8', 
                                  method='xml', 
                                  xml_declaration=False, 
                                  short_empty_elements=False).decode('utf-8')
        
        with open(uvprojx_path, 'w', encoding='utf-8') as f:
            f.write(first_line + "\n" + xml_content)
        print(f"  [DONE] Saved changes to {uvprojx_path}")

def scan_directory(base_dir):
    """掃描 SampleCode"""
    search_pattern = os.path.join(base_dir, "**", "*.uvprojx")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]
    for f in target_files:
        process_single_project(f)

if __name__ == "__main__":
    scan_directory(".")