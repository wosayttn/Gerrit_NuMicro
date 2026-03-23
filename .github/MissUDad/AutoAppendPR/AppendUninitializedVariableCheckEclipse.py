import xml.etree.ElementTree as ET
import os
import glob
import uuid

# 將旗標修正為正確的 GCC 指令 -Wmaybe-uninitialized
EXTRA_CFLAGS = '-Wmaybe-uninitialized'

def append_misc_controls(file_path, extra_flag):
    if not os.path.exists(file_path):
        print(f"Error: not found {file_path}")
        return

    try:
        # 為了保留 Eclipse .cproject 檔案特有的 <?fileVersion ...?> 與 <?xml ...?> 標頭
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 擷取 <cproject 開始前的所有宣告 (包含 xml 版號與 fileVersion)
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

    # 在 Eclipse CDT 架構下，尋找所有 C 與 C++ 編譯器的 <tool> 節點
    for tool in root.findall(".//tool"):
        super_class = tool.get("superClass", "")
        # 確認此節點是否為編譯器
        if "tool.c.compiler" in super_class or "tool.cpp.compiler" in super_class:
            
            # 判斷是 C 還是 C++，以決定對應的 other option superClass
            is_c = "tool.c.compiler" in super_class
            opt_super = "ilg.gnuarmeclipse.managedbuild.cross.option.c.compiler.other" if is_c else "ilg.gnuarmeclipse.managedbuild.cross.option.cpp.compiler.other"
            
            # 尋找是否已經有 "Other compiler flags" 這個節點
            other_opt = None
            for opt in tool.findall("option"):
                if opt.get("superClass") == opt_super:
                    other_opt = opt
                    break
            
            if other_opt is not None:
                # 若存在，則檢查並將旗標附加於字串後方
                current_val = other_opt.get("value", "")
                if extra_flag not in current_val:
                    new_val = f"{current_val} {extra_flag}".strip()
                    other_opt.set("value", new_val)
                    is_modified = True
                    print(f"  Appended '{extra_flag}' to {super_class}")
                else:
                    print(f"  Flag exists in {super_class}, skipping")
            else:
                # 若原先不存在 "其它編譯旗標" 的屬性欄位，主動為它創建一個新的 option 節點
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

    # 若發生異動，重新存檔
    if is_modified:
        # Eclipse 偏好使用 short_empty_elements (例如 <option ... />)
        xml_content = ET.tostring(root, encoding='utf-8', method='xml', 
                                  xml_declaration=False, 
                                  short_empty_elements=True).decode('utf-8')
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(header + "\n" + xml_content)
        print(f"  Saved changes to: {file_path}")

def scan_directory(base_dir):
    """Recursively search and fix GCC projects within 'SampleCode' folders."""
    print(f"Scanning/Fixing 'SampleCode' GCC projects under: {os.path.abspath(base_dir)}...")
    
    # 針對 GCC 環境，將搜尋模式改為查找隱藏檔 .cproject
    search_pattern = os.path.join(base_dir, "**", ".cproject")
    target_files = [f for f in glob.glob(search_pattern, recursive=True) if "SampleCode" in f.split(os.sep)]
    
    if not target_files:
        print("No SampleCode .cproject projects found.")
        return
        
    for f in target_files:
        print(f"\nChecking: {f}")
        append_misc_controls(f, EXTRA_CFLAGS)

    print("\nScan and Append complete.")

if __name__ == "__main__":
    scan_directory('.')
