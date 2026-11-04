import sys
import xml.etree.ElementTree as ET
import hashlib

def indent(elem, level=0):
    """Add indentation to ElementTree XML for pretty printing"""
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for e in elem:
            indent(e, level+1)
        if not e.tail or not e.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

# Check command-line arguments
if len(sys.argv) < 2:
    print(f"Usage: python {sys.argv[0]} <xml file>")
    sys.exit(1)

xml_path = sys.argv[1]
tree = ET.parse(xml_path)
root = tree.getroot()
changed = False

for target in root.findall('./Targets/Target'):
    name_elem = target.find('TargetName')
    if name_elem is not None and name_elem.text:
        name = name_elem.text
        if len(name) >= 32:
            hash_str = hashlib.sha1(name.encode()).hexdigest()[:16]
            new_name = f"Target_{hash_str}"
            print(f"TargetName '{name}' -> '{new_name}'")
            name_elem.text = new_name
            changed = True

if changed:
    indent(root)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    print(f"--> uv2_helper: updated {xml_path}.")

