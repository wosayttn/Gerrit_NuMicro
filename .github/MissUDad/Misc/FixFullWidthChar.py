import os
import sys
import io
import shutil

# Force UTF-8 output for terminal compatibility
if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# Mapping of common "Smart" or Full-width characters to standard ASCII
REPLACEMENT_MAP = {
    '\u2018': "'",  # Left single quote ‘
    '\u2019': "'",  # Right single quote ’
    '\u201c': '"',  # Left double quote “
    '\u201d': '"',  # Right double quote ”
    '\u3000': ' ',  # Full-width space
    '\uff0c': ',',  # Full-width comma ，
    '\uff1b': ';',  # Full-width semicolon ；
    '\uff08': '(',  # Full-width (
    '\uff09': ')',  # Full-width )
}

def fix_file_content(file_path):
    """
    Reads the file, replaces known non-ASCII characters, and overwrites if changed.
    """
    try:
        with open(file_path, 'r', encoding='utf-8', errors='replace') as f:
            original_content = f.read()

        new_content = original_content
        changes_made = []

        for char, replacement in REPLACEMENT_MAP.items():
            if char in new_content:
                count = new_content.count(char)
                new_content = new_content.replace(char, replacement)
                changes_made.append(f"Fixed {count} instances of {repr(char)}")

        if new_content != original_content:
            # Create backup
            shutil.copy2(file_path, file_path + ".bak")
            # Write fixed content
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)
            return changes_made
    except Exception as e:
        print(f"      [ERR] Could not process {file_path}: {e}")
    
    return None

def scan_and_fix(path):
    target_extensions = ('.c', '.h', '.cpp', '.hpp')
    files = []
    
    if os.path.isfile(path):
        files = [path]
    else:
        for root, _, filenames in os.walk(path):
            for filename in filenames:
                if filename.lower().endswith(target_extensions):
                    files.append(os.path.join(root, filename))

    print(f"Scanning and Auto-Fixing {len(files)} files...")
    print("=" * 60)

    fixed_count = 0
    for file_path in files:
        results = fix_file_content(file_path)
        if results:
            fixed_count += 1
            print(f"[FIXED] {file_path}")
            for msg in results:
                print(f"        -> {msg}")

    print("=" * 60)
    print(f"Done. Fixed {fixed_count} files. Backups created with .bak extension.")

if __name__ == "__main__":
    # WARNING: This will modify files. Ensure you have a git commit or backup first.
    target = sys.argv[1] if len(sys.argv) > 1 else "."
    scan_and_fix(target)