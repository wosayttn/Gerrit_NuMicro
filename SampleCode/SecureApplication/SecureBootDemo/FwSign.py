#!/usr/bin/env python3
import sys
import os
import subprocess

# Usage: FwSign.py bin_file key_file [fw_ver]

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} bin_file key_file [fw_ver]")
        sys.exit(1)

    bin_file = sys.argv[1]
    key_file = sys.argv[2]
    fw_ver = sys.argv[3] if len(sys.argv) > 3 else "0"

    call_path = os.getcwd()
    script_path = os.path.dirname(os.path.abspath(__file__))
    bin_basename = os.path.splitext(os.path.basename(bin_file))[0]
    key_path = os.path.dirname(key_file)
    key_basename = os.path.splitext(os.path.basename(key_file))[0]

    imgtool_path = os.path.join(script_path, '../../../Tool/imgtool.py')

    # Generate ECC P-256 private key if not exist
    if not os.path.isfile(key_file):
        print(f"[INFO] Generating private key: {key_file}")
        subprocess.check_call([
            sys.executable, imgtool_path, 'keygen', '-k', key_file, '-t', 'ecdsa-p256'
        ])

    # Generate ECC P-256 public key if not exist
    pubkey_c = os.path.join(key_path, f"{key_basename}_pub.c")
    if not os.path.isfile(pubkey_c):
        print(f"[INFO] Generating public key: {pubkey_c}")
        with open(pubkey_c, 'w') as f:
            subprocess.check_call([
                sys.executable, imgtool_path, 'getpub', '-k', key_file
            ], stdout=f)

    # Sign image
    signed_bin = os.path.join(script_path, f"{bin_basename}_sc{fw_ver}_signed.bin")
    print(f"[INFO] Signing image: {bin_file} -> {signed_bin}")
    subprocess.check_call([
        sys.executable, imgtool_path, 'sign',
        '-k', key_file,
        '--public-key-format', 'hash',
        '--align', '4',
        '-v', '1.2.3+10',
        '-H', '0x400',
        '--pad-header',
        '-S', '0x40000',
        '-s', str(fw_ver),
        bin_file,
        signed_bin
    ])
    print("[INFO] Done.")

if __name__ == "__main__":
    main()
