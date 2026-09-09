#!/usr/bin/env python3
import json
import subprocess
import os
import sys
import shutil

# Paths
WORKSPACE = os.environ.get("GITHUB_WORKSPACE", os.getcwd())

def find_tool(name, default_fallback):
    path = shutil.which(name)
    if path:
        return path
    for p in [f"/opt/bin/{name}"]:
        if os.path.exists(p):
            return p
    return default_fallback

SYFT_PATH = find_tool("syft", "/opt/bin/syft")
GRYPE_PATH = find_tool("grype", "/opt/bin/grype")

SBOM_PATH = os.path.join(WORKSPACE, "sbom.json")
CVE_PATH = os.path.join(WORKSPACE, "cve-report.json")
SCA_JSON_PATH = os.path.join(WORKSPACE, "sca-description.json")
SCA_MD_PATH = os.path.join(WORKSPACE, "sca-description.md")
MANIFEST_PATH = os.path.join(WORKSPACE, "Document", "manifest.json")

def load_manifest():
    if os.path.exists(MANIFEST_PATH):
        with open(MANIFEST_PATH, "r") as f:
            return json.load(f)
    return {}

def run_command(cmd, cwd=WORKSPACE):
    print(f"Running command: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Command failed with code {result.returncode}")
        print(f"Stderr: {result.stderr}")
        sys.exit(1)
    return result.stdout

def main():
    manifest = load_manifest()
    bsp_name = manifest.get("bsp_name", "NuMicro-M3351BSP")
    bsp_version = manifest.get("bsp_version", "v3.0.2")
    supplier = manifest.get("supplier", "Nuvoton Technology")
    
    # 1. Run Syft to get base SBOM
    print("Step 1: Running Syft to generate base CycloneDX SBOM...")
    syft_cmd = [SYFT_PATH, "dir:" + WORKSPACE, "-o", "cyclonedx-json"]
    base_sbom_str = run_command(syft_cmd)
    
    try:
        sbom = json.loads(base_sbom_str)
    except Exception as e:
        print(f"Failed to parse Syft output: {e}")
        sys.exit(1)
        
    # Ensure components list exists
    if "components" not in sbom:
        sbom["components"] = []
        
    # 2. Formulate and inject our custom third-party components from manifest.json
    print("Step 2: Injecting components from manifest.json into SBOM...")
    
    custom_components = []
    for comp in manifest.get("components", []):
        name = comp.get("name")
        version = comp.get("version")
        license_str = comp.get("license", "Unknown")
        purl = comp.get("purl")
        cpe = comp.get("cpe")
        description = comp.get("description", "")
        paths = comp.get("paths", [])
        
        licenses = []
        if license_str:
            if " " in license_str or "License" in license_str:
                licenses = [{"license": {"name": license_str}}]
            else:
                licenses = [{"license": {"id": license_str}}]
                
        properties = [
            {"name": "syft:package:foundBy", "value": "custom-thirdparty-cataloger"}
        ]
        for i, path in enumerate(paths):
            properties.append({"name": f"syft:location:{i}:path", "value": path})
            
        custom_components.append({
            "bom-ref": f"{purl}?package-id={name.lower()}",
            "type": "library",
            "name": name,
            "version": version,
            "description": description,
            "purl": purl,
            "cpe": cpe,
            "licenses": licenses,
            "properties": properties
        })
    
    # Avoid duplicating if they are somehow already present
    existing_purls = {c.get("purl") for c in sbom["components"] if c.get("purl")}
    for cc in custom_components:
        if cc["purl"] not in existing_purls:
            sbom["components"].append(cc)
            
    # Update main metadata component representing the whole BSP project
    if "metadata" not in sbom:
        sbom["metadata"] = {}
    
    sbom["metadata"]["component"] = {
        "bom-ref": f"pkg:generic/{bsp_name}@{bsp_version}?type=device",
        "type": "device",
        "name": bsp_name,
        "version": bsp_version,
        "supplier": {"name": supplier}
    }
    
    # Save the complete enriched SBOM
    with open(SBOM_PATH, "w") as f:
        json.dump(sbom, f, indent=2)
    print(f"Complete SBOM successfully written to {SBOM_PATH}")
    
    # 3. Run Grype scan on the enriched SBOM
    print("Step 3: Running Grype CVE scan on the enriched SBOM...")
    grype_cmd = [GRYPE_PATH, "sbom:" + SBOM_PATH, "-o", "json"]
    cve_report_str = run_command(grype_cmd)
    
    try:
        cve_report = json.loads(cve_report_str)
    except Exception as e:
        print(f"Failed to parse Grype output: {e}")
        sys.exit(1)
        
    with open(CVE_PATH, "w") as f:
        f.write(cve_report_str)
    print(f"Vulnerability report successfully written to {CVE_PATH}")
    
    # 4. Parse vulnerability details
    print("Step 4: Analyzing scan results and generating SCA description files...")
    vuln_summary = {
        "Critical": 0,
        "High": 0,
        "Medium": 0,
        "Low": 0,
        "Negligible": 0,
        "Unknown": 0
    }
    
    vulnerability_list = []
    
    matches = cve_report.get("matches", [])
    for match in matches:
        vulnerability = match.get("vulnerability", {})
        vuln_id = vulnerability.get("id")
        severity = vulnerability.get("severity", "Unknown").capitalize()
        if severity not in vuln_summary:
            severity = "Unknown"
        vuln_summary[severity] += 1
        
        artifact = match.get("artifact", {})
        pkg_name = artifact.get("name")
        pkg_version = artifact.get("version")
        pkg_purl = artifact.get("purl")
        
        related_vulns = match.get("relatedVulnerabilities", [])
        fix_state = "unknown"
        if related_vulns:
            fix_state = related_vulns[0].get("fix", {}).get("state", "unknown")

        vulnerability_list.append({
            "vuln_id": vuln_id,
            "severity": severity,
            "package_name": pkg_name,
            "package_version": pkg_version,
            "purl": pkg_purl,
            "description": vulnerability.get("description", "No description available."),
            "fix_state": fix_state
        })
        
    # Generate list of final components for the SCA description
    sca_components = []
    for c in sbom["components"]:
        licenses = []
        for lic_item in c.get("licenses", []):
            lic = lic_item.get("license", {})
            licenses.append(lic.get("id") or lic.get("name") or "Unknown")
            
        cve_count = sum(1 for v in vulnerability_list if v["package_name"] == c.get("name"))
        
        sca_components.append({
            "name": c.get("name"),
            "version": c.get("version"),
            "type": c.get("type"),
            "purl": c.get("purl"),
            "license": ", ".join(licenses) if licenses else "Unknown",
            "vulnerabilities_found": cve_count
        })
        
    # 5. Output SCA Description JSON
    sca_description = {
        "project_name": bsp_name,
        "project_version": bsp_version,
        "release_date": manifest.get("release_date", "2026-06-01"),
        "supplier": supplier,
        "total_components": len(sbom["components"]),
        "vulnerability_summary": vuln_summary,
        "components": sca_components,
        "vulnerabilities": vulnerability_list
    }
    
    with open(SCA_JSON_PATH, "w") as f:
        json.dump(sca_description, f, indent=2)
    print(f"SCA JSON description written to {SCA_JSON_PATH}")
    
    # 6. Output SCA Description MD (Beautiful human-readable Markdown)
    md_content = f"""# Software Composition Analysis (SCA) Report: {bsp_name}

This Software Composition Analysis (SCA) report details the third-party components, licensing, and security posture of the **{bsp_name}** ({bsp_version}) firmware distribution.

## 📊 Summary Dashboard

| Attribute | Value |
| :--- | :--- |
| **Project Name** | {bsp_name} |
| **Firmware Version** | {bsp_version} |
| **Release Date** | {sca_description['release_date']} |
| **Supplier / Vendor** | {supplier} |
| **Total Tracked Components** | {sca_description['total_components']} |
| **Vulnerability Status** | {'🔴 Action Required' if vuln_summary['Critical'] + vuln_summary['High'] > 0 else '🟢 Passed (No Critical/High Vulns)'} |

---

## 🛡️ Security Vulnerability Summary

Vulnerabilities detected by the SCA scanning engine (Grype) on the complete firmware SBOM:

*   🔴 **Critical**: `{vuln_summary['Critical']}`
*   🟠 **High**: `{vuln_summary['High']}`
*   🟡 **Medium**: `{vuln_summary['Medium']}`
*   🟢 **Low**: `{vuln_summary['Low']}`
*   🔵 **Negligible**: `{vuln_summary['Negligible']}`
*   ⚪ **Unknown**: `{vuln_summary['Unknown']}`

---

## 📦 Component Composition List

The following third-party components and local software libraries are tracked in the firmware package:

| Name | Version | Type | License | Vulnerabilities | Package URL (purl) |
| :--- | :--- | :--- | :--- | :---: | :--- |
"""
    
    for c in sca_components:
        md_content += f"| **{c['name']}** | `{c['version']}` | {c['type']} | {c['license']} | `{c['vulnerabilities_found']}` | `{c['purl']}` |\n"
        
    md_content += "\n---\n\n## 📝 Detailed Vulnerabilities List\n\n"
    
    if not vulnerability_list:
        md_content += "✨ **No vulnerabilities were detected in the audited components.**\n"
    else:
        md_content += "Below is the list of identified vulnerabilities mapped to components:\n\n"
        md_content += "| Severity | Vulnerability ID | Component | Version | Fix Status | Description |\n"
        md_content += "| :---: | :--- | :--- | :--- | :---: | :--- |\n"
        
        # Sort by severity priority
        severity_order = {"Critical": 0, "High": 1, "Medium": 2, "Low": 3, "Negligible": 4, "Unknown": 5}
        sorted_vulns = sorted(vulnerability_list, key=lambda x: severity_order.get(x["severity"], 99))
        
        for v in sorted_vulns:
            sev_emoji = {"Critical": "🔴", "High": "🟠", "Medium": "🟡", "Low": "🟢", "Negligible": "🔵", "Unknown": "⚪"}.get(v["severity"], "⚪")
            md_content += f"| {sev_emoji} {v['severity']} | `{v['vuln_id']}` | **{v['package_name']}** | `{v['package_version']}` | `{v['fix_state']}` | {v['description'][:150]}... |\n"
            
    with open(SCA_MD_PATH, "w") as f:
        f.write(md_content)
    print(f"Beautiful SCA Markdown report successfully written to {SCA_MD_PATH}")

    # 7. Pack reports into a single ZIP file
    print("Step 7: Packaging all generated reports into sca-reports.zip...")
    import zipfile
    zip_path = os.path.join(WORKSPACE, "sca-reports.zip")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zipf:
        for filepath in [SBOM_PATH, CVE_PATH, SCA_JSON_PATH, SCA_MD_PATH]:
            arcname = os.path.basename(filepath)
            zipf.write(filepath, arcname)
    print(f"Zip bundle successfully written to {zip_path}")
    print("\n🎉 SBOM and SCA generation completed successfully!")

if __name__ == "__main__":
    main()
