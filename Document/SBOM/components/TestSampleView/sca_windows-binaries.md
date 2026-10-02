# M2L31 Windows Binary Evidence

## Classification

- SBOM View: Test Sample SBOM
- Scope: Git-tracked Windows host tools, USB driver utilities, and redistributables
- Match policy: Exact SHA-256 only
- Database: MS00 binary-license-database.json

## Review Result

Each entry below is copied from the reviewed MS00 binary-license database by exact SHA-256. Filename-only matching is prohibited. The referenced vendor evidence must be shipped with the release package. A file change invalidates the match and requires renewed review.

| Artifact | Size | SHA-256 | Supplier | License | Evidence |
|---|---:|---|---|---|---|
| `SampleCode/ISP/ISP_DFU/WindowsDriver/amd64/WdfCoInstaller01011.dll` | 1629040 | `c7649879a10c9332fc0f9744c7e3224647aee9e7e62c7e21cf9e987462e3dd06` | Microsoft Corporation | Microsoft Software License Terms | `Document/SBOM/license-evidence/windowsdriver-x86-wdfcoinstaller01011-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/amd64/libusb0.dll` | 76384 | `4f18b5d2c28aa66b648c8683c6d09b52b92cbbee85984bbefad5f38a64bc2a14` | libusb-win32 project | LGPL-2.1-or-later | `Document/SBOM/license-evidence/windowsdriver-amd64-libusb0-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/amd64/libusb0_x86.dll` | 46592 | `5e84d13636fbce7869cddc8b20c7d83fa0063e98c319e8e5ab751edc9ee1da76` | libusb-win32 project | LGPL-2.1-or-later | `Document/SBOM/license-evidence/windowsdriver-amd64-libusb0-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/amd64/libusbK.dll` | 99128 | `592cd24bae321f1cb6cbe2f6e1bc5c05e279328e1c86814eb64ea1e89fdea188` | libusbK project | BSD-3-Clause OR GPL-3.0-only | `Document/SBOM/license-evidence/windowsdriver-amd64-libusbk-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/amd64/libusbK_x86.dll` | 84280 | `22803c719494f193d22519bfaff9484fecdcf1fadd6f082efd024fcee0b97ba4` | libusbK project | BSD-3-Clause OR GPL-3.0-only | `Document/SBOM/license-evidence/windowsdriver-amd64-libusbk-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/WdfCoInstaller01011.dll` | 1629040 | `c7649879a10c9332fc0f9744c7e3224647aee9e7e62c7e21cf9e987462e3dd06` | Microsoft Corporation | Microsoft Software License Terms | `Document/SBOM/license-evidence/windowsdriver-x86-wdfcoinstaller01011-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/install-filter.exe` | 46592 | `5e84d13636fbce7869cddc8b20c7d83fa0063e98c319e8e5ab751edc9ee1da76` | libusb-win32 project | LGPL-2.1-or-later | `Document/SBOM/license-evidence/windowsdriver-amd64-libusb0-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/libusb0.dll` | 46592 | `5e84d13636fbce7869cddc8b20c7d83fa0063e98c319e8e5ab751edc9ee1da76` | libusb-win32 project | LGPL-2.1-or-later | `Document/SBOM/license-evidence/windowsdriver-amd64-libusb0-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/libusb0_x86.dll` | 67680 | `00caca07869b19d10b370552ac7cc2f6f2ee246fc15db11650f6cd3f4ef9b666` | libusb-win32 project | LGPL-2.1-or-later | `Document/SBOM/license-evidence/windowsdriver-x86-libusb0-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/libusbK.dll` | 84280 | `22803c719494f193d22519bfaff9484fecdcf1fadd6f082efd024fcee0b97ba4` | libusbK project | BSD-3-Clause OR GPL-3.0-only | `Document/SBOM/license-evidence/windowsdriver-amd64-libusbk-x86-dll.md` |
| `SampleCode/ISP/ISP_DFU/WindowsDriver/x86/winusbcoinstaller2.dll` | 851176 | `ebe3b7708dd974ee87efed3113028d266af87ca8dbae77c47c6f7612824d3d6c` | Microsoft Corporation | Microsoft Software License Terms | `Document/SBOM/license-evidence/windowsdriver-x86-winusbcoinstaller2-dll.md` |
| `SampleCode/StdDriver/USBD_Micro_Printer/Windows driver/NuvPos58_Driver.exe` | 1299564 | `911e4c166fe9b06c8e2e31ab46b56ef662c43cba208d2088ca1d41c6d584e183` | Nuvoton Technology Corp. | Nuvoton Software License Agreement | `Document/SBOM/license-evidence/samplecode-usbd-micro-printer-nuvpos58-driver-exe.md` |
| `SampleCode/StdDriver/USBD_Printer_And_HID_Transfer/Windows driver/NUVPOS58_X64.exe` | 662163 | `ce0ed66a5db0a0c061fd92975939f48e93affad15f3f6e806d44b3e82078b3db` | Nuvoton Technology Corp. | Nuvoton Software License Agreement | `Document/SBOM/license-evidence/samplecode-usbd-printer-and-hid-transfer-nuvpos58-x64-exe.md` |
| `SampleCode/StdDriver/USBD_Printer_And_HID_Transfer/Windows driver/NUVPOS58_X86.exe` | 621575 | `76e4d01dbf17ca7525e3a9156bd7e6274c5db0fff88a25943c25c8d2bdd38d92` | Nuvoton Technology Corp. | Nuvoton Software License Agreement | `Document/SBOM/license-evidence/samplecode-usbd-printer-and-hid-transfer-nuvpos58-x86-exe.md` |
| `Tool/HIDTransferTest/Debug/HIDTransferTest.exe` | 745984 | `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a` | Nuvoton Technology Corp. | Apache-2.0 | `Document/SBOM/license-evidence/tool-hidtransfertest-exe.md` |
| `Tool/TK/TouchView.exe` | 3062784 | `0594f3caa3685bb11b6dbb146849de0c94fb0c309e1a543a7bfa3cf9d93f2c5f` | Nuvoton Technology Corp. | Nuvoton Software License Agreement | `Document/SBOM/license-evidence/tool-tk-touchview-exe.md` |
| `Tool/TK/TouchView_MP.exe` | 2779648 | `798717b616ae59f81bb7cb4428a653f424838ac305bdfccc75623e69cce53cac` | Nuvoton Technology Corp. | Nuvoton Software License Agreement | `Document/SBOM/license-evidence/tool-tk-touchview-mp-exe.md` |

## Machine-readable hash bindings

- SHA-256: `c7649879a10c9332fc0f9744c7e3224647aee9e7e62c7e21cf9e987462e3dd06`
- SHA-256: `4f18b5d2c28aa66b648c8683c6d09b52b92cbbee85984bbefad5f38a64bc2a14`
- SHA-256: `5e84d13636fbce7869cddc8b20c7d83fa0063e98c319e8e5ab751edc9ee1da76`
- SHA-256: `592cd24bae321f1cb6cbe2f6e1bc5c05e279328e1c86814eb64ea1e89fdea188`
- SHA-256: `22803c719494f193d22519bfaff9484fecdcf1fadd6f082efd024fcee0b97ba4`
- SHA-256: `00caca07869b19d10b370552ac7cc2f6f2ee246fc15db11650f6cd3f4ef9b666`
- SHA-256: `ebe3b7708dd974ee87efed3113028d266af87ca8dbae77c47c6f7612824d3d6c`
- SHA-256: `911e4c166fe9b06c8e2e31ab46b56ef662c43cba208d2088ca1d41c6d584e183`
- SHA-256: `ce0ed66a5db0a0c061fd92975939f48e93affad15f3f6e806d44b3e82078b3db`
- SHA-256: `76e4d01dbf17ca7525e3a9156bd7e6274c5db0fff88a25943c25c8d2bdd38d92`
- SHA-256: `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a`
