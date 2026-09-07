# Requirement

### **[Python for Window](https://www.python.org/downloads/windows/)**

### **[vcpkg using CLI](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/installation.md#vcpkg---setup-using-cli)**

To ensure you have all the necessary CLI-building programs in your environment.

```bash
C:\Users\xxx\.vcpkg
C:\Users\xxx\AppData\Local\Arm\Packs\Nuvoton\NuMicro_DFP
C:\Users\xxx\AppData\Local\vcpkg
```

# TroubleShooting

## Must enable external network connection

## PowerShell permission

If you get the message,

```bash
cannot be loaded because running scripts is disabled on this system. For more information, see about_Execution_Policies at https:/go.microsoft.com/fwlink/?LinkID=135170
```

you can Open powershell as administrator permission, to execute below command to modify policy.

```bash
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine
```

## Enable Long Paths in Windows 10, Version 1607, and Later

[See also](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=powershell#enable-long-paths-in-windows-10-version-1607-and-later)

you can Open powershell as administrator permission, to execute below command to modify policy.

```bash
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
-Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
```

## Unpack compilers got "Error installing microsoft:compilers/arm-none-eabi-gcc - Running vcpkg internally returned a nonzero exit code: 1

Directly copy folder C:\Users\username\.vcpkg\artifacts\vcpkg-ce-default\compilers.arm.none.eabi.gcc to
  C:\Users\username\.vcpkg\downloads\artifacts\vcpkg-ce-default\

The folder structure should appear as follows.
 C:\Users\username\.vcpkg\downloads\artifacts\vcpkg-ce-default\
  compilers.arm.none.eabi.gcc\
   10.3.1-2021.10\
    arm-none-eabi\
    bin\
    lib\
    share\
    artifact.json

## Update specific registry

Due to old arm:tools/open-cmsis-pack/cmsis-toolbox version(v2.3.0) in older arm registry, it doesn't include new v2.4.0.
So, we need upgrade arm registry for importing new cmsis-toolbox version(v2.4.0)

```bash
PS Z:\<Path-to-M55M1BSP>\SampleCode\Template\Project\VSCode> vcpkg x-update-registry arm

warning: vcpkg-artifacts is experimental and may change at any time.
Updating registry data from arm
Updated arm. It contains 46 metadata files.

PS Z:\<Path-to-M55M1BSP>\SampleCode\Template\Project\VSCode> vcpkg activate
warning: vcpkg-artifacts is experimental and may change at any time.
Artifact                                Version        Status       Dependency Summary
microsoft:tools/kitware/cmake           3.25.2         installed               Kitware's cmake tool
microsoft:tools/ninja-build/ninja       1.10.2         installed               Ninja is a small build system with a focus on speed.
microsoft:compilers/arm-none-eabi-gcc   10.3.1-2021.10 installed               GCC compiler for ARM CPUs.
arm:tools/open-cmsis-pack/cmsis-toolbox 2.4.0          will install            Arm CMSIS-Toolbox

[1/4] microsoft:tools/kitware/cmake already installed.
[2/4] microsoft:tools/ninja-build/ninja already installed.
[3/4] microsoft:compilers/arm-none-eabi-gcc already installed.
[4/4] Installing arm:tools/open-cmsis-pack/cmsis-toolbox...
Downloading https://artifacts.keil.arm.com/cmsis-toolbox/2.4.0/cmsis-toolbox-windows-amd64.zip...
Unpacking c:\Users\Wayne\.vcpkg\downloads\tools.open-cmsis-pack.cmsis-toolbox-2.4.0-(f255c6bb4c35be6fb38a4bbf755da8b4c2ee70874ec2755c824bd2b2b0628f60).zip...
Activating: z:\<Path-to-M55M1BSP>\SampleCode\Template\Project\VSCode
```
