# ROM Binary Image Generation

This directory contains scripts that convert the project's Intel HEX image into
separate APROM and SPIM ROM binary images.

## Prerequisite

The scripts require the SRecord `srec_cat` tool. Download the appropriate
SRecord version for Windows or Linux from:

https://srecord.sourceforge.net/download.html

On Windows, place `srec_cat.exe` in this directory, next to
`gen_rom_bin.bat`. On Linux, place the Linux `srec_cat` executable in this
directory, next to `gen_rom_bin.sh`.

Before running either script, build the project and make sure the input HEX file
exists in the project's `Release` directory.

## Windows

Run the batch file from this directory:

```bat
gen_rom_bin.bat
```

The batch file reads:

```text
../Release/SPIM_DMM_RUN_CODE.hex
```

It creates the following files in this directory:

```text
SPIM_DMM_RUN_CODE_aprom.hex
SPIM_DMM_RUN_CODE_aprom.bin
SPIM_DMM_RUN_CODE_spim.hex
SPIM_DMM_RUN_CODE_spim.bin
```

## Linux

Make the shell script executable, then pass the project directory and project
name to it:

```sh
chmod +x gen_rom_bin.sh
./gen_rom_bin.sh <project-directory> <project-name>
```

For this sample, when running from the `tools` directory:

```sh
./gen_rom_bin.sh .. SPIM_DMM_RUN_CODE
```

The script reads `<project-directory>/Release/<project-name>.hex` and writes the
generated APROM and SPIM `.hex` and `.bin` files to that same `Release`
directory.
