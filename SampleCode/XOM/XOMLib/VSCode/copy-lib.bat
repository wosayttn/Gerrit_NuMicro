@echo on

@echo +=======================================================+
@echo +         Execute Post-build bash script                +
@echo +=======================================================+
set input_path=%~1
set output_path=%~2

:: Replace slash with backslash to get correct path in batch script
set input_path=%input_path:/=\%
set output_path=%output_path:/=\%

copy "%input_path%" "%output_path%"
@echo off
