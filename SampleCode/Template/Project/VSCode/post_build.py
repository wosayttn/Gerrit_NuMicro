import sys
import platform

print('Build system: ' + platform.platform())
print(f"Python version: {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
print('+=======================================================+')
print('+            Execute Post-build script                  +')
print('+=======================================================+')

if len(sys.argv) > 1:
    print('Parameter is ' + sys.argv[1])
else:
    print('No parameter was provided.')