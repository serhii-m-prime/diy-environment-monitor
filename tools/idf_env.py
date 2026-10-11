"""Use locally supplied components in both application and bootloader CMake."""
import os

# PlatformIO configures the bootloader separately, outside the application's
# CMake process. Both must avoid optional remote Arduino service components.
os.environ["IDF_COMPONENT_MANAGER"] = "0"
