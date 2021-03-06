SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR armv7l)

SET(CMAKE_C_COMPILER "$ENV{RASPI_TOOLCHAIN}/bin/arm-linux-gnueabihf-gcc")
SET(CMAKE_CXX_COMPILER "$ENV{RASPI_TOOLCHAIN}/bin/arm-linux-gnueabihf-g++")
SET(CMAKE_SYSROOT $ENV{RASPI_ROOTFS})
SET(CMAKE_FIND_ROOT_PATH $ENV{RASPI_ROOTFS})
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Workarounds:
# Not populated automaticaly
SET(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)

# rootfs/usr/include/arm-linux-gnueabihf not included
INCLUDE_DIRECTORIES(SYSTEM "${CMAKE_FIND_ROOT_PATH}/usr/include/${CMAKE_LIBRARY_ARCHITECTURE}")
LINK_DIRECTORIES("${CMAKE_FIND_ROOT_PATH}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")

# pkg config looks up packages on the hostsystem
set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_LIBDIR} "${CMAKE_FIND_ROOT_PATH}/usr/lib/pkgconfig:${CMAKE_FIND_ROOT_PATH}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT} "${CMAKE_FIND_ROOT_PATH}")
