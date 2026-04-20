# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mateo/esp/esp-idf/components/bootloader/subproject"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/tmp"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/src"
  "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mateo/Desktop/pwrinspace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
