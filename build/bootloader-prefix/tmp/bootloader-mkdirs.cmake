# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/alaive/esp/esp-idf/components/bootloader/subproject"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/tmp"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/src"
  "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/alaive/Pulpit/PWrInSpace/TANWA-8-COM/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
