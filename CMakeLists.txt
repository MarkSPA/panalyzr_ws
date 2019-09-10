project(GromitLP)
cmake_minimum_required(VERSION 3.6)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
option(LAPIS_STATIC "Build lapis as a static library" OFF)

set(FWDIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Directory where the apppack build lives")
set(LIBUSBDIR "$ENV{LIBUSBDIR}" CACHE PATH "Path to LibUSB build directory")

set(LIBTYPE SHARED)
if (LAPIS_STATIC)
set(LIBTYPE STATIC)
set(LIBUSBNAME libusb-1.0.a)
endif()

IF (WIN32)
add_definitions(-D_WIN32_WINNT=0x0600 -DWIN32_LEAN_AND_MEAN -DNOMINMAX)
IF (CMAKE_CL_64)
set(LIBUSBLIBDIR "x64/Release/dll")
ELSE()
set(LIBUSBLIBDIR "Win32/Release/dll")
ENDIF()
ELSE()
if(CMAKE_COMPILER_IS_GNUCC AND NOT LAPIS_STATIC)
execute_process(COMMAND "${CMAKE_CURRENT_LIST_DIR}/../glibc.sh")
add_compile_options(-include "${CMAKE_CURRENT_BINARY_DIR}/glibc.h")
endif()
ENDIF()
set(LIBUSB_INCLUDE_DIR "D:/libusb/include/libusb-1.0")
set(LIBUSB_LIBRARIES "D:/libusb/MS64/static/libusb-1.0.lib")
FIND_PATH(LIBUSB_INCLUDE_DIR libusb.h REQUIRED
    HINTS ${LIBUSBDIR}
    #PATH_SUFFIXES libusb libusb-1.0)
	PATH_SUFFIXES libusb libusb-1.0)
FIND_LIBRARY(LIBUSB_LIBRARIES
    NAMES ${LIBUSBNAME} libusb-1.0 usb-1.0
    HINTS ${LIBUSBDIR} ../../thirdparty/libusb/libusb/.libs/
    PATH_SUFFIXES ${LIBUSBLIBDIR} libusb/.libs install/lib
    REQUIRED)
find_package( Threads )

include_directories(../RFCLib)
add_subdirectory(../thirdparty/fmt fmt EXCLUDE_FROM_ALL)
set_property(TARGET fmt PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(liblapis ${LIBTYPE} ../RFCLib/CRC32.cpp ../RFCLib/filepackloader.cpp ../RFCLib/morephtransport.cpp ../RFCLib/morephsearch.cpp ../RFCLib/morephtransport_impl.cpp ../RFCLib/morephsearch_impl.cpp  ../RFCLib/memmap.cpp ../RFCLib/morephinterface.cpp gromitinterface.cpp
    FPGAregs.cpp USBdata.cpp WhitelistMem.cpp lapis.cpp "${FWDIR}/gromit.apppack")
target_include_directories(liblapis PRIVATE ${LIBUSB_INCLUDE_DIR})
target_link_libraries(liblapis PRIVATE fmt ${LIBUSB_LIBRARIES})
target_compile_definitions(liblapis PRIVATE LAPIS_DLL_EXPORTS RFCOMM_STATIC ASIO_STANDALONE MOREPH_USBONLY)
set_target_properties(liblapis PROPERTIES PREFIX "")
set_target_properties(liblapis PROPERTIES INSTALL_RPATH "\$ORIGIN")
set_target_properties(liblapis PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
if (LAPIS_STATIC)
target_compile_definitions(liblapis PRIVATE LAPIS_STATIC)
endif()

function(add_apppack target apppack)
IF (WIN32)
file(TO_NATIVE_PATH "${apppack}" pack)
add_custom_command(TARGET ${target} POST_BUILD
                   COMMAND copy /b \"$<SHELL_PATH:$<TARGET_FILE:${target}>>\"+\"${pack}\" \"$<SHELL_PATH:$<TARGET_FILE:${target}>>\")
ELSE()
add_custom_command(TARGET ${target} POST_BUILD
                   COMMAND cat ${apppack} >> $<SHELL_PATH:$<TARGET_FILE:${target}>>)
ENDIF()
set_property(TARGET ${target} PROPERTY LINK_DEPENDS "${apppack}")
endfunction()

add_apppack(liblapis "${FWDIR}/gromit.apppack")

get_directory_property(PARENT_DIR PARENT_DIRECTORY)
if(NOT PARENT_DIR) # Standalone
add_executable(lapis-demo main.cpp)
#set_target_properties(lapis-demo PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE) # CMake >= 3.13
set_target_properties(lapis-demo PROPERTIES INSTALL_RPATH "\$ORIGIN")
set_target_properties(lapis-demo PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
if (LAPIS_STATIC AND NOT MSVC)
target_link_libraries(lapis-demo PRIVATE -static)
add_apppack(lapis-demo "${FWDIR}/gromit.apppack")
endif()
target_link_libraries(lapis-demo PRIVATE liblapis fmt)
if (NOT WIN32)
target_link_libraries(lapis-demo PRIVATE dl Threads::Threads)
endif()
endif()