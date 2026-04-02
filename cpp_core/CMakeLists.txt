cmake_minimum_required(VERSION 3.16)
project(xvisio_core VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find xvsdk
set(XVSDK_INCLUDE_DIRS "/usr/include/xvsdk")
set(XVSDK_LIBRARIES "/usr/lib/libxvsdk.so")

if (NOT EXISTS "${XVSDK_LIBRARIES}")
    message(FATAL_ERROR "libxvsdk.so not found at ${XVSDK_LIBRARIES}. Please run 'pixi run host-setup' first.")
endif()

if (NOT EXISTS "${XVSDK_INCLUDE_DIRS}")
    message(FATAL_ERROR "xvsdk headers not found at ${XVSDK_INCLUDE_DIRS}. Please run 'pixi run host-setup' first.")
endif()

# Create core library
add_library(xvisio_core SHARED
    src/device.cpp
)

target_include_directories(xvisio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${XVSDK_INCLUDE_DIRS}
)

target_link_libraries(xvisio_core PUBLIC
    ${XVSDK_LIBRARIES}
    pthread
)

# For inplace editable installs, copy core library to python/xvisio directory
add_custom_command(TARGET xvisio_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:xvisio_core>
        "${CMAKE_SOURCE_DIR}/python/xvisio/"
    COMMENT "Copying core library to python/xvisio for editable install"
)

# Installation
install(TARGETS xvisio_core
    EXPORT xvisio_core_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT xvisio_core_targets
    FILE xvisio_core_targets.cmake
    NAMESPACE xvisio_core::
    DESTINATION lib/cmake/xvisio_core
)

