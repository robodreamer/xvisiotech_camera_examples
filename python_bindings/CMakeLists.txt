# Python - always find Python regardless of build system
find_package(Python REQUIRED COMPONENTS Interpreter Development)

# Nanobind discovery
execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    RESULT_VARIABLE nanobind_result
    OUTPUT_VARIABLE nanobind_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(nanobind_result AND NOT nanobind_result EQUAL 0)
    message(FATAL_ERROR "Failed to get Nanobind CMake directory! Make sure nanobind is installed: pip install nanobind")
endif()

find_package(nanobind REQUIRED CONFIG PATHS "${nanobind_cmake_dir}")

# Build the nanobind module
nanobind_add_module(_xvisio_impl
    NB_STATIC
    LTO
    NOMINSIZE
    src/bindings.cpp
)

# Link with core library
target_link_libraries(_xvisio_impl PRIVATE
    xvisio_core
)

# For inplace editable installs, copy extension to python/xvisio directory
add_custom_command(TARGET _xvisio_impl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:_xvisio_impl>
        "${CMAKE_SOURCE_DIR}/python/xvisio/"
    COMMENT "Copying extension module to python/xvisio for editable install"
)

# Set RPATH so Python can find libxvisio_core.so
# Library is installed at xvisio/lib/, module at xvisio/, so use $ORIGIN/lib
set_target_properties(_xvisio_impl PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/lib"
    BUILD_RPATH "$ORIGIN:$<TARGET_FILE_DIR:xvisio_core>"
)

# Generate Python type stubs (non-fatal - stub generation may fail if dependencies aren't available)
# The module will work fine without .pyi files - they're just for IDE autocomplete
# Note: Module is copied to python/xvisio/ for editable installs, so stubgen needs PYTHONPATH
add_custom_command(
    TARGET _xvisio_impl POST_BUILD
    COMMAND sh -c "LD_LIBRARY_PATH=/usr/lib:$$LD_LIBRARY_PATH PYTHONPATH=\"${CMAKE_SOURCE_DIR}/python:$$PYTHONPATH\" ${Python_EXECUTABLE} -m nanobind.stubgen -q -i \"${CMAKE_SOURCE_DIR}/python/xvisio\" -M py.typed -m _xvisio_impl -o \"${CMAKE_SOURCE_DIR}/python/xvisio/__init__.pyi\" && touch \"${CMAKE_SOURCE_DIR}/python/xvisio/py.typed\" || echo 'Stub generation skipped (dependencies may be missing)'"
    COMMENT "Generating Python type stubs (non-fatal)"
    VERBATIM
)

# Install the nanobind module
install(TARGETS _xvisio_impl
    LIBRARY DESTINATION .
    RUNTIME DESTINATION .
)

# Install Python type stubs if generated
if(TARGET _xvisio_stub)
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/__init__.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/py.typed
        DESTINATION .
        OPTIONAL
    )
endif()

