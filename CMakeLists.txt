cmake_minimum_required(VERSION 3.16)
project(xvisio VERSION 0.1.0)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create core library
add_subdirectory(cpp_core)

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python_bindings)
endif()

# Install examples directory for pip-installed users
install(DIRECTORY examples/
    DESTINATION share/xvisio/examples
    FILES_MATCHING
        PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)
