cmake_minimum_required(VERSION 3.22)
project(v5_pros_indexer LANGUAGES C CXX)

# ---- IDE / parsing settings (no cross-compiling) ----------------------------
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Typical PROS-style defines that many projects rely on for headers
add_compile_definitions(
        VEX_V5
        PROS_USE_SIMPLE_NAMES
        PROS_USE_LITERALS
        PROS_USE_TYPED_TASKS
)

# ---- Where the VS Code extension put the CLI/toolchain on macOS -------------
# These defaults match the PROS VS Code extension cache on macOS.
# If yours differs, edit these or set them in CLion (Preferences → CMake → Cache variables)
set(DEFAULT_PROS_BASE "$ENV{HOME}/Library/Application Support/Code/User/globalStorage/sigbots.pros/install")

set(PROS_TOOLCHAIN_DIR
        "${DEFAULT_PROS_BASE}/pros-toolchain-macos"
        CACHE PATH "Path to PROS toolchain directory (contains bin/, lib/gcc/arm-none-eabi/*, etc.)")

# Optional extra include roots if you have a separate SDK checkout (usually not needed)
set(PROS_SDK_INCLUDE
        ""
        CACHE PATH "Optional: path to PROS SDK headers, if installed separately")

# ---- Discover GCC include subdirs from the toolchain ------------------------
# CLion only needs headers; we find the right versioned include folders.
file(GLOB _GCC_VERSION_DIRS "${PROS_TOOLCHAIN_DIR}/lib/gcc/arm-none-eabi/*")
list(LENGTH _GCC_VERSION_DIRS _GCC_DIR_COUNT)
if(_GCC_DIR_COUNT GREATER 0)
    list(GET _GCC_VERSION_DIRS 0 _GCC_VER_DIR) # pick the first match
else()
    set(_GCC_VER_DIR "${PROS_TOOLCHAIN_DIR}/lib/gcc/arm-none-eabi/UNKNOWN")
endif()

# ---- Project sources ---------------------------------------------------------
# Index everything under src/ and include/ so CLion sees your code.
file(GLOB_RECURSE APP_CXX CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.cc" "${CMAKE_SOURCE_DIR}/src/*.cxx")
file(GLOB_RECURSE APP_C   CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/src/*.c")

add_library(pros_indexer STATIC ${APP_CXX} ${APP_C})
# Don’t actually build it by default; this is here for code model only.
set_target_properties(pros_indexer PROPERTIES EXCLUDE_FROM_ALL ON)

# ---- Include directories for IntelliSense / code model ----------------------
# Your project headers
target_include_directories(pros_indexer PRIVATE
        "${CMAKE_SOURCE_DIR}/include"
)

# Toolchain standard library headers (so <cstdint>, <algorithm>, etc. parse)
# Note the quotes to handle the space in "Application Support".
target_include_directories(pros_indexer SYSTEM PRIVATE
        "${PROS_TOOLCHAIN_DIR}/arm-none-eabi/include"
        "${_GCC_VER_DIR}/include"
        "${_GCC_VER_DIR}/include-fixed"
)

# Optional: PROS SDK headers if you have them separately
if(PROS_SDK_INCLUDE)
    target_include_directories(pros_indexer SYSTEM PRIVATE "${PROS_SDK_INCLUDE}")
endif()

# ---- Nice-to-have warnings for local sanity checks (host parsing only) ------
target_compile_options(pros_indexer PRIVATE
        -Wall -Wextra -Wno-attributes -Wno-psabi
)

# ---- Tell CLion where compile_commands.json should go (helps indexing) ------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
