cmake_minimum_required(VERSION 3.19)

# Toolchain MUST be set before project()
set(CMAKE_TOOLCHAIN_FILE
    "${CMAKE_CURRENT_LIST_DIR}/lib/libDaisy/cmake/toolchains/ArmGNUToolchain.cmake"
    CACHE FILEPATH "ARM toolchain file")

project(DrumPad LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD_REQUIRED   ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-ggdb3>
    $<$<CONFIG:Release>:-Os>
    $<$<CONFIG:Release>:-flto>
)
add_link_options(
    $<$<CONFIG:Release>:-flto>
)

# libDaisy (must be cloned into lib/libDaisy — see README)
add_subdirectory(lib/libDaisy)

# Linker script for Daisy Seed internal flash (0x08000000, 128KB)
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/lib/libDaisy/core/STM32H750IB_flash.lds")

# Firmware target
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE daisy)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${LINKER_SCRIPT}
)

target_link_options(${PROJECT_NAME} PUBLIC
    LINKER:-T,${LINKER_SCRIPT}
    LINKER:--gc-sections
    LINKER:--check-sections
    LINKER:--unresolved-symbols=report-all
    LINKER:--warn-common
    LINKER:--warn-section-align
    LINKER:--print-memory-usage
)

# Post-build: produce .bin for DFU upload
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
            -O binary
            "$<TARGET_FILE:${PROJECT_NAME}>"
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"
    COMMENT "Generating ${PROJECT_NAME}.bin for DFU upload"
)
