cmake_minimum_required(VERSION 3.13.1)
project(GRR_CAN)

if(NOT cdc-acm-console IN_LIST SNIPPET)
  set(SNIPPET cdc-acm-console ${SNIPPET} CACHE STRING "" FORCE)
endif()

find_package(Zephyr)

# ── Generate board_config_generated.h from YAML ──────────────
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(BOARD_CONFIG_YAML "${CMAKE_CURRENT_LIST_DIR}/config/board_config.yaml")
set(BOARD_CONFIG_HDR  "${CMAKE_BINARY_DIR}/generated/board_config_generated.h")

add_custom_command(
    OUTPUT  ${BOARD_CONFIG_HDR}
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_LIST_DIR}/scripts/generate_board_config.py
            ${BOARD_CONFIG_YAML}
            ${BOARD_CONFIG_HDR}
    DEPENDS ${BOARD_CONFIG_YAML}
            ${CMAKE_CURRENT_LIST_DIR}/scripts/generate_board_config.py
    COMMENT "Generating board_config_generated.h from YAML"
)
add_custom_target(board_config_gen DEPENDS ${BOARD_CONFIG_HDR})

# ── Application sources ──────────────────────────────────────
file(GLOB app_sources
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cc"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cxx")

if(NOT app_sources)
  message(WARNING "No application sources found in ${CMAKE_CURRENT_LIST_DIR}/src")
else()
  target_sources(app PRIVATE ${app_sources})
endif()

# Make sure the generated header is built before compiling, and is findable
add_dependencies(app board_config_gen)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/generated)
