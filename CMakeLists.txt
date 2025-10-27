cmake_minimum_required(VERSION 3.13.1)
project(GRR_CAN)

if(NOT cdc-acm-console IN_LIST SNIPPET)
  set(SNIPPET cdc-acm-console ${SNIPPET} CACHE STRING "" FORCE)
endif()

find_package(Zephyr)


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
