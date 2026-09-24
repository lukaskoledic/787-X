cmake_minimum_required(VERSION 3.20)
project(B789PFD LANGUAGES C CXX)

set(XPLANE_SDK "" CACHE PATH "Path to the extracted X-Plane Plugin SDK")

find_path(XPLM_INCLUDE_DIR
    NAMES XPLMPlugin.h
    HINTS "${XPLANE_SDK}/CHeaders/XPLM"
    REQUIRED
)

if(APPLE)
    find_library(XPLM_LIBRARY
        NAMES XPLM XPLM_64
        HINTS "${XPLANE_SDK}/Libraries/Mac"
        PATH_SUFFIXES XPLM.framework
        REQUIRED
    )
    set(PLUGIN_FILENAME mac)
elseif(WIN32)
    find_library(XPLM_LIBRARY
        NAMES XPLM_64 XPLM
        HINTS "${XPLANE_SDK}/Libraries/Win"
        REQUIRED
    )
    set(PLUGIN_FILENAME win)
else()
    find_library(XPLM_LIBRARY
        NAMES XPLM_64 XPLM
        HINTS "${XPLANE_SDK}/Libraries/Lin"
        REQUIRED
    )
    set(PLUGIN_FILENAME lin)
endif()

add_library(787PFD MODULE
    src/787_pfd.cpp
    src/nanovg_gl_impl.c
    third_party/nanovg/src/nanovg.c
)

target_compile_features(787PFD PRIVATE cxx_std_17)
target_include_directories(787PFD PRIVATE
    "${XPLM_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanovg/src"
)
set_property(TARGET 787PFD PROPERTY C_STANDARD 99)
set_property(TARGET 787PFD PROPERTY C_STANDARD_REQUIRED ON)
target_compile_definitions(787PFD PRIVATE
    XPLM200=1
    XPLM210=1
    XPLM300=1
    XPLM301=1
)

if(APPLE)
    target_compile_definitions(787PFD PRIVATE APL=1 GL_SILENCE_DEPRECATION=1)
    find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
    target_link_libraries(787PFD PRIVATE "${XPLM_LIBRARY}" "${OPENGL_FRAMEWORK}")
elseif(WIN32)
    target_compile_definitions(787PFD PRIVATE IBM=1)
    target_link_libraries(787PFD PRIVATE "${XPLM_LIBRARY}" opengl32)
else()
    target_compile_definitions(787PFD PRIVATE LIN=1)
    target_link_libraries(787PFD PRIVATE "${XPLM_LIBRARY}" GL)
endif()

set_target_properties(787PFD PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${PLUGIN_FILENAME}"
    SUFFIX ".xpl"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/64"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/64"
)
