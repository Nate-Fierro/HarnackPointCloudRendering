cmake_minimum_required(VERSION 3.24)
project(HarnackPointCloudRendering LANGUAGES C CXX)

include(FetchContent)
find_package(OpenGL REQUIRED)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Keep dependencies local to the build tree and pin their source revisions.
set(glew-cmake_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(glew-cmake_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FREEGLUT_REPLACE_GLUT OFF CACHE BOOL "" FORCE)

FetchContent_Declare(glew
    URL https://codeload.github.com/Perlmint/glew-cmake/zip/f456deace7b408655109aaeff71421ef2d3858c6
    URL_HASH SHA256=d78469796864320ef11c83ff12d1322e82f35d0e03ce5416769083df9134cfb9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_Declare(freeglut
    URL https://codeload.github.com/freeglut/freeglut/zip/566cd459201ae29003ef5686c06506d8646e55c6
    URL_HASH SHA256=cc220d63f9bcc5cb6e103e4d5a740a51094963bee1e50a39161765a114f19bc1
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_Declare(cycodebase
    URL https://codeload.github.com/cemyuksel/cyCodeBase/zip/62da186e0b2f2d3673d1f18386c66caf5798cd9b
    URL_HASH SHA256=69aa98efdbe6b81278894dfe63a564933dd87f7890e9e2474c5320f82b611d2e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR headers-only)
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3147391d946bb4b6c68edd901f2add6ac1f31f8c/eigen-3147391d946bb4b6c68edd901f2add6ac1f31f8c.tar.gz
    URL_HASH SHA256=0c8c490764f9c2a793133491adca0cd073b73e0bde965c68cbe58d91b5ed4261
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR headers-only)
FetchContent_MakeAvailable(glew freeglut cycodebase eigen)

add_library(point_cloud INTERFACE)
target_include_directories(point_cloud SYSTEM INTERFACE
    "${cycodebase_SOURCE_DIR}" "${eigen_SOURCE_DIR}")
target_include_directories(point_cloud INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(point_cloud INTERFACE cxx_std_17)
target_compile_definitions(point_cloud INTERFACE NOMINMAX _CRT_SECURE_NO_WARNINGS)

# Triangle's C source must use the same scalar type and ABI as triangle.h.
add_library(triangle STATIC triangle.c)
target_link_libraries(point_cloud INTERFACE triangle)

add_library(harnack_viewer STATIC viewer.cpp)
target_include_directories(harnack_viewer SYSTEM PRIVATE "${freeglut_SOURCE_DIR}/include")
target_link_libraries(harnack_viewer PUBLIC point_cloud libglew_static freeglut_static OpenGL::GL)
target_compile_definitions(harnack_viewer PUBLIC GLEW_STATIC FREEGLUT_STATIC
    HARNACK_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_executable(harnack main.cpp)
target_link_libraries(harnack PRIVATE harnack_viewer)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT harnack)

add_custom_target(harnack_shaders
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:harnack>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/ScreenQuad.vert" "$<TARGET_FILE_DIR:harnack>/ScreenQuad.vert"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Harnack_PtCld.frag" "$<TARGET_FILE_DIR:harnack>/Harnack_PtCld.frag"
    VERBATIM)
add_dependencies(harnack harnack_shaders)

option(HARNACK_BUILD_DEBUG "Build screenshot tools and regression checks" OFF)
option(HARNACK_BUILD_AREA_TOOL "Build offline mesh-geodesic area precomputation" OFF)
if(HARNACK_BUILD_AREA_TOOL)
    FetchContent_Declare(geodesic
        URL https://codeload.github.com/mojocorp/geodesic/zip/7b08bfb03c444819a31f216cb1e0a3fb9528f806
        URL_HASH SHA256=588f347fb5fa80514ebbd042c84c42378c7f69abfe07e4b120c1630cc2f8ccd1
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR headers-only)
    FetchContent_MakeAvailable(geodesic)
    add_executable(precompute_areas tools/precompute_areas.cpp)
    target_include_directories(precompute_areas SYSTEM PRIVATE "${geodesic_SOURCE_DIR}/include")
    target_compile_features(precompute_areas PRIVATE cxx_std_17)
    target_compile_definitions(precompute_areas PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()
if(HARNACK_BUILD_DEBUG)
    enable_testing()
    add_executable(harnack_debug debug/main.cpp debug/screenshot.cpp)
    target_link_libraries(harnack_debug PRIVATE harnack_viewer)
    add_dependencies(harnack_debug harnack_shaders)
    add_executable(point_cloud_tests debug/point_cloud_tests.cpp)
    target_link_libraries(point_cloud_tests PRIVATE point_cloud)
    add_test(NAME point_cloud_regressions COMMAND point_cloud_tests "${CMAKE_CURRENT_SOURCE_DIR}/data")
    if(HARNACK_BUILD_AREA_TOOL)
        add_executable(geodesic_area_tests debug/geodesic_area_tests.cpp)
        target_include_directories(geodesic_area_tests SYSTEM PRIVATE "${geodesic_SOURCE_DIR}/include")
        target_include_directories(geodesic_area_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
        target_compile_features(geodesic_area_tests PRIVATE cxx_std_17)
        target_compile_definitions(geodesic_area_tests PRIVATE NOMINMAX)
        add_test(NAME geodesic_area_regressions COMMAND geodesic_area_tests)
    endif()
endif()
