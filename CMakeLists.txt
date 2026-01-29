cmake_minimum_required(VERSION 3.22)
project(phyc)

set(CMAKE_CXX_STANDARD 20)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

add_library(glad external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
add_library(imgui
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw glad)

add_subdirectory(external/glm)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "disable doc")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "disable pkgconfig")
set(EIGEN_BUILD_TESTS OFF CACHE BOOL "disable tests")
add_subdirectory(external/eigen)


add_executable(phyc src/main.cpp
        src/renderer/Scene.cpp
        src/renderer/GUI.cpp
        src/renderer/light/Light.cpp
        src/physics/SurfaceShapes.cpp
        src/physics/World.cpp
        src/physics/RigidBody.cpp
        src/physics/RigidBodyCollision.cpp
        src/physics/RigidBodyAerodynamics.cpp)


target_link_libraries(phyc PRIVATE Eigen3::Eigen)

target_link_libraries(phyc PRIVATE
        glfw
        glad
        imgui
        glm::glm
)

if (UNIX AND NOT APPLE)
    target_link_libraries(phyc PRIVATE GL dl)
elseif (APPLE)
    target_link_libraries(phyc PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
elseif (WIN32)
    target_link_libraries(phyc PRIVATE opengl32)
endif ()