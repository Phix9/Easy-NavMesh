cmake_minimum_required(VERSION 3.15)

set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(Easy_NavMesh)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL2 REQUIRED)
find_package(SDL2_ttf REQUIRED)

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows")
endif()

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/vector2.h
    src/scene.h
    src/mesh.h
    src/button.h
    src/singleton.h
    src/resources_manager.h
    )

target_link_libraries(${PROJECT_NAME} 
    SDL2::SDL2 
    SDL2_ttf::SDL2_ttf
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)