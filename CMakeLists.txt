cmake_minimum_required(VERSION 3.10)
project(SimpleSensoray_Project)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -D_LINUX -no-pie -lm -ldl -pthread -lpthread")
find_package (PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package (OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(X11 REQUIRED)
find_package(glfw3 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories (${X11_INCLUDE_DIR})
include_directories (${OPENGL_INCLUDE_DIR})
include_directories (${GLUT_INCLUDE_DIR})
include_directories (${GLFW_INCLUDE_DIRS})


INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/src")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/extras/GLFW")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/DHD/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/extras/GLFW/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/Eigen")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/glew/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/giflib/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/lib3ds/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/libjpeg/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/libpng/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/openal/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/openal/OpenAL32/Include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/openal/Alc")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/pugixml/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/include/theoraplayer")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/external/ogg/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/external/theora/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/external/vorbis/include")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/external/vorbis/lib")
INCLUDE_DIRECTORIES("/home/naghmehz/Documents/chai3d-3.2.0/external/theoraplayer/src/Theora")

set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl -lGLU")
set (CHAI3D_INCLUDE_DIRS "/home/naghmehz/Documents/chai3d-3.2.0//src" "/home/naghmehz/Documents/chai3d-3.2.0/external/Eigen" "/home/naghmehz/Documents/chai3d-3.2.0/external/glew/include" "${OPENGL_INCLUDE_DIR}")
set (CHAI3D_LIBRARIES chai3d ${OPENGL_LIBRARIES})
set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)


add_executable(chai_demo main.cpp)

add_library(
	simple826
	simple826.cpp
	simple826.hpp
	)
add_library(
	forcesensor
	forcesensor.cpp
	forcesensor.hpp
	)

INCLUDE_DIRECTORIES("/home/naghmehz/Documents/sensoray-sdk/driver")
link_directories("/home/naghmehz/Documents/sensoray-sdk/driver")
target_link_libraries(simple826 PRIVATE  "/home/naghmehz/Documents/sensoray-sdk/demo/lib826_64.a" m)
target_link_libraries(forcesensor PRIVATE  simple826 "/home/naghmehz/Documents/sensoray-sdk/demo/lib826_64.a" m)

target_link_libraries(chai_demo PRIVATE  forcesensor simple826  "/home/naghmehz/Documents/sensoray-sdk/demo/lib826_64.a" m)

target_link_libraries(chai_demo  PUBLIC  "/home/naghmehz/Documents/chai3d-3.2.0/lib/release/lin-x86_64-cc/libchai3d.a" "/home/naghmehz/Documents/chai3d-3.2.0/external/DHD/lib/lin-x86_64/libdrd.a" usb-1.0 glfw OpenGL::GL  ${CMAKE_DL_LIBS} Threads::Threads)
