project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

#add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(ExtendedKF ${sources})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  # At least if using vcpkg
  target_link_libraries(ExtendedKF zlib ssleay32 libuv uWS)

else(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  target_link_libraries(ExtendedKF z ssl uv uWS)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
