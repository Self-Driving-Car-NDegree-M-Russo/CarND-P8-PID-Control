project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)

# REMEMBER to set BOOST_ROOT env variable to the root folder where Boost was installed
set(BOOST_LIBRARYDIR $(BOOST_ROOT)/lib)
set(BOOST_INCLUDEDIRS $(BOOST_ROOT)/include/boost)

find_package(Boost COMPONENTS unit_test_framework
        log_setup log filesystem thread system REQUIRED)

include_directories ($(BOOST_ROOT))
include_directories ($(BOOST_LIBRARYDIR))
include_directories ($(BOOST_INCLUDEDIRS))

include_directories(/usr/local/include)

link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_subdirectory(test)

add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS ${Boost_LIBRARIES})
