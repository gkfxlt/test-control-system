cmake_minimum_required(VERSION 3.12)

project(kaanh)

if(WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif(WIN32)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


message(${aris_DIR})

if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -DNON_MATLAB_PARSING)
endif(WIN32)
if(UNIX)
	add_definitions(-DUNIX -DNON_MATLAB_PARSING)
endif(UNIX)

#add_library(kaanh_lib STATIC src/kaanh.cpp   src/kaanh.hpp src/basicsystem.hpp src/basicsystem.cpp)


# user program started...
set(SOURCE_FILES src/main.cpp  src/basicsystem.hpp src/basicsystem.cpp)
add_executable(kaanh ${SOURCE_FILES})

#target_link_libraries(kaanh kaanh_lib)


if(UNIX)
	target_link_libraries(kaanh ${aris_LIBRARIES} atomic rt)
endif(UNIX)
IF(WIN32)
	target_link_libraries(kaanh ${aris_LIBRARIES})
endif(WIN32)
