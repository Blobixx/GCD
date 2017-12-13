# Specify the minimum version for CMake

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Project's name

PROJECT(GeneralizedCylinderDecomposition)

SET(TARGET_NAME gcd)


IF(NOT WIN32)

	SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11 -Wno-deprecated")
	#set(CMAKE_LIBRARY_PATH "${CMAKE_LIBRARY_PATH} /usr/local/opt/llvm/5.0.0/lib/")

ENDIF(NOT WIN32)

find_package(Boost COMPONENTS system REQUIRED)


find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDES})
if(NOT CGAL_FOUND)
    message(FATAL_ERROR "Could not find CGAL!")
endif(NOT CGAL_FOUND)

find_library(MPFR REQUIRED)
include_directories(${MPFR_INCLUDES})
if(NOT MPFR_FOUND)
    message(FATAL_ERROR "Could not find MPFR!")
endif(NOT MPFR_FOUND)

find_library(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIR})
if(NOT (GMP_INCLUDE_DIR AND GMP_LIBRARIES))
    message(FATAL_ERROR "Could not find GMP!")
endif(NOT (GMP_INCLUDE_DIR AND GMP_LIBRARIES))

IF(APPLE)
   INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
   FIND_LIBRARY(GLUT_LIBRARY GLUT )
   FIND_LIBRARY(OpenGL_LIBRARY OpenGL )
   MARK_AS_ADVANCED (GLUT_LIBRARY
                     OpenGL_LIBRARY)
   SET(EXTRA_LIBS ${GLUT_LIBRARY} ${OpenGL_LIBRARY} )
ENDIF (APPLE)

include_directories(${CMAKE_SOURCE_DIR}/include)

#file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
SET(SOURCES_FOLDER ${CMAKE_SOURCE_DIR}/src)
SET(INCLUDE_FOLDER ${CMAKE_SOURCE_DIR}/include)

SET(SOURCES
    ${SOURCES_FOLDER}/main.cpp
    ${SOURCES_FOLDER}/Camera.cpp
    ${SOURCES_FOLDER}/Trackball.cpp
    ${SOURCES_FOLDER}/Mesh.cpp
    ${SOURCES_FOLDER}/GC.cpp
    ${SOURCES_FOLDER}/Shape.cpp
)
SET(HEADER_FILES
    ${INCLUDE_FOLDER}/Vec3.h
    ${INCLUDE_FOLDER}/Camera.h
    ${INCLUDE_FOLDER}/Trackball.h
    ${INCLUDE_FOLDER}/Mesh.h
    ${INCLUDE_FOLDER}/GC.h
    ${INCLUDE_FOLDER}/HermiteCurve.h
    ${INCLUDE_FOLDER}/Shape.h
    ${INCLUDE_FOLDER}/Utils.h
)

# SET(SOURCES
# 	main.cpp
# 	GC.cpp
# 	Camera.cpp
# 	Mesh.cpp
# )

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_executable(${TARGET_NAME} ${SOURCES} ${HEADER_FILES})
SET_TARGET_PROPERTIES(${TARGET_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${TARGET_NAME} CGAL)
target_link_libraries(${TARGET_NAME} ${EXTRA_LIBS})
target_link_libraries(${TARGET_NAME} ${MPFR_LIBRARIES})
target_link_libraries(${TARGET_NAME} ${GMP_LIBRARIES})
target_link_libraries(${TARGET_NAME} ${Boost_SYSTEM_LIBRARY})