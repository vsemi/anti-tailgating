cmake_minimum_required(VERSION 3.5.0)

PROJECT(peoplecount)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

message( STATUS "Architecture: ${ARCHITECTURE}" )


find_package(SQLite3 REQUIRED)

message("SQLite3 version: ${SQLite3_VERSION}")
message("SQLite3 INCLUDE_DIR: ${SQLite3_INCLUDE_DIR}")
message("SQLite3 LIBRARY_DIRS: ${SQLite3_LIBRARY_DIRS}")
message("SQLite3 LIBS: ${SQLite3_LIBS}")

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package (Threads)

find_package(PCL REQUIRED)

find_package(eclipse-paho-mqtt-c REQUIRED)

message("eclipse-paho-mqtt-c version: ${eclipse-paho-mqtt-c_VERSION}")

file(GLOB peoplecount_INCLUDE_DIR
    "include"
)

file(GLOB peoplecount_LIB_DIR
    "lib/${ARCHITECTURE}"
)

file(GLOB peoplecount_SRC
    "src/*.cpp"
)

include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${peoplecount_INCLUDE_DIR} )

link_directories( ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} {CMAKE_THREAD_LIBS_INIT} ${peoplecount_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE( peoplecount ${peoplecount_SRC})

TARGET_LINK_LIBRARIES(peoplecount ${OpenCV_LIBS} ${PCL_LIBRARIES} paho-mqtt3as SQLite::SQLite3 vsemi_pct)



