cmake_minimum_required(VERSION 2.8)
project(kid_size)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

set(OpenCV_DIR "/usr/local/include/")
find_package(OpenCV 3.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  hl_communication
  RhAL
  robot_model
  robocup_referee
  rhoban_graphs
  rhoban_random
  rhoban_unsorted
  tclap
  rhoban_team_play
  rhoban_model_learning
  csa_mdp_experiments
  vive_provider
  )

catkin_package(
  INCLUDE_DIRS Motion/
  #LIBRARIES #TODO: which libraries are published?
  CATKIN_DEPENDS tclap RhAL robot_model robocup_referee rhoban_graphs rhoban_team_play rhoban_random rhoban_unsorted rhoban_model_learning hl_communication vive_provider
  )

#Build option
option (KID_SIZE_USES_FLYCAPTURE "Use flycapture to build the vision" ON)
option (KID_SIZE_USES_IDS "Enable support for ids imaging cameras" OFF)
option (BUILD_VISION_OPENCL "Build vision with OpenCL support" OFF)
option (BUILD_KID_SIZE_PROGRAM_VISION "Enable the build of Vision Programs" OFF)
option (BUILD_KID_SIZE_PROGRAM_STRATEGY "Enable the build of Strategy programs" OFF)
option (BUILD_KID_SIZE_PROGRAM_WALK_ENGINE "Walk engine test program" OFF)
option (KID_SIZE_PYTHON_BINDING "Build python binding for kid size" OFF)

#Define Vision enable Macro + specific includes
enable_language(ASM)
add_definitions(-DENABLE_OPENCV)

# Use SSE2 and AVX if provided, otherwise, show a warning
if ("${CMAKE_CXX_FLAGS}" MATCHES "-msse2" AND
    "${CMAKE_CXX_FLAGS}" MATCHES "-mavx")
  # Enable SSE2 optimizations
  add_definitions(-DCNN_USE_SSE)
  # Enable Advanced Vector Extensions
  add_definitions(-DCNN_USE_AVX)
else()
  message(WARNING "missing flags for SSE2 and AVX! DNN will be really slow")
endif()

if (BUILD_VISION_OPENCL)
    add_definitions(-DENABLE_OPENCL)
endif ()

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

#Enable real warnings
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wpedantic")

# Enable open mp support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


#Enable Weird option linker
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -fPIC")

#Included internal Directories
set (INCLUDED_DIRECTORIES
  Motion
  Vision
  ${catkin_INCLUDE_DIRS})

include_directories(${INCLUDED_DIRECTORIES} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
#Linked dependency external Libraries
set (LINKED_LIBRARIES
    pthread
    ${catkin_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${OpenCV_LIBS}
)

#Add Vision libraries
if (KID_SIZE_USES_FLYCAPTURE)
  add_definitions(-DKID_SIZE_USES_FLYCAPTURE)
  set (LINKED_LIBRARIES
    ${LINKED_LIBRARIES}
    flycapture
    )
endif(KID_SIZE_USES_FLYCAPTURE)
if (KID_SIZE_USES_IDS)
  add_definitions(-DKID_SIZE_USES_IDS)
  set (LINKED_LIBRARIES
    ${LINKED_LIBRARIES}
    ueye_api
    )
endif(KID_SIZE_USES_IDS)


#List Sources sub source directories
set(SOURCES_DIRECTORIES
    Motion/moves
    Motion/scheduler
    Motion/services
    Motion/strategy
    Motion/engines
)

set (SOURCES_DIRECTORIES
  ${SOURCES_DIRECTORIES}
  Vision/Application
  Vision/Binding
  Vision/CameraState
  Vision/Filters
  Vision/Filters/Ball
  Vision/Filters/Basics
  Vision/Filters/Colors
  Vision/Filters/Custom
  Vision/Filters/Features
  Vision/Filters/Patches
  Vision/Filters/Source
  Vision/FrameSource
  Vision/Hough
  Vision/Localisation
  Vision/Localisation/Ball
  Vision/Localisation/Robot
  Vision/Localisation/Field
  Vision/Tags
  Vision/Utils
  )

#TODO: use a scheme similar to the one used for sources if 
set (PROTOBUF_MESSAGES
  Vision/Protobuf/camera_state.proto
  Vision/Protobuf/video.proto
  )

include_directories(${PROTOBUF_INCLUDE_DIR})
protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})

set (ALL_SOURCES ${PROTO_SOURCES})

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
    message(STATUS "-- Sources: Adding ${DIRECTORY}/")
    include (${DIRECTORY}/Sources.cmake)
    set (PREFIXED_SOURCES)
    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)
    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

#Build the kid_size main library
add_library (kid_size SHARED ${ALL_SOURCES})
target_link_libraries (kid_size ${LINKED_LIBRARIES})

# Adding build infos in the main
exec_program(
    "git"
    ${CMAKE_CURRENT_SOURCE_DIR}
    ARGS "rev-parse HEAD"
    OUTPUT_VARIABLE VERSION_NUM )
exec_program(
    "whoami"
    ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE VERSION_USER )
exec_program(
    "hostname"
    ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE VERSION_HOSTNAME )

set_source_files_properties (
    KidSize/buildinfos.c
    PROPERTIES COMPILE_FLAGS
     "-DVERSION_USER=\"${VERSION_USER}\" -DVERSION_NUM=\"${VERSION_NUM}\" -DVERSION_HOSTNAME=\"${VERSION_HOSTNAME}\"")

#Force main recompilation to have up to date build info
add_custom_target(date ALL COMMAND touch ${CMAKE_CURRENT_SOURCE_DIR}/KidSize/buildinfos.c)

#Build the main kid_size binary
add_executable (KidSize KidSize/main.cpp KidSize/buildinfos.c)
target_link_libraries (KidSize ${LINKED_LIBRARIES} kid_size)
add_dependencies(KidSize date)

if (BUILD_KID_SIZE_PROGRAM_STRATEGY)
    add_executable(KickStrategy Motion/strategy/main.cpp)
    target_link_libraries(KickStrategy ${LINKED_LIBRARIES} kid_size)
endif ()

if (BUILD_KID_SIZE_PROGRAM_WALK_ENGINE)
    add_executable(WalkEngine Motion/engines/test_walk_engine.cpp)
    target_link_libraries(WalkEngine ${LINKED_LIBRARIES} kid_size)
endif ()

#Build Vision Programs
if (BUILD_KID_SIZE_PROGRAM_VISION)
    # Simple player
    add_executable(Player Vision/Examples/Player.cpp)
    target_link_libraries(Player ${LINKED_LIBRARIES} kid_size)
endif ()

enable_testing()

set(TESTS
  services/vive_service
  )

if (CATKIN_ENABLE_TESTING)
  # create an executable for each test
  foreach(test ${TESTS})
    string(REPLACE "/" "_" id ${test})
    catkin_add_gtest(${id} tests/${test}.cpp)
    target_link_libraries(${id} ${LINKED_LIBRARIES} kid_size)
  endforeach(test)
endif()

if (KID_SIZE_PYTHON_BINDING)
  set(PYBIND11_PYTHON_VERSION 3.6)
  add_subdirectory(pybind11)

  pybind11_add_module(rhoban KidSize/python.cpp)
  target_link_libraries(rhoban kid_size ${PYTHON_LIBRARY} pybind11::module)
endif()