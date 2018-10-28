cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 11)
project(qpc)

find_package(PythonLibs)
IF(PythonLibs_FOUND)

 message(STATUS  "Python IS  FOUND : VERSION" ${PYTHONLIBS_VERSION_STRING})
 message(STATUS  ${PYTHON_INCLUDE_DIR})

ENDIF()
include_directories(
    ${PYTHON_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_library(qptrajectory SHARED include/qptrajectory.cpp include/qptrajectory.h)
target_link_libraries(
    qptrajectory  ${PYTHON_LIBRARIES}
  #-lgmp -lmpfr -lCGAL
)

#qt5_use_modules(qtsampleplayer Core Widgets Multimedia MultimediaWidgets Gui OpenGL)
add_executable(cvxqp main.cpp )
target_link_libraries(


cvxqp
${PYTHON_LIBRARIES}
qptrajectory

)
