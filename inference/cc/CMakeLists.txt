cmake_minimum_required( VERSION 2.8 )

project( TFCustomOp )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake/modules)
find_package(TensorFlow 1.12 EXACT REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=${TensorFlow_ABI}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=${TensorFlow_ABI}" )


find_package( OpenCV REQUIRED )
MESSAGE(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")

TensorFlow_REQUIRE_C_LIBRARY()
TensorFlow_REQUIRE_SOURCE()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories($ENV{TENSORFLOW_BUILD_DIR}/includes/)


add_executable (run_infer run_infer.cc)
#target_include_directories(run_infer PRIVATE TensorFlow_DEP)
target_link_libraries(run_infer ${OpenCV_LIBS})
target_link_libraries(run_infer TensorFlow_DEP)
