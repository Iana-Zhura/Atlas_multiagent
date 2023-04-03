cmake_minimum_required(VERSION 3.0)
project(pytorch_motion_planner_benchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ompl 1.5 REQUIRED)
include_directories(${OMPL_INCLUDE_DIR})

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(Boost 1.58 REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS bench_mr_binding/*.h)
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS bench_mr_binding/*.cpp)

set(BENCH_MR_DIR third_party/bench-mr)
include_directories(${BENCH_MR_DIR}/src)
include_directories(${BENCH_MR_DIR}/third_party/chomp)
include_directories(${BENCH_MR_DIR}/third_party/params/include)
link_directories(${BENCH_MR_DIR}/build/)
link_directories(${BENCH_MR_DIR}/build/lib)
link_directories(${BENCH_MR_DIR}/build/third_party/steering_functions)

set(EXTRA_LIB ${EXTRA_LIB} ${SBPL_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} chomp mzcommon steering_functions)
set(EXTRA_LIB ${EXTRA_LIB} mpb_core mpb_steering mpb_planners mpb_smoothers)

set(PYTHON_MODULE_NAME pybench_mr)

include_directories(
        third_party/pybind11/include/pybind11
        /usr/include/python3.9
)

add_library(${PYTHON_MODULE_NAME} SHARED ${SOURCES})
set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES PREFIX "" SUFFIX ".so")
target_link_libraries(${PYTHON_MODULE_NAME} ${EXTRA_LIB} ${SBPL_LIBRARIES} ${Boost_LIBRARIES}
        ${OMPL_LIBRARIES} chomp.a mzcommon.a steering_functions.a
        mpb_core.a mpb_steering.a mpb_planners.a mpb_smoothers.a)

