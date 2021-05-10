# --------------------
# REQUIRED CMAKE SETUP
# --------------------

# set the minimum required cmake version for this project
cmake_minimum_required(VERSION 2.8.9)

# project name
project (fabrikTest)

# ------------
# DEPENDENCIES
# ------------

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

# search for the eigen3 package
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
FIND_PACKAGE (Lua51 REQUIRED)

# include the eigen header directories, such that #include <Eigen/...>
# works in your C++ files
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${RBDL_INCLUDE_DIR})

include_directories(./include)
# ---------------------
# BUILD THE ACTUAL CODE
# ---------------------

# add an executable to the project
# usage: add_executable(EXECUTABLE_NAME CPP_FILE1 CPP_FILE2 CPP_FILE3 ...)
add_executable(fabrik
	src/point.cpp
	src/fabrik.cpp
	main.cpp)

add_executable(fabrik2 tests/Fabrik_Test.cpp src/fabrik3.cpp include/fabrik3.h)
add_executable(ccd_test tests/ccd_Test.cpp src/ccd.cpp include/ccd.h )
add_executable(ccd_with_RBDL_Test tests/CCD_With_RBDL_Test.cpp src/ccd.cpp include/ccd.h )


# link libraries to your executable
# usage: target_link_libraries(EXECUTABLE_NAME LIBRARY1 LIBRARY2 ...)
# note : this is completely useless here, as Eigen is header-only
target_link_libraries(fabrik ${EIGEN3_LIBRARIES}
	${RBDL_LIBRARY}
	${RBDL_LuaModel_LIBRARY}
	${LUA_LIBRARIES}
	)

target_link_libraries(ccd_with_RBDL_Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)

target_link_libraries(ccd_test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)

target_link_libraries(fabrik2 ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)


target_compile_features(fabrik
  PRIVATE
    cxx_std_17)

#configure_file(kuka+cube.lua kuka+cube.lua COPYONLY)
#~ configure_file(kuka.obj kuka.obj COPYONLY)
#configure_file(meshes/kuka.obj meshes/kuka.obj COPYONLY)

configure_file(models/chain.lua models/chain.lua COPYONLY)
configure_file(meshes/unit_cube.obj meshes/unit_cube.obj COPYONLY)

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall")
endif()
