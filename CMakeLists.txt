# --------------------
# REQUIRED CMAKE SETUP
# --------------------

# set the minimum required cmake version for this project
cmake_minimum_required(VERSION 2.8.9)

add_compile_options(-std=c++11)

# project name
project (kuka_rbdl)

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


# ---------------------
# BUILD THE ACTUAL CODE
# ---------------------

# add an executable to the project
# usage: add_executable(EXECUTABLE_NAME CPP_FILE1 CPP_FILE2 CPP_FILE3 ...)
add_executable(kuka_rbdl main.cpp)
add_executable(D-H-JacobianTest D-H-JacobianTest.cpp D_H_Jacobian.cpp D_H_Parameterization.cpp Utils.cpp IK.h)

# link libraries to your executable
# usage: target_link_libraries(EXECUTABLE_NAME LIBRARY1 LIBRARY2 ...)
# note : this is completely useless here, as Eigen is header-only 
target_link_libraries(kuka_rbdl ${EIGEN3_LIBRARIES}
	${RBDL_LIBRARY}
	${RBDL_LuaModel_LIBRARY}
	${LUA_LIBRARIES}
	)
target_link_libraries(D-H-JacobianTest ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)


configure_file(kuka+cube.lua kuka+cube.lua COPYONLY)
#~ configure_file(kuka.obj kuka.obj COPYONLY)
configure_file(meshes/kuka.obj meshes/kuka.obj COPYONLY)

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall")
endif()
