# --------------------
# REQUIRED CMAKE SETUP
# --------------------

# set the minimum required cmake version for this project
cmake_minimum_required(VERSION 2.8.9)

add_compile_options(-std=c++11)

# project name
project (IKProject)

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
add_executable(D-H-Test tests/D-H-Test.cpp src/HierarchyOfDHParameterization.cpp src/D_H_Parameterization.cpp src/Utils.cpp src/KuKa_KR5_R850_D_H.cpp src/Trihedral.cpp include/IK.h)
add_executable(ExtractGlobalPositionAndOrientationOfJoints tests/ExtractGlobalPositionAndOrientationOfJoints.cpp include/IK.h)
add_executable(kuka_rbdl_IK_Test tests/kuka_rbdl_IK_Test.cpp)
add_executable(LoadAndWriteAnimation_LUA_KukaRobot_Test tests/LoadAndWriteAnimation_LUA_KukaRobot_Test.cpp include/IK.h)
add_executable(D-H-JacobianTest tests/D-H-JacobianTest.cpp src/HierarchyOfDHParameterization.cpp src/D_H_Parameterization.cpp src/Utils.cpp src/KuKa_KR5_R850_D_H.cpp include/IK.h)
add_executable(Build_KuKa_KR5_R850_D_H-Test tests/Build_KuKa_KR5_R850_D_H-Test.cpp src/KuKa_KR5_R850_D_H.cpp src/D_H_Parameterization.cpp src/HierarchyOfDHParameterization.cpp src/Utils.cpp src/Trihedral.cpp include/IK.h)
add_executable(D-H-DOF_Variation-Test tests/D-H-DOF_Variation-Test.cpp src/KuKa_KR5_R850_D_H.cpp src/D_H_Parameterization.cpp src/HierarchyOfDHParameterization.cpp src/Utils.cpp src/Trihedral.cpp include/IK.h)
add_executable(Trihedral_Orientation_In_Each_Frame_Test tests/Trihedral_Orientation_In_Each_Frame_Test.cpp src/KuKa_KR5_R850_D_H.cpp src/D_H_Parameterization.cpp src/HierarchyOfDHParameterization.cpp src/Utils.cpp src/Trihedral.cpp include/IK.h)
add_executable(IK_Test tests/IK_Test.cpp src/KuKa_KR5_R850_D_H.cpp src/D_H_Parameterization.cpp src/HierarchyOfDHParameterization.cpp src/Utils.cpp src/Trihedral.cpp include/IK.h)
# link libraries to your executable
# usage: target_link_libraries(EXECUTABLE_NAME LIBRARY1 LIBRARY2 ...)
# note : this is completely useless here, as Eigen is header-only
target_link_libraries(D-H-Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(IK_Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(Trihedral_Orientation_In_Each_Frame_Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(D-H-DOF_Variation-Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(Build_KuKa_KR5_R850_D_H-Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(ExtractGlobalPositionAndOrientationOfJoints ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(kuka_rbdl_IK_Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(LoadAndWriteAnimation_LUA_KukaRobot_Test ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)
target_link_libraries(D-H-JacobianTest ${EIGEN3_LIBRARIES}
		${RBDL_LIBRARY}
		${RBDL_LuaModel_LIBRARY}
		${LUA_LIBRARIES}
		)


configure_file(LUA/kuka+cube.lua LUA/kuka+cube.lua COPYONLY)
#~ configure_file(kuka.obj kuka.obj COPYONLY)
configure_file(meshes/kuka.obj meshes/kuka.obj COPYONLY)
configure_file(LUA/kuka.lua LUA/kuka.lua COPYONLY)

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall")
endif()
