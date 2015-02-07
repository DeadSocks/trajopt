find_package(PCL 1.6 REQUIRED COMPONENTS common io)


include_directories(
  SYSTEM
  ${BULLET_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${CMAKE_SOURCE_DIR}/ext
  ${CMAKE_SOURCE_DIR}/src
)

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(arm_to_cart_target
	arm_to_cart_target.cpp
	atlas_utils.cpp
)
set_target_properties(arm_to_cart_target PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(arm_to_cart_target ${OpenRAVE_BOTH_LIBRARIES} sco utils json osgviewer trajopt)

add_executable(arm_to_joint_target arm_to_joint_target.cpp atlas_utils.cpp)
set_target_properties(arm_to_joint_target PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(arm_to_joint_target ${OpenRAVE_BOTH_LIBRARIES} ${Boost_LIBRARIES} sco utils json osgviewer trajopt)