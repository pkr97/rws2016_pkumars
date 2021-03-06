cmake_minimum_required(VERSION 2.8.3)
project(ball_detection)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  		cv_bridge
  		image_transport
  		roscpp
  		rospy
  		sensor_msgs
  		std_msgs
  		actionlib
  		actionlib_msgs
  		message_generation
  		tf
		visualization_msgs
  		)

find_package(PCL 1.5 REQUIRED COMPONENTS segmentation filters visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
		 )

## Generate actions in the 'action' folder
add_action_files(
   FILES
		 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 		)

catkin_package(
  		INCLUDE_DIRS include
  		CATKIN_DEPENDS
 		roscpp rospy
  		std_msgs actionlib
   		actionlib_msgs 
		message_runtime
    		cv_bridge
     		image_transport
      		sensor_msgs
       		message_generation
		DEPENDS PCL	
		)

include_directories(
  			${catkin_INCLUDE_DIRS}
  			${OpenCV_INCLUDE_DIRS}
 			${Boost_INCLUDE_DIRS}
 			${PCL_INCLUDE_DIRS} 
		)

find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED COMPONENTS)

#link_directories(${ARMADILLO_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

find_package(PkgConfig REQUIRED)
#find_package(Armadillo 3.6.0 REQUIRED)


pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
link_directories(/usr/include/libfreenect)
link_directories(/usr/local/include/libfreenect)
link_directories(/usr/include/eigen3)
link_directories(/usr/local/cuda/lib64)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fivepoint
  DEPENDS opencv
)

include_directories(
  ${OpenCV_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  include
)




##This	will	create	executables	of	the	nodes
add_executable(ball src/ball.cpp)
#add_executable(ball1 src/ball1.cpp)
#add_executable(ball2 src/ball2.cpp)
#add_executable(ball3 src/ball3.cpp)
#add_executable(ball4 src/ball4.cpp)
#add_executable(ball5 src/ball5.cpp)
#add_executable(ball20 src/ball20.cpp src/utils.cpp src/CGrid_3D.cpp src/precomp.cpp)
#add_executable(ball21 src/ball21.cpp src/utils.cpp src/CGrid_3D.cpp src/CTrajectory.cpp) 
#add_executable(ball17 src/ball17.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp)
#add_executable(ball18 src/ball18.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp)
#add_executable(skelCord src/skelCord.cpp)
#add_executable(ball22 src/ball22.cpp src/utils.cpp src/CGrid_3D.cpp src/CTrajectory.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp) #  
#add_executable(ball23 src/ball23.cpp src/utils.cpp src/CGrid_3D.cpp src/CTrajectory.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp) #  
#add_executable(ball_trajectory src/ball_trajectory.cpp)
#add_executable(ball24 src/ball24.cpp src/utils.cpp src/CGrid_3D.cpp src/CTrajectory.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp) # 

#add_executable(transform src/transform.cpp)
#add_executable(transform1 src/transform1.cpp)
#add_executable(ball_R1 src/ball_R1.cpp src/utils.cpp src/CGrid_3D.cpp)
#add_executable(ball25 src/ball25.cpp src/utils.cpp src/CGrid_3D.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp) # 
#add_executable(ball26 src/ball26.cpp src/utils.cpp src/CGrid_3D.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp)
#############################################################################################################################################################
#add_executable(ball_cord src/ball_cord.cpp src/utils.cpp src/CGrid_3D.cpp)
#add_executable(ball_estimate src/ball_estimate.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp) #
#add_executable(ballCord_transform src/ballCord_transform.cpp)
#add_executable(handCord src/handCord.cpp)
#add_executable(ball_cordKalman src/ball_cordKalman.cpp src/utils.cpp src/CGrid_3D.cpp src/QuadraticRegression.cpp src/LinearRegression.cpp src/simple_kalman.cpp)
#add_executable(polyfitTest src/polyfitTest.cpp src/precomp.cpp) #  src/polyfit.cpp 

   

##This	will	generate	message	header	file	before	building	the	target
add_dependencies(ball ball_detection_generate_messages_cpp)
#add_dependencies(ball1 ball_detection_generate_messages_cpp)
#add_dependencies(ball2 ball_detection_generate_messages_cpp)
#add_dependencies(ball3 ball_detection_generate_messages_cpp)
#add_dependencies(ball4 ball_detection_generate_messages_cpp)
#add_dependencies(ball5 ball_detection_generate_messages_cpp)
#add_dependencies(ball20 ball_detection_generate_messages_cpp)
#add_dependencies(ball21 ball_detection_generate_messages_cpp)
#add_dependencies(ball17 ball_detection_generate_messages_cpp)
#add_dependencies(ball18 ball_detection_generate_messages_cpp)
#add_dependencies(skelCord ball_detection_generate_messages_cpp)
#add_dependencies(ball22 ball_detection_generate_messages_cpp)
#add_dependencies(ball23 ball_detection_generate_messages_cpp)
#add_dependencies(ball_trajectory ball_detection_generate_messages_cpp)
#add_dependencies(ball24 ball_detection_generate_messages_cpp)

#add_dependencies(transform ball_detection_generate_messages_cpp)
#add_dependencies(transform1 ball_detection_generate_messages_cpp)
#add_dependencies(ball_R1 ball_detection_generate_messages_cpp)
#add_dependencies(ball25 ball_detection_generate_messages_cpp)
#add_dependencies(ball26 ball_detection_generate_messages_cpp)
#############################################################################################################################################################
#add_dependencies(ball_cord ball_detection_generate_messages_cpp)
#add_dependencies(ball_estimate ball_detection_generate_messages_cpp)
#add_dependencies(ballCord_transform ball_detection_generate_messages_cpp)
#add_dependencies(handCord ball_detection_generate_messages_cpp)
#add_dependencies(ball_cordKalman ball_detection_generate_messages_cpp)
#add_dependencies(polyfitTest ball_detection_generate_messages_cpp)




##This	will	link	executables	to	the	appropriate	libraries
target_link_libraries(ball ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 	
#target_link_libraries(ball1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 	
#target_link_libraries(ball2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(ball3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(ball4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ball5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(ball20 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) 
#target_link_libraries(ball21 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ball17 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(ball18 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(skelCord ${catkin_LIBRARIES}) 
#target_link_libraries(ball22 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
#target_link_libraries(ball23 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
#target_link_libraries(ball_trajectory ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ball24 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})  

#target_link_libraries(transform ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(transform1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ball_R1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 
#target_link_libraries(ball25 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
#target_link_libraries(ball26 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
#############################################################################################################################################################
#target_link_libraries(ball_cord ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ball_estimate ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(ballCord_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(handCord ${catkin_LIBRARIES})
#target_link_libraries(ball_cordKalman ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})
#target_link_libraries(polyfitTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})




