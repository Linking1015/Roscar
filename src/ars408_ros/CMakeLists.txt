cmake_minimum_required(VERSION 3.0.2)
project(ars408_ros)
find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	sensor_msgs
	visualization_msgs
	can_msgs
	tf2_ros
	geometry_msgs
)
catkin_package(
	CATKIN_DEPENDS rospy std_msgs sensor_msgs visualization_msgs can_msgs tf2_ros geometry_msgs
)
catkin_install_python(PROGRAMS
	src/ars408_node.py
	src/tf_publisher.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
