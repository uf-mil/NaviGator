cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

find_package(catkin REQUIRED
	nav_msgs
	genmsg 
	actionlib_msgs 
	actionlib
	sub8_msgs
  	rostest
	)

add_action_files(
	DIRECTORY 
	action
	FILES
	current_goal.action
)
catkin_python_setup()
generate_messages(DEPENDENCIES actionlib_msgs nav_msgs)

catkin_package()

catkin_package(CATKIN_DEPENDS 
	actionlib_msgs 
	nav_msgs
	)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()

