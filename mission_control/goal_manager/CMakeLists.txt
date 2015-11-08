cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

find_package(catkin REQUIRED
	nav_msgs
	actionlib_msgs
	sub8_msgs
  	rostest
	)

catkin_python_setup()
catkin_package()

add_action_files(
	  FILES
	  current_goal.action
	)

catkin_package(CATKIN_DEPENDS 
	actionlib_msgs 
	nav_msgs
	sub8_msgs
	)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()

