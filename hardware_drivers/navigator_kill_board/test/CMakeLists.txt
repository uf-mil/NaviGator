if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(kill_board.test)
endif()

