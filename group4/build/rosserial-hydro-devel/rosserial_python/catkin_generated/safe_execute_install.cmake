execute_process(COMMAND "/home/pioneer/group41/build/rosserial-hydro-devel/rosserial_python/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/pioneer/group41/build/rosserial-hydro-devel/rosserial_python/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
