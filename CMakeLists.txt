cmake_minimum_required( VERSION 2.8 FATAL_ERROR)
project(PCL_SHOW)

add_executable( pcl_show src/main.cpp)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "pcl_show" )

# Find Packages
find_package( PCL 1.8 REQUIRED )
 
if( PCL_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
 
  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )
  #add_definitions( -DPCL_NO_PRECOMPILE )
 
  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
 
  # Additional Dependencies
  target_link_libraries( pcl_show ${PCL_LIBRARIES} )

  install (TARGETS pcl_show DESTINATION bin)
endif()