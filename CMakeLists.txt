cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)

option(social_physical_build_tests "Build all of SocialPhysical's own tests." OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR) # if building as top-level
	option(social_physical_build_samples "Build all of tableSample." ON)
else()
	option(social_physical_build_samples "Build all of tableSample." ON)
endif()

project(SocialPhysical VERSION 0.9.1 LANGUAGES CXX)

#Make sure that custom modules like FindMujoco are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# UNCOMMENT THESE TO ENABLE USING MUJOCO
# Load local dependency paths
include(cmake/LocalProperties.cmake)
find_package(Mujoco REQUIRED MODULE)
find_package(Rangev3 REQUIRED MODULE)

add_library(SocialPhysicalLib INTERFACE)
#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(SocialPhysical::SocialPhysicalLib ALIAS SocialPhysicalLib)

target_include_directories(SocialPhysicalLib INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	$<INSTALL_INTERFACE:include>
	~/.mujoco/mjpro150/include
)
target_link_libraries(SocialPhysicalLib INTERFACE
	 Rangev3::Rangev3 Mujoco::Mujoco
)

if (social_physical_build_tests)
	enable_testing()
	add_subdirectory(test)
endif()

if (social_physical_build_samples)
	add_executable(tableSample src/table.cpp)
	target_link_libraries(tableSample
		PUBLIC
			 Rangev3::Rangev3 Mujoco::Mujoco
		PRIVATE
			SocialPhysicalLib
	)
endif()
