project(GitHubExample,)

cmake_minimum_required(VERSION 3.2)

## Lab_PC 	Lab_Linux	Laptop_PC	Laptop_Linux
set(COMPUTER_SYSTEM "Lab_Linux")

set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")


if (COMPUTER_SYSTEM STREQUAL "Lab_PC")
	include_directories(
		D:/openSim/opensim-source-install/sdk/include
		D:/openSim/opensim-source-install/sdk/include/OpenSim
		D:/openSim/opensim_dependencies_install/simbody/include
		${CMAKE_SOURCE_DIR}/inc)

	link_directories(
		D:/openSim/opensim-source-install/sdk/lib
		D:/openSim/opensim_dependencies_install/simbody/lib)
elseif (COMPUTER_SYSTEM STREQUAL "Lab_Linux")
	include_directories(
		/home/vinay/OpenSim/opensim-source-install/include/OpenSim/ 
		/home/vinay/OpenSim/opensim-source-install/include/simbody/ 
		/home/vinay/OpenSim/opensim-source-install/include/
		${CMAKE_SOURCE_DIR}/inc)

	link_directories(
		/home/vinay/OpenSim/opensim-source-install/lib64/)
elseif (COMPUTER_SYSTEM STREQUAL "Laptop_PC")
	include_directories(
		D:/OpenSimAPI/opensim-core-source-install/sdk/include
		D:/OpenSimAPI/opensim-core-source-install/sdk/include/OpenSim
		D:/OpenSimAPI/opensim-core-source-install/sdk/Simbody/include
		${CMAKE_SOURCE_DIR}/inc)
	
	link_directories(
		D:/OpenSimAPI/opensim-core-source-install/sdk/lib
		D:/OpenSimAPI/opensim-core-source-install/sdk/Simbody/lib	
	)
endif()

	
add_executable(main ${PROJECT_SOURCE_DIR}/toyLeg_example.cpp ${PROJECT_SOURCE_DIR}/PistonActuator.cpp)

target_link_libraries( main
debug osimActuators_d optimized osimActuators 
debug osimAnalyses_d optimized osimAnalyses
debug osimCommon_d optimized osimCommon 
debug osimLepton_d optimized osimLepton
debug osimSimulation_d optimized osimSimulation
debug osimTools_d optimized osimTools
debug SimTKcommon_d optimized SimTKcommon
debug SimTKmath_d optimized SimTKmath 
debug SimTKsimbody_d optimized SimTKsimbody)
