###############################################################################
#
#	Prerequisites
#
###############################################################################

# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Spam main workspace
PROJECT(Spam)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Project parent directory
GET_FILENAME_COMPONENT(PROJECT_PARENT .. ABSOLUTE CACHE INTERNAL "Path prefix for the project parent")

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Installation prefix")

# Traget names
SET(CMAKE_RELEASE_POSTFIX "" CACHE PATH "Release postfix")
mark_as_advanced(CMAKE_RELEASE_POSTFIX)
SET(CMAKE_DEBUG_POSTFIX "DEBUG" CACHE PATH "Debug postfix")
mark_as_advanced(CMAKE_DEBUG_POSTFIX)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_PARENT}/lib CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

option(POST_BUILD_COPY_FILES "Copy resource files after build" YES)
mark_as_advanced(POST_BUILD_COPY_FILES)

MACRO(COPY_FILES TAR DST)
	if(POST_BUILD_COPY_FILES)
		foreach(SRC ${ARGN})
			add_custom_command(TARGET ${TAR} POST_BUILD COMMAND ${CMAKE_COMMAND} -DSRC="${SRC}" -DDST="${DST}" -P "${CMAKE_SOURCE_DIR}/copy.cmake")
		endforeach(SRC)
	endif(POST_BUILD_COPY_FILES)
ENDMACRO(COPY_FILES TAR DST)

# Boost
FIND_PACKAGE(Boost COMPONENTS system filesystem thread)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# PCL
FIND_PACKAGE(PCL REQUIRED)

ADD_DEFINITIONS(
	${OPENCV_DEFINITIONS}
	${PCL_DEFINITIONS}
)
INCLUDE_DIRECTORIES(
	${PROJECT_ROOT}/include
	${Boost_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
	${OPENCV_LIBRARIES}
	${PCL_LIBRARY_DIRS}
)

if (WIN32)
	SET(PROGRAM_FILES $ENV{ProgramW6432})
	SET(X86_WIN "win64")
	SET(X86_X64 "64")
	
	if (MSVC90)
		SET(MSVC_VER "90")
	elseif(MSVC10)
		SET(MSVC_VER "100")
	elseif(MSVC11)
		SET(MSVC_VER "110")
	endif()
	
	# Build options
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_DEPRECATE")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
	SET(CMAKE_DLL_EXPORT_FLAGS " ")
	
	# Library settings
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" NO)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)
	
	# Expat
	SET(EXPAT_INCLUDE "${PROGRAM_FILES}/expat/include" CACHE PATH "Path prefix for Expat include")
	#MARK_AS_ADVANCED(EXPAT_INCLUDE)
	SET(EXPAT_LIBRARY "${PROGRAM_FILES}/expat/lib" CACHE PATH "Path prefix for Expat library")
	#MARK_AS_ADVANCED(EXPAT_LIBRARY)
	# Freeglut
	SET(FREEGLUT_INCLUDE "${PROGRAM_FILES}/freeglut/include" CACHE PATH "Path prefix for Freeglut include")
	#MARK_AS_ADVANCED(FREEGLUT_INCLUDE)
	SET(FREEGLUT_LIBRARY "${PROGRAM_FILES}/freeglut/lib" CACHE PATH "Path prefix for Freeglut library")
	#MARK_AS_ADVANCED(FREEGLUT_LIBRARY)
	# PhysX
	SET(PHYSX_INCLUDE "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs" CACHE PATH "Path prefix for PhysX include")
	#MARK_AS_ADVANCED(PHYSX_INCLUDE)
	SET(PHYSX_LIBRARY "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs/lib/${X86_WIN}" CACHE PATH "Path prefix for PhysX library")
	#MARK_AS_ADVANCED(PHYSX_LIBRARY)
	# Golem
	SET(GOLEM_INCLUDE "${PROGRAM_FILES}/Golem/include" CACHE PATH "Path prefix for Golem include")
	#MARK_AS_ADVANCED(GOLEM_INCLUDE)
	SET(GOLEM_LIBRARY "${PROGRAM_FILES}/Golem/lib" CACHE PATH "Path prefix for Golem library")
	#MARK_AS_ADVANCED(GOLEM_LIBRARY)
	SET(GOLEM_BINARIES "${PROGRAM_FILES}/Golem/bin" CACHE PATH "Path prefix for Golem binaries")
	#MARK_AS_ADVANCED(GOLEM_BINARIES)
	# Grasp
	SET(GRASP_INCLUDE "${PROGRAM_FILES}/Grasp/include" CACHE PATH "Path prefix for Grasp include")
	#MARK_AS_ADVANCED(GRASP_INCLUDE)
	SET(GRASP_LIBRARY "${PROGRAM_FILES}/Grasp/lib" CACHE PATH "Path prefix for Grasp library")
	#MARK_AS_ADVANCED(GRASP_LIBRARY)
	SET(GRASP_BINARIES "${PROGRAM_FILES}/Grasp/bin" CACHE PATH "Path prefix for Grasp binaries")
	#MARK_AS_ADVANCED(GRASP_BINARIES)
	# National Instruments DAQmx
	OPTION(BUILD_SPAM_FTSENSOR_DAQ "Build NI DAQ F/T sensor device" YES)
	MARK_AS_ADVANCED(BUILD_SPAM_FTSENSOR_DAQ)
	if(BUILD_SPAM_FTSENSOR_DAQ)
	SET(NIDAQMX_INCLUDE "${PROGRAM_FILES}/National Instruments/Shared/ExternalCompilerSupport/C/include" CACHE PATH "Path prefix for National Instruments DAQmx include")
		MARK_AS_ADVANCED(NIDAQMX_INCLUDE)
		SET(NIDAQMX_LIBRARY "${PROGRAM_FILES}/National Instruments/Shared/ExternalCompilerSupport/C/lib${X86_X64}/msvc" CACHE PATH "Path prefix for National Instruments DAQmx library")
		MARK_AS_ADVANCED(NIDAQMX_LIBRARY)
	endif(BUILD_SPAM_FTSENSOR_DAQ)
	# PointGray
	OPTION(BUILD_SPAM_CAMERA_PGR "Build Point Gray camera device" YES)
	MARK_AS_ADVANCED(BUILD_SPAM_CAMERA_PGR)
	if(BUILD_SPAM_CAMERA_PGR)
		SET(POINTGRAY_INCLUDE "${PROGRAM_FILES}/Point Grey Research/FlyCapture2/include" CACHE PATH "Path prefix for PointGray include")
		MARK_AS_ADVANCED(POINTGRAY_INCLUDE)
		SET(POINTGRAY_LIBRARY "${PROGRAM_FILES}/Point Grey Research/FlyCapture2/lib${X86_X64}" CACHE PATH "Path prefix for PointGray library")
		MARK_AS_ADVANCED(POINTGRAY_LIBRARY)
	endif(BUILD_SPAM_CAMERA_PGR)
	# OpenNI
	option(BUILD_SPAM_CAMERA_DEPTH_OPENNI "Build OpenNI camera device" YES)
	if(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
		SET(OPENNI2_INCLUDE "${PROGRAM_FILES}/OpenNI2/Include" CACHE PATH "Path prefix for OpenNI include")
		MARK_AS_ADVANCED(OPENNI2_INCLUDE)
		SET(OPENNI2_LIBRARY "${PROGRAM_FILES}/OpenNI2/Lib" CACHE PATH "Path prefix for OpenNI library")
		MARK_AS_ADVANCED(OPENNI2_LIBRARY)
	endif(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
elseif (UNIX)
	# Build options
	if(X86_64)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX64 -D__x86_64__ -DNX64 -Wno-deprecated-declarations -std=c++0x")
	else(X86_64)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX32 -m32 -Di386 -DNX32 -Wno-deprecated-declarations -std=c++0x")
	endif(X86_64)
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb")
	SET(CMAKE_DLL_EXPORT_FLAGS " ")

	# Library settings
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" YES)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)
	
	# Expat
	SET(EXPAT_INCLUDE /usr/include CACHE PATH "Path prefix for Expat include")
	#MARK_AS_ADVANCED(EXPAT_INCLUDE)
	SET(EXPAT_LIBRARY /usr/lib CACHE PATH "Path prefix for Expat library")
	#MARK_AS_ADVANCED(EXPAT_LIBRARY)
	# Freeglut
	SET(FREEGLUT_INCLUDE /usr/include CACHE PATH "Path prefix for Freeglut include")
	#MARK_AS_ADVANCED(FREEGLUT_INCLUDE)
	SET(FREEGLUT_LIBRARY /usr/lib CACHE PATH "Path prefix for Freeglut library")
	#MARK_AS_ADVANCED(FREEGLUT_LIBRARY)
	# PhysX
	SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3/SDKs CACHE PATH "Path prefix for PhysX include")
	#MARK_AS_ADVANCED(PHYSX_INCLUDE)
	SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")
	#MARK_AS_ADVANCED(PHYSX_LIBRARY)
	# Golem
	SET(GOLEM_INCLUDE /usr/local/include CACHE PATH "Path prefix for Golem include")
	#MARK_AS_ADVANCED(GOLEM_INCLUDE)
	SET(GOLEM_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Golem library")
	#MARK_AS_ADVANCED(GOLEM_LIBRARY)
	SET(GOLEM_BINARIES /usr/local/bin CACHE PATH "Path prefix for Golem binaries")
	#MARK_AS_ADVANCED(GOLEM_BINARIES)
	# Grasp
	SET(GRASP_INCLUDE /usr/local/include CACHE PATH "Path prefix for Grasp include")
	#MARK_AS_ADVANCED(GRASP_INCLUDE)
	SET(GRASP_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Grasp library")
	#MARK_AS_ADVANCED(GRASP_LIBRARY)
	SET(GRASP_BINARIES /usr/local/bin CACHE PATH "Path prefix for Grasp binaries")
	#MARK_AS_ADVANCED(GRASP_BINARIES)
	# National Instruments DAQmx
	OPTION(BUILD_SPAM_FTSENSOR_DAQ "Build NI DAQ F/T sensor device" NO)
	MARK_AS_ADVANCED(BUILD_SPAM_FTSENSOR_DAQ)
	if(BUILD_SPAM_FTSENSOR_DAQ)
		SET(NIDAQMX_INCLUDE "/usr/local/natinst/nidaqmxbase/include" CACHE PATH "Path prefix for National Instruments DAQmx include")
		MARK_AS_ADVANCED(NIDAQMX_INCLUDE)
		SET(NIDAQMX_LIBRARY "/usr/local/natinst/nidaqmxbase/lib" CACHE PATH "Path prefix for National Instruments DAQmx library")
		MARK_AS_ADVANCED(NIDAQMX_LIBRARY)
	endif(BUILD_SPAM_FTSENSOR_DAQ)
	# PointGray
	OPTION(BUILD_SPAM_CAMERA_PGR "Build Point Gray camera device" NO)
	MARK_AS_ADVANCED(BUILD_SPAM_CAMERA_PGR)
	if(BUILD_SPAM_CAMERA_PGR)
		SET(POINTGRAY_INCLUDE /usr/include/flycapture CACHE PATH "Path prefix for PointGray include")
		MARK_AS_ADVANCED(POINTGRAY_INCLUDE)
		SET(POINTGRAY_LIBRARY /usr/lib CACHE PATH "Path prefix for PointGray library")
		MARK_AS_ADVANCED(POINTGRAY_LIBRARY)
	endif(BUILD_SPAM_CAMERA_PGR)
	# OpenNI
	option(BUILD_SPAM_CAMERA_DEPTH_OPENNI "Build OpenNI camera device" NO)
	if(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
		SET(OPENNI2_INCLUDE /usr/include/openni2 CACHE PATH "Path prefix for OpenNI include")
		MARK_AS_ADVANCED(OPENNI2_INCLUDE)
		SET(OPENNI2_LIBRARY /usr/lib CACHE PATH "Path prefix for OpenNI library")
		MARK_AS_ADVANCED(OPENNI2_LIBRARY)
	endif(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
endif()

LINK_DIRECTORIES(
	${EXPAT_LIBRARY}
	${FREEGLUT_LIBRARY}
	${PHYSX_LIBRARY}
	${GOLEM_LIBRARY}
	${GOLEM_BINARIES}
	${GRASP_LIBRARY}
	${GRASP_BINARIES}
	${OPENNI2_LIBRARY}
	${NIDAQMX_LIBRARY}
	${POINTGRAY_LIBRARY}
)
INCLUDE_DIRECTORIES(
	${EXPAT_INCLUDE}
	${FREEGLUT_INCLUDE}
	${PHYSX_INCLUDE}/Physics/include
	${PHYSX_INCLUDE}/Foundation/include
	${PHYSX_INCLUDE}/PhysXLoader/include
	${PHYSX_INCLUDE}/Cooking/include
	${GOLEM_INCLUDE}
	${GRASP_INCLUDE}
	${OPENNI2_INCLUDE}
	${NIDAQMX_INCLUDE}
	${POINTGRAY_INCLUDE}
)

# Spam library
SET(SPAM_HEADERS
	${PROJECT_ROOT}/include/Spam/Spam/Belief.h
	${PROJECT_ROOT}/include/Spam/Spam/Data.h
	${PROJECT_ROOT}/include/Spam/Spam/GraphPlanner.h
	${PROJECT_ROOT}/include/Spam/Spam/Heuristic.h
	${PROJECT_ROOT}/include/Spam/Spam/Robot.h
)
SET(SPAM_SOURCES
	${PROJECT_ROOT}/src/Spam/Spam/Belief.cpp
	${PROJECT_ROOT}/src/Spam/Spam/Data.cpp
	${PROJECT_ROOT}/src/Spam/Spam/GraphPlanner.cpp
	${PROJECT_ROOT}/src/Spam/Spam/Heuristic.cpp
	${PROJECT_ROOT}/src/Spam/Spam/Robot.cpp
)
if(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(SpamSpam SHARED ${SPAM_HEADERS} ${SPAM_SOURCES})
	#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
else(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(SpamSpam STATIC ${SPAM_HEADERS} ${SPAM_SOURCES})
endif(BUILD_DYNAMIC_LIBS)
if (WIN32)
	TARGET_LINK_LIBRARIES(
		SpamSpam
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64
		expat freeglut Gdiplus
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceRobotJustin${CMAKE_RELEASE_POSTFIX} debug GolemDeviceRobotJustin${CMAKE_DEBUG_POSTFIX} 
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
	)
	if(BUILD_SPAM_FTSENSOR_DAQ)
		TARGET_LINK_LIBRARIES(
			SpamSpam
			optimized daqft${CMAKE_RELEASE_POSTFIX} debug daqft${CMAKE_DEBUG_POSTFIX}		 
			NIDAQmx
		)
	endif(BUILD_SPAM_FTSENSOR_DAQ)
	if(BUILD_SPAM_CAMERA_PGR)
		TARGET_LINK_LIBRARIES(SpamSpam FlyCapture2_v${MSVC_VER})
	endif(BUILD_SPAM_CAMERA_PGR)
	if(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
		TARGET_LINK_LIBRARIES(SpamSpam OpenNI2)
	endif(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
elseif (UNIX)
	TARGET_LINK_LIBRARIES(
		SpamSpam
		${PCL_LIBRARIES}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${OpenCV_LIBS}
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		${Boost_LIBRARIES}
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceRobotJustin${CMAKE_RELEASE_POSTFIX} debug GolemDeviceRobotJustin${CMAKE_DEBUG_POSTFIX}
		${CMAKE_DL_LIBS}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		GL
		GLU
		glut
		NxCharacter NxCooking PhysXCore PhysXLoader
	)
	if(BUILD_SPAM_FTSENSOR_DAQ)
		TARGET_LINK_LIBRARIES(
			SpamSpam
			optimized daqft${CMAKE_RELEASE_POSTFIX} debug daqft${CMAKE_DEBUG_POSTFIX}		 
			nidaqmxbase
		)
	endif(BUILD_SPAM_FTSENSOR_DAQ)
	if(BUILD_SPAM_CAMERA_PGR)
		TARGET_LINK_LIBRARIES(SpamSpam flycapture)
	endif(BUILD_SPAM_CAMERA_PGR)
	if(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
		TARGET_LINK_LIBRARIES(SpamSpam OpenNI2)
	endif(BUILD_SPAM_CAMERA_DEPTH_OPENNI)
endif()	
SET_PROPERTY(TARGET SpamSpam PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET SpamSpam PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET SpamSpam PROPERTY PROJECT_LABEL "Spam")
SET_PROPERTY(TARGET SpamSpam PROPERTY FOLDER "Spam")
INSTALL(TARGETS SpamSpam RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${SPAM_HEADERS} DESTINATION include/Spam/Spam/)

# RagPlanner
SET(RAG_PLANNER_SOURCES
	${PROJECT_ROOT}/src/Spam/RagPlanner/RagPlanner.cpp
)
SET(RAG_PLANNER_HEADERS
	${PROJECT_ROOT}/include/Spam/RagPlanner/RagPlanner.h
)
SET(RAG_PLANNER_FILES
	${PROJECT_ROOT}/resources/SpamDeviceRobotBhamSim.xml
	${PROJECT_ROOT}/resources/SpamDeviceRobotJustin.xml
	${PROJECT_ROOT}/resources/SpamRagPlanner_RobotBham.xml
	${PROJECT_ROOT}/resources/SpamRagPlanner_RobotJustin.xml
)

ADD_EXECUTABLE(SpamRagPlanner ${RAG_PLANNER_SOURCES} ${RAG_PLANNER_HEADERS} ${RAG_PLANNER_FILES})
SET_TARGET_PROPERTIES(SpamRagPlanner PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -D_SPAM_RAG_MAIN_")
TARGET_LINK_LIBRARIES(SpamRagPlanner SpamSpam)
COPY_FILES(SpamRagPlanner ${RUNTIME_OUTPUT_DIRECTORY} ${RAG_PLANNER_FILES})
SET_PROPERTY(TARGET SpamRagPlanner PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET SpamRagPlanner PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
INSTALL(TARGETS SpamRagPlanner RUNTIME DESTINATION bin)
INSTALL(FILES ${RAG_PLANNER_HEADERS} DESTINATION include/Spam/RagPlanner/)
INSTALL(FILES ${RAG_PLANNER_FILES} DESTINATION bin)

SET_PROPERTY(TARGET SpamRagPlanner PROPERTY PROJECT_LABEL "RagPlanner")
SET_PROPERTY(TARGET SpamRagPlanner PROPERTY FOLDER "Programs")
SOURCE_GROUP("Include Files" FILES ${RAG_PLANNER_HEADERS})
SOURCE_GROUP("Resource Files" FILES ${RAG_PLANNER_FILES})
