###############################################################################
#
#	Prerequisites
#
###############################################################################

# cmake requirements
cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
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

# Architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(X86_64 1)
	#MESSAGE("Architecture x86 64 bit")
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(X86_32 1)
	#MESSAGE("Architecture x86 32 bit")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

###############################################################################
# Core libraries

option(BUILD_SPAM_CORE "Build Spam Core library" YES)
option(BUILD_SPAM_HBPLAN "Build Spam hypothesis-based planning library" YES)

###############################################################################
# Applications
option(BUILD_SPAM_APP "Build Spam application library" YES)
option(BUILD_SPAM_POSEPLANNER "Build Spam pose planner application" YES)
option(BUILD_SPAM_R2GPLANNER "Build Spam RAG planner application" YES)

###############################################################################
# Data Libraries
option(BUILD_SPAM_DATA_BELIEF "Build Spam data belief library" YES)
option(BUILD_SPAM_DATA_R2GTRAJECTORY "Build Spam data R2G trajectory library" YES)

###############################################################################
# Demos 
option(BUILD_SPAM_DEMO_R2G "Build Spam demo reach-to-grasp library" YES)

###############################################################################

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

###############################################################################
#
#	Bham software
#
###############################################################################

if (WIN32)
	if(X86_64)
		SET(PROGRAM_FILES $ENV{ProgramW6432})
		SET(X86_WIN "64")
		SET(X86_X64 "/x64")
	else(X86_64)
		SET(PROGRAM_FILES $ENV{ProgramFiles})
		SET(X86_WIN "32")
		SET(X86_X64 "")
	endif(X86_64)
	
	if (MSVC90)
		SET(MSVC_VER "90")
	elseif(MSVC10)
		SET(MSVC_VER "100")
	elseif(MSVC11)
		SET(MSVC_VER "110")
	elseif(MSVC12)
		SET(MSVC_VER "120")
	endif(MSVC90)
	
	# Build options
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_DEPRECATE")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_LIBRARY_DECLDIR_EXPORT /wd4251 /wd4275")
	
	# Library settings
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" NO)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)

	# Output
	SET(OUTPUT_DIRECTORY_POSTFIX "")

	# Set the install directory.
	SET(CMAKE_INSTALL_PREFIX "${PROGRAM_FILES}" CACHE PATH "Installation prefix")

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
	#SET(PHYSX_INCLUDE "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs" CACHE PATH "Path prefix for PhysX include")
	#MARK_AS_ADVANCED(PHYSX_INCLUDE)
	#SET(PHYSX_LIBRARY "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs/lib/${X86_WIN}" CACHE PATH "Path prefix for PhysX library")
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
	# Eigen
	SET(EIGEN_INCLUDE "${PROGRAM_FILES}/eigen" CACHE PATH "Path prefix for EIGEN include")
	# Boost
	ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	SET(Boost_USE_STATIC_LIBS ON)
	SET(Boost_ALL_DYN_LINK OFF)
	FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	# PCL
	FIND_PACKAGE(PCL REQUIRED)
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
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" NO)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)

	# Output
	SET(OUTPUT_DIRECTORY_POSTFIX "")
	
	# Set the install directory.
	SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Installation prefix")

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
	#SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3/SDKs CACHE PATH "Path prefix for PhysX include")
	#MARK_AS_ADVANCED(PHYSX_INCLUDE)
	#SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")
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
	# Eigen
	SET(EIGEN_INCLUDE /usr/include CACHE PATH "Path prefix for EIGEN include")
	# Boost
	#ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	#SET(Boost_USE_STATIC_LIBS OFF)
	#SET(Boost_ALL_DYN_LINK ON)
	FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	# PCL
	FIND_PACKAGE(PCL REQUIRED)
endif(WIN32)

ADD_DEFINITIONS(
	${OPENCV_DEFINITIONS}
	${PCL_DEFINITIONS}
)
LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
	${OPENCV_LIBRARIES}
	${PCL_LIBRARY_DIRS}
	${EXPAT_LIBRARY}
	${FREEGLUT_LIBRARY}
	#${PHYSX_LIBRARY}
	${GOLEM_LIBRARY}
	${GOLEM_BINARIES}
	${GRASP_LIBRARY}
	${GRASP_BINARIES}
)
INCLUDE_DIRECTORIES(
	${PROJECT_ROOT}/include
	${Boost_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EXPAT_INCLUDE}
	${FREEGLUT_INCLUDE}
	${EIGEN_INCLUDE}
	#${PHYSX_INCLUDE}/Physics/include
	#${PHYSX_INCLUDE}/Foundation/include
	#${PHYSX_INCLUDE}/PhysXLoader/include
	#${PHYSX_INCLUDE}/Cooking/include
	${GOLEM_INCLUDE}
	${GRASP_INCLUDE}
)

###############################################################################

option(POST_BUILD_COPY_FILES "Copy resource files after build" YES)
mark_as_advanced(POST_BUILD_COPY_FILES)

MACRO(COPY_FILES TAR DST)
	if(POST_BUILD_COPY_FILES)
		foreach(SRC ${ARGN})
			add_custom_command(TARGET ${TAR} POST_BUILD COMMAND ${CMAKE_COMMAND} -DSRC="${SRC}" -DDST="${DST}" -P "${CMAKE_SOURCE_DIR}/copy.cmake")
		endforeach(SRC)
	endif(POST_BUILD_COPY_FILES)
ENDMACRO(COPY_FILES TAR DST)

###############################################################################
#
# CORE library
#
###############################################################################
IF (BUILD_SPAM_CORE)
	SET(CORE_HEADERS
		${PROJECT_ROOT}/include/Spam/Core/Collision.h
		${PROJECT_ROOT}/include/Spam/Core/JContact.h
	)
	SET(CORE_SOURCES
		${PROJECT_ROOT}/src/Spam/Core/Collision.cpp
		${PROJECT_ROOT}/src/Spam/Core/JContact.cpp
	)
	if(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamCore SHARED ${CORE_HEADERS} ${CORE_SOURCES})
		#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
	else(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamCore STATIC ${CORE_HEADERS} ${CORE_SOURCES})
	endif(BUILD_DYNAMIC_LIBS)
	if (WIN32)
		TARGET_LINK_LIBRARIES(
			SpamCore
			optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
			optimized GraspContact${CMAKE_RELEASE_POSTFIX} debug GraspContact${CMAKE_DEBUG_POSTFIX}
			#optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
			optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}
			optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}		 
			#NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64
			expat freeglut Gdiplus
			optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
			optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
			optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
			optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
			optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
			optimized GolemUI${CMAKE_RELEASE_POSTFIX} debug GolemUI${CMAKE_DEBUG_POSTFIX}
			optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
            ${CMAKE_DL_LIBS}
			optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
			optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
			${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
		)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(
			SpamCore
			${PCL_LIBRARIES}
			optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
			optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
			${OpenCV_LIBS}
			optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
			optimized GraspContact${CMAKE_RELEASE_POSTFIX} debug GraspContact${CMAKE_DEBUG_POSTFIX}
			#optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
			##optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}	
			##optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            		##optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			##optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}	 
			${Boost_LIBRARIES}
			optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
			optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
			optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
			optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
			##optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
			##optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
			##optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
			##optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
			${CMAKE_DL_LIBS}
            		##optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
			optimized GolemUI${CMAKE_RELEASE_POSTFIX} debug GolemUI${CMAKE_DEBUG_POSTFIX}
			GL
			GLU
			glut
			#NxCharacter NxCooking PhysXCore PhysXLoader
		)
	endif(WIN32)	
	SET_PROPERTY(TARGET SpamCore PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamCore PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	SET_PROPERTY(TARGET SpamCore PROPERTY PROJECT_LABEL "Core")
	SET_PROPERTY(TARGET SpamCore PROPERTY FOLDER "Spam")
	if(BUILD_DYNAMIC_LIBS)
		if (WIN32)
			INSTALL(TARGETS SpamCore RUNTIME DESTINATION bin COMPONENT core_libs)
		elseif (UNIX)
			INSTALL(TARGETS SpamCore LIBRARY DESTINATION bin COMPONENT core_libs)
		endif()
	else(BUILD_DYNAMIC_LIBS)
		INSTALL(TARGETS SpamCore ARCHIVE DESTINATION lib COMPONENT core_libs)
	endif(BUILD_DYNAMIC_LIBS)
	INSTALL(FILES ${CORE_HEADERS} DESTINATION include/Spam/Core/ COMPONENT core_headers)
	SOURCE_GROUP("Include Files" FILES ${CORE_HEADERS})
ENDIF (BUILD_SPAM_CORE)

###############################################################################
#
# HBPLAN library
#
###############################################################################
IF (BUILD_SPAM_HBPLAN)
	SET(HBPLAN_HEADERS
		${PROJECT_ROOT}/include/Spam/HBPlan/Belief.h
		${PROJECT_ROOT}/include/Spam/HBPlan/GraphPlanner.h
		${PROJECT_ROOT}/include/Spam/HBPlan/Heuristic.h
		${PROJECT_ROOT}/include/Spam/HBPlan/Hypothesis.h
	)
	SET(HBPLAN_SOURCES
		${PROJECT_ROOT}/src/Spam/HBPlan/Belief.cpp
		${PROJECT_ROOT}/src/Spam/HBPlan/GraphPlanner.cpp
		${PROJECT_ROOT}/src/Spam/HBPlan/Heuristic.cpp
		${PROJECT_ROOT}/src/Spam/HBPlan/Hypothesis.cpp
	)
	if(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamHBPlan SHARED ${HBPLAN_HEADERS} ${HBPLAN_SOURCES})
		#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
	else(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamHBPlan STATIC ${HBPLAN_HEADERS} ${HBPLAN_SOURCES})
	endif(BUILD_DYNAMIC_LIBS)
	if (WIN32)
		TARGET_LINK_LIBRARIES(SpamHBPlan SpamCore)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(SpamHBPlan SpamCore
			optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
			optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
            		optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
	)
	endif()	
	SET_PROPERTY(TARGET SpamHBPlan PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamHBPlan PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	SET_PROPERTY(TARGET SpamHBPlan PROPERTY PROJECT_LABEL "HBPlan")
	SET_PROPERTY(TARGET SpamHBPlan PROPERTY FOLDER "Spam")
	if(BUILD_DYNAMIC_LIBS)
		if (WIN32)
			INSTALL(TARGETS SpamHBPlan RUNTIME DESTINATION bin COMPONENT core_libs)
		elseif (UNIX)
			INSTALL(TARGETS SpamHBPlan LIBRARY DESTINATION bin COMPONENT core_libs)
		endif()
	else(BUILD_DYNAMIC_LIBS)
		INSTALL(TARGETS SpamHBPlan ARCHIVE DESTINATION lib COMPONENT core_libs)
	endif(BUILD_DYNAMIC_LIBS)
	INSTALL(FILES ${HBPLAN_HEADERS} DESTINATION include/Spam/HBPlan/ COMPONENT core_headers)
	SOURCE_GROUP("Include Files" FILES ${HBPLAN_HEADERS})
ENDIF (BUILD_SPAM_HBPLAN)

###############################################################################
#
# APP library
#
###############################################################################
IF (BUILD_SPAM_APP)
	SET(APP_POSEPLANNER_HEADERS
		${PROJECT_ROOT}/include/Spam/App/PosePlanner/Data.h
		${PROJECT_ROOT}/include/Spam/App/PosePlanner/PosePlanner.h
	)
	SET(APP_R2GPlanner_HEADERS
		${PROJECT_ROOT}/include/Spam/App/R2GPlanner/Data.h
		${PROJECT_ROOT}/include/Spam/App/R2GPlanner/R2GPlanner.h
	)
	SET(APP_SOURCES
		${PROJECT_ROOT}/src/Spam/App/PosePlanner/PosePlanner.cpp
		${PROJECT_ROOT}/src/Spam/App/R2GPlanner/R2GPlanner.cpp
	)
	SET(APP_FILES
	)
	
	if(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamApp SHARED ${APP_POSEPLANNER_HEADERS} ${APP_R2GPlanner_HEADERS} ${APP_SOURCES} ${APP_FILES})
		#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
	else(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamApp STATIC ${APP_POSEPLANNER_HEADERS} ${APP_R2GPlanner_HEADERS} ${APP_SOURCES})
	endif(BUILD_DYNAMIC_LIBS)
	if (WIN32)
		TARGET_LINK_LIBRARIES(SpamApp SpamHBPlan)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(SpamApp SpamHBPlan
			optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}	
			optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            		optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}	 

		)
	endif()
	
	SET_PROPERTY(TARGET SpamApp PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamApp PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	SET_PROPERTY(TARGET SpamApp PROPERTY PROJECT_LABEL "App")
	SET_PROPERTY(TARGET SpamApp PROPERTY FOLDER "Spam/App")
	
	if(BUILD_DYNAMIC_LIBS)
		if (WIN32)
			INSTALL(TARGETS SpamApp RUNTIME DESTINATION bin COMPONENT app_libs)
		elseif (UNIX)
			INSTALL(TARGETS SpamApp LIBRARY DESTINATION bin COMPONENT app_libs)
		endif()
	else(BUILD_DYNAMIC_LIBS)
		INSTALL(TARGETS SpamApp ARCHIVE DESTINATION lib COMPONENT core_libs)
	endif(BUILD_DYNAMIC_LIBS)
	INSTALL(FILES ${APP_POSEPLANNER_HEADERS} DESTINATION include/Spam/App/PosePlanner/ COMPONENT apps_headers)
	INSTALL(FILES ${APP_R2GPlanner_HEADERS} DESTINATION include/Spam/App/R2GPlanner/ COMPONENT apps_headers)
	INSTALL(FILES ${APP_FILES} DESTINATION bin COMPONENT apps_configs)
	
	SOURCE_GROUP("Include Files" FILES ${APP_POSEPLANNER_HEADERS} ${APP_R2GPlanner_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${APP_FILES})
ENDIF (BUILD_SPAM_APP)

###############################################################################
#
# PosePlanner executables
#
###############################################################################
IF (BUILD_SPAM_POSEPLANNER)
	SET(POSE_PLANNER_SOURCES
		${PROJECT_ROOT}/src/Spam/App/PosePlanner/Main.cpp
	)
	SET(POSE_PLANNER_HEADERS
	)
	SET(POSE_PLANNER_FILES
	)

	ADD_EXECUTABLE(SpamPosePlanner ${POSE_PLANNER_SOURCES} ${POSE_PLANNER_HEADERS} ${POSE_PLANNER_FILES})
	SET_TARGET_PROPERTIES(SpamPosePlanner PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -D_SPAM_POSE_MAIN_")
	TARGET_LINK_LIBRARIES(SpamPosePlanner SpamApp)
	COPY_FILES(SpamPosePlanner ${RUNTIME_OUTPUT_DIRECTORY} ${POSE_PLANNER_FILES})
	SET_PROPERTY(TARGET SpamPosePlanner PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamPosePlanner PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	INSTALL(TARGETS SpamPosePlanner RUNTIME DESTINATION bin COMPONENT apps_execs)
	INSTALL(FILES ${POSE_PLANNER_HEADERS} DESTINATION include/Spam/PosePlanner/ COMPONENT apps_headers)
	INSTALL(FILES ${POSE_PLANNER_FILES} DESTINATION bin COMPONENT apps_configs)

	SET_PROPERTY(TARGET SpamPosePlanner PROPERTY PROJECT_LABEL "PosePlanner")
	SET_PROPERTY(TARGET SpamPosePlanner PROPERTY FOLDER "Spam/App")
	SOURCE_GROUP("Include Files" FILES ${POSE_PLANNER_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${POSE_PLANNER_FILES})
ENDIF (BUILD_SPAM_POSEPLANNER)

###############################################################################
#
# R2GPlanner executables
#
###############################################################################
IF (BUILD_SPAM_R2GPLANNER)
	SET(R2G_PLANNER_SOURCES
		${PROJECT_ROOT}/src/Spam/App/R2GPlanner/Main.cpp
	)
	SET(R2G_PLANNER_HEADERS
	)
	SET(R2G_PLANNER_FILES
		${PROJECT_ROOT}/resources/Spam/R2GPlanner/SpamR2GPlanner_RobotBoris.xml
	)

	ADD_EXECUTABLE(SpamR2GPlanner ${R2G_PLANNER_SOURCES} ${R2G_PLANNER_HEADERS} ${R2G_PLANNER_FILES})
	SET_TARGET_PROPERTIES(SpamR2GPlanner PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -D_SPAM_RAG_MAIN_")
	TARGET_LINK_LIBRARIES(SpamR2GPlanner SpamApp)
	COPY_FILES(SpamR2GPlanner ${RUNTIME_OUTPUT_DIRECTORY} ${R2G_PLANNER_FILES})
	SET_PROPERTY(TARGET SpamR2GPlanner PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamR2GPlanner PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	INSTALL(TARGETS SpamR2GPlanner RUNTIME DESTINATION bin COMPONENT apps_execs)
	INSTALL(FILES ${R2G_PLANNER_HEADERS} DESTINATION include/Spam/R2GPlanner/ COMPONENT apps_header)
	INSTALL(FILES ${R2G_PLANNER_FILES} DESTINATION bin COMPONENT apps_configs)

	SET_PROPERTY(TARGET SpamR2GPlanner PROPERTY PROJECT_LABEL "R2GPlanner")
	SET_PROPERTY(TARGET SpamR2GPlanner PROPERTY FOLDER "Spam/App")
	SOURCE_GROUP("Include Files" FILES ${R2G_PLANNER_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${R2G_PLANNER_FILES})
ENDIF (BUILD_SPAM_R2GPLANNER)

###############################################################################
#
# Data Belief library
#
###############################################################################
IF (BUILD_SPAM_DATA_BELIEF)
	SET(DATA_BELIEF_HEADERS
		${PROJECT_ROOT}/include/Spam/Data/Belief/Belief.h
	)
	SET(DATA_BELIEF_SOURCES
		${PROJECT_ROOT}/src/Spam/Data/Belief/Belief.cpp
	)
	SET(DATA_BELIEF_FILES
		${PROJECT_ROOT}/resources/Spam/Data/SpamDataBelief.xml
	)
	
	if(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamDataBelief SHARED ${DATA_BELIEF_HEADERS} ${DATA_BELIEF_SOURCES} ${DATA_BELIEF_FILES})
		#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
	else(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamDataBelief STATIC ${DATA_BELIEF_HEADERS} ${DATA_BELIEF_SOURCES} ${DATA_BELIEF_FILES})
	endif(BUILD_DYNAMIC_LIBS)
	if (WIN32)
		TARGET_LINK_LIBRARIES(SpamDataBelief SpamHBPlan)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(SpamDataBelief SpamHBPlan)
	endif()
	
	SET_PROPERTY(TARGET SpamDataBelief PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamDataBelief PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	SET_PROPERTY(TARGET SpamDataBelief PROPERTY PROJECT_LABEL "Belief")
	SET_PROPERTY(TARGET SpamDataBelief PROPERTY FOLDER "Spam/Data")
	
	COPY_FILES(SpamDataBelief ${RUNTIME_OUTPUT_DIRECTORY} ${DATA_BELIEF_FILES})
	if(BUILD_DYNAMIC_LIBS)
		if (WIN32)
			INSTALL(TARGETS SpamDataBelief RUNTIME DESTINATION bin COMPONENT app_libs)
		elseif (UNIX)
			INSTALL(TARGETS SpamDataBelief LIBRARY DESTINATION bin COMPONENT app_libs)
		endif()
	else(BUILD_DYNAMIC_LIBS)
		INSTALL(TARGETS SpamDataBelief ARCHIVE DESTINATION lib COMPONENT data_libs)
	endif(BUILD_DYNAMIC_LIBS)
	INSTALL(FILES ${DATA_BELIEF_HEADERS} DESTINATION include/Spam/Data/Belief/ COMPONENT data_headers)
	INSTALL(FILES ${DATA_BELIEF_FILES} DESTINATION bin COMPONENT data_configs)
	
	SOURCE_GROUP("Include Files" FILES ${DATA_BELIEF_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${DATA_BELIEF_FILES})
ENDIF (BUILD_SPAM_DATA_BELIEF)

###############################################################################
#
# Data Belief library
#
###############################################################################
IF (BUILD_SPAM_DATA_R2GTRAJECTORY)
	SET(DATA_R2GTRAJECTORY_HEADERS
		${PROJECT_ROOT}/include/Spam/Data/R2GTrajectory/R2GTrajectory.h
	)
	SET(DATA_R2GTRAJECTORY_SOURCES
		${PROJECT_ROOT}/src/Spam/Data/R2GTrajectory/R2GTrajectory.cpp
	)
	SET(DATA_R2GTRAJECTORY_FILES
		${PROJECT_ROOT}/resources/Spam/Data/SpamDataR2GTrajectory.xml
	)
	
	if(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamDataR2GTrajectory SHARED ${DATA_R2GTRAJECTORY_HEADERS} ${DATA_R2GTRAJECTORY_SOURCES} ${DATA_R2GTRAJECTORY_FILES})
		#SET_TARGET_PROPERTIES(SpamSpam PROPERTIES COMPILE_FLAGS ${CMAKE_DLL_EXPORT_FLAGS})
	else(BUILD_DYNAMIC_LIBS)
		ADD_LIBRARY(SpamDataR2GTrajectory STATIC ${DATA_R2GTRAJECTORY_HEADERS} ${DATA_R2GTRAJECTORY_SOURCES} ${DATA_R2GTRAJECTORY_FILES})
	endif(BUILD_DYNAMIC_LIBS)
	if (WIN32)
		TARGET_LINK_LIBRARIES(SpamDataR2GTrajectory SpamHBPlan)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(SpamDataR2GTrajectory SpamHBPlan)
	endif()
	
	SET_PROPERTY(TARGET SpamDataR2GTrajectory PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamDataR2GTrajectory PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	SET_PROPERTY(TARGET SpamDataR2GTrajectory PROPERTY PROJECT_LABEL "R2GTrajectory")
	SET_PROPERTY(TARGET SpamDataR2GTrajectory PROPERTY FOLDER "Spam/Data")
	
	COPY_FILES(SpamDataR2GTrajectory ${RUNTIME_OUTPUT_DIRECTORY} ${DATA_R2GTRAJECTORY_FILES})
	if(BUILD_DYNAMIC_LIBS)
		if (WIN32)
			INSTALL(TARGETS SpamDataR2GTrajectory RUNTIME DESTINATION bin COMPONENT app_libs)
		elseif (UNIX)
			INSTALL(TARGETS SpamDataR2GTrajectory LIBRARY DESTINATION bin COMPONENT app_libs)
		endif()
	else(BUILD_DYNAMIC_LIBS)
		INSTALL(TARGETS SpamDataR2GTrajectory ARCHIVE DESTINATION lib COMPONENT data_libs)
	endif(BUILD_DYNAMIC_LIBS)
	INSTALL(FILES ${DATA_R2GTRAJECTORY_HEADERS} DESTINATION include/Spam/Data/R2GTrajectory/ COMPONENT data_headers)
	INSTALL(FILES ${DATA_R2GTRAJECTORY_FILES} DESTINATION bin COMPONENT data_configs)
	
	SOURCE_GROUP("Include Files" FILES ${DATA_R2GTRAJECTORY_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${DATA_R2GTRAJECTORY_FILES})
ENDIF (BUILD_SPAM_DATA_R2GTRAJECTORY)

###############################################################################
#
# Spam demo
#
###############################################################################
IF(BUILD_SPAM_DEMO_R2G)
	SET(DEMO_R2G_SOURCES
		${PROJECT_ROOT}/src/Spam/Demo/R2GPlanner/R2GDemo.cpp
	)
	SET(DEMO_R2G_HEADERS
		${PROJECT_ROOT}/include/Spam/Demo/R2GPlanner/R2GDemo.h
	)
	SET(DEMO_R2G_FILES
		${PROJECT_ROOT}/resources/Spam/Demo/SpamDemoR2G_RobotBoris.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraDepthSimDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraKinectDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraOpenCVDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraOpenGLDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraOpenNIDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraPointGreyDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspCameraRobotDepthDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataContactModelDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataContactQueryDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataImageDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataPointsCurvDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataTrajectoryDemoRag.xml
		${PROJECT_ROOT}/resources/Spam/Demo/GraspDataVideoDemoRag.xml
	)
	
	ADD_EXECUTABLE(SpamDemoR2G ${DEMO_R2G_SOURCES} ${DEMO_R2G_HEADERS} ${DEMO_R2G_FILES})
	if (WIN32)
		SET_TARGET_PROPERTIES(SpamDemoR2G PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
	elseif (UNIX)
		SET_TARGET_PROPERTIES(SpamDemoR2G PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof")
	endif()
	TARGET_LINK_LIBRARIES(SpamDemoR2G SpamApp)
	COPY_FILES(SpamDemoR2G ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_R2G_FILES})
	SET_PROPERTY(TARGET SpamDemoR2G PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET SpamDemoR2G PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	INSTALL(TARGETS SpamDemoR2G RUNTIME DESTINATION bin COMPONENT demo_execs)
	INSTALL(FILES ${DEMO_R2G_HEADERS} DESTINATION include/Spam/Demo/Rag/ COMPONENT demo_headers)
	INSTALL(FILES ${DEMO_R2G_SOURCES} DESTINATION src/Spam/Demo/Rag/ COMPONENT demo_sources)
	INSTALL(FILES ${DEMO_R2G_FILES} DESTINATION bin COMPONENT demo_configs)

	SET_PROPERTY(TARGET SpamDemoR2G PROPERTY PROJECT_LABEL "Spam")
	SET_PROPERTY(TARGET SpamDemoR2G PROPERTY FOLDER "Spam/Demo")
	SOURCE_GROUP("Include Files" FILES ${DEMO_R2G_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${DEMO_R2G_FILES})
ENDIF(BUILD_SPAM_DEMO_R2G)

###############################################################################
#
# Packaging
#
###############################################################################

# Windows 32/64 bit
if (WIN32)
	OPTION(CPACK_USE_PACKAGE_SH "Build shell-installable package (.sh)." OFF)
	OPTION(CPACK_USE_PACKAGE_TGZ "Build gziped tarball (.tar.gz)." OFF)
	OPTION(CPACK_USE_PACKAGE_TBZ2 "Build bzip2'd tarball (.tar.bz2)." OFF)
	OPTION(CPACK_USE_PACKAGE_DEB "Build DEB package (.deb)." OFF)
	OPTION(CPACK_USE_PACKAGE_RPM "Build RPM package (.rpm)." OFF)
	OPTION(CPACK_USE_PACKAGE_NSIS "Build NSIS package (.exe)." ON)
# Unix Linux/Apple
elseif (UNIX)
	OPTION(CPACK_USE_PACKAGE_SH "Build shell-installable package (.sh)." OFF)
	OPTION(CPACK_USE_PACKAGE_TGZ "Build gziped tarball (.tar.gz)." OFF)
	OPTION(CPACK_USE_PACKAGE_TBZ2 "Build bzip2'd tarball (.tar.bz2)." OFF)
	OPTION(CPACK_USE_PACKAGE_DEB "Build DEB package (.deb)." ON)
	OPTION(CPACK_USE_PACKAGE_RPM "Build RPM package (.rpm)." OFF)
	OPTION(CPACK_USE_PACKAGE_NSIS "Build NSIS package (.exe)." OFF)
endif()

if (CPACK_USE_PACKAGE_SH OR CPACK_USE_PACKAGE_TGZ OR CPACK_USE_PACKAGE_TBZ2 OR CPACK_USE_PACKAGE_DEB OR CPACK_USE_PACKAGE_RPM OR CPACK_USE_PACKAGE_NSIS)
	# components
	SET(CPACK_COMPONENT_CORE_LIBS_DISPLAY_NAME "C++ libraries")
	SET(CPACK_COMPONENT_CORE_HEADERS_DISPLAY_NAME "C++ headers")
	SET(CPACK_COMPONENT_CORE_CONFIGS_DISPLAY_NAME "Configuration files")
	SET(CPACK_COMPONENT_CORE_LIBS_DESCRIPTION "Spam Core C++ libraries")
	SET(CPACK_COMPONENT_CORE_HEADERS_DESCRIPTION "Spam Core C++ headers")
	SET(CPACK_COMPONENT_CORE_CONFIGS_DESCRIPTION "Spam Core Configuration files")
	SET(CPACK_COMPONENT_CORE_LIBS_GROUP "Core")
	SET(CPACK_COMPONENT_CORE_HEADERS_GROUP "Core")
	SET(CPACK_COMPONENT_CORE_CONFIGS_GROUP "Core")
	SET(CPACK_COMPONENT_GROUP_CORE_DESCRIPTION "Spam Core libraries")

	#SET(CPACK_COMPONENT_DATA_LIBS_DISPLAY_NAME "C++ libraries")
	SET(CPACK_COMPONENT_DATA_HEADERS_DISPLAY_NAME "C++ headers")
	SET(CPACK_COMPONENT_DATA_EXECS_DISPLAY_NAME "Executables")
	SET(CPACK_COMPONENT_DATA_CONFIGS_DISPLAY_NAME "Configuration files")
	SET(CPACK_COMPONENT_DATA_HEADERS_DESCRIPTION "Spam Data C++ headers")
	SET(CPACK_COMPONENT_DATA_EXECS_DESCRIPTION "Spam Data executables")
	SET(CPACK_COMPONENT_DATA_CONFIGS_DESCRIPTION "Spam Data configuration files")
	SET(CPACK_COMPONENT_DATA_HEADERS_GROUP "Data")
	SET(CPACK_COMPONENT_DATA_EXECS_GROUP "Data")
	SET(CPACK_COMPONENT_DATA_CONFIGS_GROUP "Data")
	SET(CPACK_COMPONENT_GROUP_DATA_DESCRIPTION "Spam Data plugins")
	
	SET(CPACK_COMPONENT_APPS_LIBS_DISPLAY_NAME "C++ libraries")
	SET(CPACK_COMPONENT_APPS_HEADERS_DISPLAY_NAME "C++ headers")
	SET(CPACK_COMPONENT_APPS_EXECS_DISPLAY_NAME "Executables")
	SET(CPACK_COMPONENT_APPS_CONFIGS_DISPLAY_NAME "Configuration files")
	SET(CPACK_COMPONENT_APPS_LIBS_DESCRIPTION "Spam Application C++ libraries")
	SET(CPACK_COMPONENT_APPS_HEADERS_DESCRIPTION "Spam Application C++ headers")
	SET(CPACK_COMPONENT_APPS_EXECS_DESCRIPTION "Spam Application executables")
	SET(CPACK_COMPONENT_APPS_CONFIGS_DESCRIPTION "Spam Application configuration files")
	SET(CPACK_COMPONENT_APPS_LIBS_GROUP "Applications")
	SET(CPACK_COMPONENT_APPS_HEADERS_GROUP "Applications")
	SET(CPACK_COMPONENT_APPS_EXECS_GROUP "Applications")
	SET(CPACK_COMPONENT_APPS_CONFIGS_GROUP "Applications")
	SET(CPACK_COMPONENT_GROUP_APPS_DESCRIPTION "Spam Applications")
	
	SET(CPACK_COMPONENT_DEMO_HEADERS_DISPLAY_NAME "C++ headers")
	SET(CPACK_COMPONENT_DEMO_SOURCES_DISPLAY_NAME "C++ sources")
	SET(CPACK_COMPONENT_DEMO_EXECS_DISPLAY_NAME "Executables")
	SET(CPACK_COMPONENT_DEMO_CONFIGS_DISPLAY_NAME "Configuration files")
	SET(CPACK_COMPONENT_DEMO_HEADERS_DESCRIPTION "Spam Demonstration C++ headers")
	SET(CPACK_COMPONENT_DEMO_SOURCES_DESCRIPTION "Spam Demonstration C++ sources")
	SET(CPACK_COMPONENT_DEMO_EXECS_DESCRIPTION "Spam Demonstration executables")
	SET(CPACK_COMPONENT_DEMO_CONFIGS_DESCRIPTION "Spam Demonstration configuration files")

	# install types
	SET(CPACK_ALL_INSTALL_TYPES Complete Developer)
	
	SET(CPACK_COMPONENT_CORE_LIBS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_CORE_HEADERS_INSTALL_TYPES Complete Developer)
	#SET(CPACK_COMPONENT_CORE_EXECS_INSTALL_TYPES Complete)
	SET(CPACK_COMPONENT_CORE_CONFIGS_INSTALL_TYPES Complete Developer)

	#SET(CPACK_COMPONENT_DATA_LIBS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DATA_HEADERS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DATA_EXECS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DATA_CONFIGS_INSTALL_TYPES Complete Developer)

	SET(CPACK_COMPONENT_APPS_LIBS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_APPS_HEADERS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_APPS_EXECS_INSTALL_TYPES Complete)
	SET(CPACK_COMPONENT_APPS_CONFIGS_INSTALL_TYPES Complete Developer)
	
	#SET(CPACK_COMPONENT_DEMO_LIBS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DEMO_HEADERS_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DEMO_SOURCES_INSTALL_TYPES Complete Developer)
	SET(CPACK_COMPONENT_DEMO_EXECS_INSTALL_TYPES Complete)
	SET(CPACK_COMPONENT_DEMO_CONFIGS_INSTALL_TYPES Complete Developer)
	
	# name
	SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
	
	# description
	SET(CPACK_PACKAGE_VENDOR "Claudio Zito")
	SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Simultaneous Perception and Manipulation (SPAM) framework")
	SET(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_ROOT}/Readme.md")
	SET(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_ROOT}/LICENSE")
	set(CPACK_PACKAGE_CONTACT "Claudio Zito <cxz004@cs.bham.ac.uk>")

	# version
	SET(CPACK_PACKAGE_VERSION_2 "1" CACHE STRING "version 2.0")
	SET(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_2}")
	
	if (WIN32)
		if(X86_64)
			SET(CPACK_ARCH "x64")
		else(X86_64)
			SET(CPACK_ARCH "x86")
		endif(X86_64)
	elseif (UNIX)
		if(X86_64)
			SET(CPACK_ARCH "amd64")
		else(X86_64)
			SET(CPACK_ARCH "i386")
		endif(X86_64)
		SET(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
	endif ()
	SET(CPACK_PACKAGE_FILE_NAME	"${CPACK_PACKAGE_NAME}-v${CPACK_PACKAGE_VERSION}-${CPACK_ARCH}")
	#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME}")
	SET(CPACK_PACKAGE_INSTALL_DIRECTORY "Spam")
	
	# Package generators
	if (CPACK_USE_PACKAGE_SH)
		LIST(APPEND CPACK_GENERATOR "STGZ")
	endif ()
	if (CPACK_USE_PACKAGE_TGZ)
		LIST(APPEND CPACK_GENERATOR "TGZ")
	endif ()
	if (CPACK_USE_PACKAGE_TBZ2)
		LIST(APPEND CPACK_GENERATOR "TBZ2")
	endif ()
	if (CPACK_USE_PACKAGE_DEB)
		LIST(APPEND CPACK_GENERATOR "DEB")
		SET(CPACK_DEB_COMPONENT_INSTALL ON)
		SET(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
		SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CPACK_ARCH}")
		SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
		SET(CPACK_COMPONENTS_ALL_IN_ONE_PACKAGE ON)
	endif ()
	if (CPACK_USE_PACKAGE_RPM)
		LIST(APPEND CPACK_GENERATOR "RPM")
		SET(CPACK_RPM_COMPONENT_INSTALL ON)
		SET(CPACK_RPM_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
		SET(CPACK_COMPONENTS_ALL_IN_ONE_PACKAGE ON)
	endif ()
	if (CPACK_USE_PACKAGE_NSIS)
		LIST(APPEND CPACK_GENERATOR "NSIS")
		SET(CPACK_NSIS_COMPONENT_INSTALL ON)
		SET(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_NAME}")
		SET(CPACK_COMPONENTS_ALL_IN_ONE_PACKAGE ON)
	endif ()

	# CPack
	INCLUDE(CPack)
endif ()
