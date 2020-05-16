project(VIG_FactorGraphOptimization)

set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_FLAGS "-std=c++11")

#Eigen,OpenCV
find_package(Eigen3 REQUIRED)
find_package( OpenCV 3.0.0 REQUIRED )
find_package(Pangolin REQUIRED)

#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs
set(GTSAM_UNSTABLE_LIBRARIES gtsam_unstable)

######  include and library directories.
include_directories(
	${EIGEN3_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include/gtsam_derived
	${PROJECT_SOURCE_DIR}/include/test
	)

add_library(${PROJECT_NAME} SHARED
	src/VIGCommon.cpp
	src/VIG_FactorGraph_GTSrc.cpp
	src/VIGSystem.cpp
	src/Configuration.cpp
	src/IMU.cpp
	src/GNSS.cpp
	src/FeatureTracker.cpp
	src/FeatureManager.cpp
	src/FactorGraphEstimator.cpp
	src/VIGInitializer.cpp
	src/Viewer.cpp

	src/gtsam_derived/drPreintegratedImuMeasurements.cpp

	src/test/TestVSlamSystem.cpp
	)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${Pangolin_LIBRARIES}
	# "gtsam"
	# "gtsam_unstable"
	${GTSAM_LIBRARIES}
	${GTSAM_UNSTABLE_LIBRARIES}
	)

#############################
add_executable(VIG_FactorGraph_GTSrc src/VIG_FactorGraph_GTSrc.cpp)
target_link_libraries(VIG_FactorGraph_GTSrc ${PROJECT_NAME})
