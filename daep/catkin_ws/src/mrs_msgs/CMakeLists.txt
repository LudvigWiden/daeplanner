cmake_minimum_required(VERSION 3.5)
project(mrs_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg/mpc_tracker FILES

  FuturePoint.msg
  FutureTrajectory.msg
  MpcPredictionFullState.msg
  MpcTrackerDiagnostics.msg
  )

# messages for GIMBAL
add_message_files(DIRECTORY msg/gimbal FILES

  GimbalState.msg
  )

# messages for GNSS
add_message_files(DIRECTORY msg/gnss FILES

  RtkFixType.msg
  RtkGps.msg
  GpsData.msg
  )

# messages for LANDOFF_TRACKER
add_message_files(DIRECTORY msg/landoff_tracker FILES

  LandoffDiagnostics.msg
  )

# messages for GazeboSimulation
add_message_files(DIRECTORY msg/gazebo_simulation FILES

  GazeboSpawnerDiagnostics.msg
  )

# messages for the UAV_MANAGERS
add_message_files(DIRECTORY msg/uav_managers FILES

  BumperStatus.msg
  ConstraintManagerDiagnostics.msg
  ControlError.msg
  ControllerDiagnostics.msg
  ControllerStatus.msg
  ControlManagerDiagnostics.msg
  DynamicsConstraints.msg
  EstimationDiagnostics.msg
  EstimatorCorrection.msg
  EstimatorDiagnostics.msg
  EstimatorInput.msg
  EstimatorOutput.msg
  EulerAngles.msg
  GainManagerDiagnostics.msg
  PathReference.msg
  ReferenceList.msg
  Reference.msg
  ReferenceStamped.msg
  TrackerCommand.msg
  TrackerStatus.msg
  TrajectoryReference.msg
  UavManagerDiagnostics.msg
  UavState.msg
  VelocityReference.msg
  VelocityReferenceStamped.msg
  )

# messages for the UAV_STATUS
add_message_files(DIRECTORY msg/uav_status FILES

  CustomTopic.msg
  NodeCpuLoad.msg
  UavStatus.msg
  UavStatusShort.msg
  )

# messages for the SE3_CONTROLLER
add_message_files(DIRECTORY msg/se3_controller FILES

  Se3Gains.msg
  )

# messages for the TRAJECTORY_GENERATION
add_message_files(DIRECTORY msg/trajectory_generation FILES

  Path.msg
  PathWithVelocity.msg
  ReferenceWithVelocity.msg
  )

# services for UAV_MANAGERS
add_service_files(DIRECTORY srv/uav_managers FILES

  BumperParamsSrv.srv
  ConstraintsOverride.srv
  DynamicsConstraintsSrv.srv
  ReferenceSrv.srv
  ReferenceStampedSrv.srv
  TrajectoryReferenceSrv.srv
  TransformPoseSrv.srv
  TransformReferenceSrv.srv
  TransformVector3Srv.srv
  ValidateReferenceList.srv
  ValidateReference.srv
  VelocityReferenceSrv.srv
  VelocityReferenceStampedSrv.srv
  )

# messages for the PROFILER
add_message_files(DIRECTORY msg/profiler FILES

  ProfilerUpdate.msg
  )

# messages for the MRS_BUMPER
add_message_files(DIRECTORY msg/bumper FILES

  Histogram.msg
  ObstacleSectors.msg
  )

# messages for SPEED_TRACKER
add_message_files(DIRECTORY msg/speed_tracker FILES

  SpeedTrackerCommand.msg
  )

# general messages
add_message_files(DIRECTORY msg/general FILES

  Float64ArrayStamped.msg
  Float64.msg
  Float64MultiArrayStamped.msg
  Float64Stamped.msg
  ImageLabeledArray.msg
  ImageLabeled.msg
  PoseWithCovarianceArrayStamped.msg
  PoseWithCovarianceIdentified.msg
  Sphere.msg
  StringStamped.msg
  TrackArrayStamped.msg
  Track.msg
  TrackStamped.msg
  )

# general services
add_service_files(DIRECTORY srv/general FILES

  Float64Srv.srv
  Float64StampedSrv.srv
  GetFloat64.srv
  SetInt.srv
  String.srv
  Vec1.srv
  Vec4.srv
  )

# services for TRAJECTORY_GENERATION
add_service_files(DIRECTORY srv/trajectory_generation FILES

  PathSrv.srv
  GetPathSrv.srv
  )

# messages for stamped_msgs
add_message_files(DIRECTORY msg/stamped_msgs FILES

  UInt16Stamped.msg
  BoolStamped.msg
  )

add_message_files(DIRECTORY msg/hw_api FILES

  HwApiAccelerationHdgCmd.msg
  HwApiAccelerationHdgRateCmd.msg
  HwApiActuatorCmd.msg
  HwApiAltitude.msg
  HwApiAttitudeCmd.msg
  HwApiAttitudeRateCmd.msg
  HwApiCapabilities.msg
  HwApiControlGroupCmd.msg
  HwApiPositionCmd.msg
  HwApiRcChannels.msg
  HwApiStatus.msg
  HwApiVelocityHdgCmd.msg
  HwApiVelocityHdgRateCmd.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs
  )

set(LIBRARIES
  MrsMsgs_Main
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsMsgs_Main
  src/main.cpp
  )

add_dependencies(MrsMsgs_Main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMsgs_Main
  ${catkin_LIBRARIES}
  )

add_custom_command(TARGET MrsMsgs_Main
  PRE_BUILD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMAND bash -c "${CMAKE_CURRENT_SOURCE_DIR}/link_msg_files.sh"
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
