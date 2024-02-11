^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-01-20)
------------------
* dofec srv update
* adding dofec salvo msg
* updated big dofec messages
* adding messages for big dofec board
* adding led strip driver mrs module service
* adding radar msg and updated status msg
* added collison avoidance info
* new RTK messages
* Contributors: Dan Hert, Tomas Baca

1.0.3 (2022-05-09)
------------------
* added uwb estimator type
* typo fix in msg description
* added ImageLabeled and ImageLabeledArray msgs
* update cmake with ImageLabeled
* nimbro test service
* added constraints override
* added install for msg and srv
* + install in cmakelists
* added nimbro test msg
* added llcp msg
* added tarot gimbal msg
* update mrs_pcl_tools/PclToolsDiagnostics.msg
* add mrs_pcl_tools/PclToolsDiagnostics.msg
* added pixels_per_column, columns_per_frame and pixel_shift_by_row to OusterInfo message (`#5 <https://github.com/ctu-mrs/mrs_msgs/issues/5>`_)
  Co-authored-by: vasek <uav25>
* Contributors: Dan Hert, Matej Petrlik, Myralllka, Pavel Petracek, Tomas Baca, Václav Pritzl

1.0.2 (2021-10-03)
------------------
* add UInt16Stamped msg
* added cpu temp to mrs_status
* added input_id for mpc and traj. gen
* added rc mode to control manager diag
* Add override jerk constraints in path msg
* updated docs build script
* added cont.&track. flags regarding human activation
  added Track.msg
* added gimbal PRY message
* added TrackStamped message
* added the Track and TrackArrayStamped messages
* added PathWithVelocity message
* added pathfinder msg
* TrajectoryGeneration: added getPath service
* MpcTracker: updated fullstate prediction
* updated path msg
* Contributors: Dan Hert, Matouš Vrba, Pavel Petracek, Tomas Baca, Vit Kratky

1.0.1 (2021-05-16)
------------------
* updated path
* added velocity reference
* updates for mrs status
* fixed msg linking script
* added ram total to mrs_status msg
* added AloamgarmDebug msg
* added UavManagerDiagnostics
* updated full state's vector
* updated subtypes of PositionCommand
* added full state MPC prediction message
* added ALOAMREP heading estimator type for repredictor testing
* added another estimator type for testing of ALOAM with repredictor
* added aloamgarm altitudetype
* Contributors: Daniel Hert, Matej Petrlik, Tomas Baca, Vaclav Pritzl

1.0.0 (2021-03-18)
------------------
* Major release

0.0.6 (2021-03-16)
------------------
* Noetic-compatible
* update msg linking script
* new cmake version
* Contributors: Daniel Hert, Jan Bednar, Matej Petrlik, Matouš Vrba, Pavel Petracek, Robert Penicka, Tomas Baca, Tomáš Báča, Viktor Walter, klaxalk, mergify[bot]

0.0.5 (2020-02-26)
------------------
* nothing changed here ;-)

0.0.4 (2020-02-18)
------------------
* added bumper params msg
* serial msg for raw message
* set TOWER estimator type number to a unique value
* TOWER lcalizatino types
* added services for reference validation
* added flying_normally to control manager diag
* adde motors to control manager diag
* added SetInt srv
* added rampup info to AttitudeCmd
* added gain and constraint diagnostics
* added missing alt odom type
* added new srv
* updated mpc diagnostics msg
* added ControlError.msg
* new constraints message, changed the constraints srv
* Adding pose array message for the fire_detect package
* updated attitude_cmd
* add Float64MultiArrayStamped.msg to cmake
* add Float64MultiArrayStamped.msg
* Add ALOAM altitude estimator
* Add aloam slam estimator type
* updated profiler's rate
* BRICKFLOW heading estimator
* change altitude estimator service
* new altitude estimators
* added new reference messages
* new message and service types
* updated speed tracker's message
* removed start idx from the tracker trajectory
* added swarming command message
* upated uav_state msg
* removed orientation from UavState, it is already in the pose
* added UavState message
* added ICP estimator type
* Contributors: Dan Hert, Matej Petrlik, Matej Petrlik (desktop), Pavel Petracek, Pavel Petráček, Robert Penicka, Tomas Baca, Viktor Walter, Vit Kratky

0.0.3 (2019-10-25)
------------------
* added bumper status
* height available in odometry diagnostics
* added mpc tracker diagnostsics collision avoidance
* added the constraints override feature for controllers
* Remove GimbalPitch.srv
* added other uav avoidance trajectoris to mpc diagnostics
* vslam pose estimator
* +gripper
* VIO heading type
* added landoff diagnostics
* added disturbances to attitude command
* added PlannerTask service
* added rviz cylinder msgs
* Contributors: Dan Hert, Matej Petrlik, Matej Petrlik (desktop), Pavel Petráček, Tomas Baca, uav61

0.0.2 (2019-07-01)
------------------
* + BRICKFLOW estimator
* updated AttitudeCommand
* Contributors: Matej Petrlik, Matej Petrlik (desktop), NAKI, Pavel Petráček, Tomas Baca, Tomáš Báča, Vojtech Spurny

0.0.1 (2019-05-20)
------------------
