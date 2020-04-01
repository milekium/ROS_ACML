# whereIam
Udacity Robotics Software Engineer Nanodegree Program, project 3

## Dependenties
for the project
```bash
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-amcl
```
for PGM Map Creator 
```
sudo apt-get install libignition-math2-dev protobuf-compiler
```

## Parameters:


### map server 
frame_id: "map" # The frame to set in the header of the published map.	(string, default: "map")

### amcl 
initial_pose_x: -7.5 # initial x-coordinate value
initial_pose_y: 11.5 # initial y-coordinate value
initial_pose_a: -1.57079 # initial yaw coordinate value
min_particles: 1	
max_particles: 50
update_min_d: 0.15	#Translational movement required before performing a filter update.(default: 0.2 meters)
update_min_a: 0.2 #Rotational movement required before performing a filter update. (default: π/6.0 radians)0.5236
kld_err: 0.01 # Maximum error between the true distribution and the estimated distribution. (double, default: 0.01)
kld_z: 0.99 # Upper standard normal quantile for (1 - p), where p is the probability that the error on the estimated distrubition will be less than kld_err.(double, default: 0.99)
resample_interval: 2 # Number of filter updates required before resampling.(int, default: 2)
transform_tolerance: 0.2 # Time with which to post-date the transform that is published, to indicate that this transform is valid into the future. (double, default: 0.1 seconds)
recovery_alpha_slow: 0.0 # Exponential decay rate for the slow average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.001.(double, default: 0.0 (disabled))
recovery_alpha_fast: 0.0 # Exponential decay rate for the fast average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.1.(double, default: 0.0 (disabled))
do_beamskip: true # when true skip laser scan when scan doesn't work for a mayority of particles
beam_skip_distance: 0.003 # Distance from a valid map point before scan is consider invalid 
beam_skip_threshold: 0.1 # Ratio of samples of which the scans are valid to consider a valid scan		 		
initial_cov_xx: 0.5*0.5 # Initial pose covariance (x*x), used to initialize filter with Gaussian distribution.(double, default: 0.5*0.5 meters)
initial_cov_yy: 0.5*0.5 # Initial pose covariance (y*y), used to initialize filter with Gaussian distribution.(double, default: 0.5*0.5 meters)
initial_cov_aa: 0.2618*0.2618 # Initial pose covariance (yaw*yaw), used to initialize filter with Gaussian distribution.(double, default: (π/12)*(π/12) radian)
gui_publish_rate: 10 # Maximum rate (Hz) at which scans and paths are published for visualization, -1.0 to disable.(double, default: -1.0 Hz)
save_pose_rate: 1 # Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used on subsequent runs to initialize the filter. -1.0 to disable. (double, default: 0.5 Hz)
use_map_topic: false # When set to true, AMCL will subscribe to the map topic rather than making a service call to receive its map. New in navigation 1.4.2 (bool, default: false)
first_map_only: false # When set to true, AMCL will only use the first map it subscribes to, rather than updating each time a new one is received. New in navigation 1.4.2 (bool, default: false)

#### Laser model parameters
laser_model_type: likelihood_field # Which model to use, either beam, likelihood_field, or likelihood_field_prob.("likelihood_field")
laser_min_range: -1.0 # Minimum scan range to be considered; -1.0 will cause the laser's reported minimum range to be used.(double, default: -1.0)
laser_max_range: -1.0 # Maximum scan range to be considered; -1.0 will cause the laser's reported maximum range to be used.(double, default: -1.0)
laser_max_beams: 30 # How many evenly-spaced beams in each scan to be used when updating the filter.(int, default: 30)
laser_z_hit: 0.95 # Mixture weight for the z_hit part of the model.(double, default: 0.95)
laser_z_short: 0.05
laser_z_max: 0.05
laser_z_rand: 0.05 # Mixture weight for the z_rand part of the model.(double, default: 0.05)
laser_sigma_hit: 0.2 # Standard deviation for Gaussian model used in z_hit part of the model.(double, default: 0.2 meters)
laser_lambda_short: 0.1 # Exponential decay parameter for z_short part of model. (double, default: 0.1)
laser_likelihood_max_dist: 2.0 # Maximum distance to do obstacle inflation on map, for use in likelihood_field model.(double, default: 2.0 meters)
    
#### Odometry model parameters
odom_model_type: omni # Which model to use, either "diff", "omni", "diff-corrected" or "omni-corrected".(string, default: "diff")
odom_alpha1: 0.2 # Specifies the expected noise in odometry's rotation estimate from the rotational component of the robot's motion.(double, default: 0.2)
odom_alpha2: 0.2 # Specifies the expected noise in odometry's rotation estimate from translational component of the robot's motion.(double, default: 0.2)
odom_alpha3: 0.8 # Specifies the expected noise in odometry's translation estimate from the translational component of the robot's motion.(double, default: 0.2)
odom_alpha4: 0.2 # Specifies the expected noise in odometry's translation estimate from the rotational component of the robot's motion.(double, default: 0.2)
odom_alpha5: 0.2 # Translation-related noise parameter (only used if model is "omni").(double, default: 0.2)
odom_frame_id: odom # Which frame to use for odometry.(string, default: "odom")
base_frame_id: robot_footprint # Which frame to use for the robot base(string, default: "base_link")
global_frame_id: map # The name of the coordinate frame published by the localization system(string, default: "map")
tf_broadcast: false # Set this to false to prevent amcl from publishing the transform between the global frame and the odometry frame.(bool, default: true)

### move_base 
base_global_planner: navfn/NavfnROS
base_local_planner: base_local_planner/TrajectoryPlannerROS
recovery_behaviors:  [ { name: "conservative_reset", type: "clear_costmap_recovery/ClearCostmapRecovery"}, {name: "rotate_recovery", type: "rotate_recovery/RotateRecovery"}, {name: "aggressive_reset", type: "clear_costmap_recovery/ClearCostmapRecovery" } ] 
planner_patience: 5 # How long the planner will wait in seconds in an attempt to find a valid plan before space-clearing operations are performed.(double, default: 5.0)
controller_patience: 15 # How long the controller will wait in seconds without receiving a valid control before space-clearing operations are performed.(double, default: 15.0)
conservative_reset_dist: 3 # The distance away from the robot in meters beyond which obstacles will be cleared from the costmap when attempting to clear space in the map. Note, this parameter is only used when the default recovery behaviors are used for move_base.(double, default: 3.0)
recovery_behavior_enabled: true # Whether or not to enable the move_base recovery behaviors to attempt to clear out space.(bool, default: true)
clearing_rotation_allowed: false # Determines whether or not the robot will attempt an in-place rotation when attempting to clear out space. Note: This parameter is only used when the default recovery behaviors are in use, meaning the user has not set the recovery_behaviors parameter to anything custom.(bool, default: true)
shutdown_costmaps: false # Determines whether or not to shutdown the costmaps of the node when move_base is in an inactive state(bool, default: false)
oscillation_timeout: 0.0 # How long in seconds to allow for oscillation before executing recovery behaviors. A value of 0.0 corresponds to an infinite timeout. New in navigation 1.3.1(double, default: 0.0)
oscillation_distance: 0.3 # How far in meters the robot must move to be considered not to be oscillating. Moving this far resets the timer counting up to the ~oscillation_timeout New in navigation 1.3.1(double, default: 0.5)
planner_frequency: 1 # The rate in Hz at which to run the global planning loop. If the frequency is set to 0.0, the global planner will only run when a new goal is received or the local planner reports that its path is blocked. New in navigation 1.6.0(double, default: 0.0)
max_planning_retries: -1 # How many times to allow for planning retries before executing recovery behaviors. A value of -1.0 corresponds to an infinite retries.(int32_t, default: -1)
oscillation_reset_dist: 0.02(double, default: 0.05) How far the robot must travel in meters before oscillation flags are reset.
### TrajectoryPlannerROS:
#### Robot Configuration Parameters, these are the velocity limit of the
robotmax_vel_x: 0.3
min_vel_x: 0.1

#### Angular velocity limit
max_vel_theta: 1.0
min_vel_theta: -1.0
min_in_place_vel_theta: 0.6
#### These are the acceleration limits of the robot
acc_lim_x: 0.5
acc_lim_theta: 1.0
#### Goal Tolerance Parameters: The tolerance of robot when it reach the goal position
yaw_goal_tolerance: 0.3
xy_goal_tolerance: 0.15
# Forward Simulation Parameters
sim_time: 4. amount of time (in seconds) to forward-simulate trajectories.
vx_samples: 8. number of samples to use when exploring the x velocity space.
vtheta_samples: 20. number of samples to use when exploring the theta velocity space respectively.
controller _ frequency: 4. the frequency at which this controller will be called
# Trajectory Scoring Parameters
meter_scoring: true
pdist_scale: 0.6
gdist_scale: 0.8
occdist_scale: 0.01
heading_lookahead: 0.325
dwa: true.  DWA Local Planner maximizes an objective function to obtain optimal velocity pairs
# Oscillation Prevention Parameters
oscillation_reset_dist: 0.05
# Differential-drive robot configuration : If the robot is holonomic configuration, set to true other vice set to false.
holonomic_robot: false
max_vel_y: 0.0
min_vel_y: 0.0
acc_lim_y: 0.0
vy_samples: 1



## Citations
Notes taken from:
-Udacity Robotics Software Engineer Nanodegree Program
-RobotOperatingSystem(ROS)TheCompleteReferenceVolume2 book.
-Mastering ROS for Robotics Programming book.
-ROS Navigation Tuning Guide: https://arxiv.org/pdf/1706.09068.pdf
