#pragma once

#include <mutex>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "drone_racing/data_saver.h"
#include "drone_racing/minimum_jerk_trajectory.h"
#include "drone_racing/trajectory_base.h"


// use for what lines
using namespace RapidQuadrocopterTrajectoryGenerator; // RQTG



namespace drone_racing 
{

    // Tell compiler these classes exists, maybe better with includes
    class DroneRacing;
    class Visualizer;
    class DataSaver;



    class TrajectoryManager 
    {

        public: // Con- and destructors

            explicit TrajectoryManager( const ros::NodeHandle& pnh );
            
            virtual ~TrajectoryManager();



        public: // Methods
            
            // Input: bool to determine if trajectory is model based
            // Calls trajectory_implementation_->clearTrajectories(is_mb).
            void clearTrajectories( const bool is_mb );

            // Input:
            // - Ptr to instance of class DroneRacing
            // Sample Trajectory
            // - if show_full_trajectory_==true: 
            //      trajectory = sampleTrajectory (
            //          trajectory_implementation_->getEndTimeFromTrajectory(), 
            //          trajectory_dt);
            // - if show_full_trajectory_==false: 
            //      end_time = std::min(trajectory_implementation_->getEndTimeFromTrajectory(), viz_horizon_);
            //      trajectory = sampleTrajectory(end_time, trajectory_dt);
            // Show trajectory
            // - if use_mb_ == false:
            //      reactive_nav->visualizer_->visualizeTrajectory(trajectory, use_mb_, Visualizer::Color::kGreen, 0);
            // - if use_mb_ == true:
            //      reactive_nav->visualizer_->visualizeTrajectory(trajectory, use_mb_, Visualizer::Color::kRed, 0);
            void visualizeTrajectory( DroneRacing* reactive_nav );


            // Input: 
            // - index of best trajectory?
            // - bool to determine if trajectory is model based
            // Calls 
            // - trajectory_implementation_->setBestTrajectory(best_traj_idx, is_mb)
            // - trajectory_implementation_->clearTrajectories( is_mb
            // Set
            // use_mb_ = is_mb;
            void setBestTraj (
                const int best_traj_idx, 
                const bool is_mb
            );


            // Input:
            // - duration
            // - time increment
            // Creates trajectory with time points t=0, dt, 2dt, ..., ~ duration and returns it
            // - trajectory_implementation_->getDesiredStateFromTrajectory( t )
            std::vector<quadrotor_common::TrajectoryPoint> sampleTrajectory (
                const double duration, 
                const double dt
            );


            // return trajectory_implementation_->getDesiredStateFromTrajectory();
            quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory();


            // Calls trajectory_implementation_->setStartTime();
            void setStartTime();

            // Input:
            // - ptr to instance of class DroneRacing (unused)
            // - start state
            // - state estimate
            // - network selection
            //
            // Calculates 
            // - desired_velocity: denormalize network_selection.z (input arg) with max_velocity_
            //      - >min_velocity_
            // - planing_length_rescaled = planning_length_*desired_velocity 
            //      - >planning_length_min_
            //      - <planning_length_max_
            // - yaw: denormalize network_selection.x (input arg) with camera_semi_fov_rad_yaw_
            // - pitch: denormalize network_selection.y (input arg) with camera_semi_fov_rad_pitch_
            // - goal_b (vector from start to end state):  f(planning_length_rescaled, yaw, pitch)
            //      - as geometry_msgs::Pose goal_pose;
            //      - as rpg::Pose T_B_goal
            // - T_W_B: transform from world to body frame of quad from state estimate (input arg)
            // - T_W_goal: transform from world to goal = T_W_B * T_B_goal
            // - end_state: position from T_W_goal
            // - success trajectory_implementation_->computeTrajectory(
            //          start state (input), end_state, only position constrained, desired_velocity, not model based)
            //
            // Return success
            bool computeTrajectory (
                DroneRacing* reactive_nav,
                const quadrotor_common::TrajectoryPoint& start_state,
                const nav_msgs::Odometry state_estimate,
                Eigen::Vector3d network_selection
            );



        private: // Methods

            void loadParameters();



        private: // Member variables

            std::unique_ptr<drone_racing::TrajectoryBase> trajectory_implementation_;

            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;

            double max_velocity_;
            double min_velocity_;
            // Planning Parameters
            double planning_length_;
            double planning_length_min_;
            double planning_length_max_;
            double global_traj_horizon_;

            double camera_semi_fov_rad_yaw_;
            double camera_semi_fov_rad_pitch_;
            double theta_gain_;

            // Trajectory Visualization
            double trajectory_viz_freq_;
            bool show_full_trajectory_;
            double viz_horizon_;

            bool use_mb_;
    };



}













/* DESCRIPTION

When an instance of the class TrajectoryManager is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &pnh (input arg)
            (double)            max_velocity_               =   {"max_velocity", 1.0}
            (double)            min_velocity_               =   {"min_velocity", 0.5}
            (double)            global_traj_horizon_        =   {"global_traj_horizon_", 1.0}
            (double)            planning_length_            =   {"planning_length", 2.0}
            (double)            planning_length_min_        =   {"planning_length_min", 2.0}
            (double)            planning_length_max_        =   {"planning_length_max", 2.0}
            (double)            camera_semi_fov_rad_yaw_    =   {"camera_fov_yaw", 45.0} * pi/180
            (double)            camera_semi_fov_rad_pitch_  =   {"camera_fov_pitch", 45.0} * pi/180
            (double)            theta_gain_                 =   {"theta_gain", 1.0}
            (double)            trajectory_viz_freq_        =   {"trajectory_viz_freq", 10.0}
            (double)            viz_horizon_                =   max( {"viz_horizon", 1.0}, 1/trajectory_viz_freq_ )
            (bool)              show_full_trajectory_       =   {"show_full_trajectory", false}

            (std::unique_ptr<drone_racing::TrajectoryBase>)
                                trajectory_implementation_.reset( new drone_racing::MinimumJerkTrajectory() )
            



The following public methods are provided:

    A)  void setStartTime()

            1)  trajectory_implementation_->time_start_best_trajectory_ = ros::Time::now();

    B)  void clearTrajectories( const bool is_mb )

            1)  Erase all elements from std::vectors, 
                trajectory_implementation_->trajectories_mb_ or ...->trajectories_nw_, 
                according to input bool arg

    C)  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory()

            1)  Return state (pos,vel,acc,heading) from trajectory_implementation_->best_traj_ at time, now - trajectory_implementation_->time_start_best_trajectory_ + 0.02


    D)  void visualizeTrajectory( DroneRacing* reactive_nav )
            
            1)  Samples Trajectory (according to bools show_full_trajectory_, viz_horizon_)
                    - trajectory = sampleTrajectory( ... )

            2)  Visualize Trajectory in red or green depending on bool use_mb_
                    - reactive_nav->visualizer_->visualizeTrajectory(trajectory, use_mb_, red/green, 0);

    E)  std::vector<quadrotor_common::TrajectoryPoint> sampleTrajectory (
            const double duration, 
            const double dt
        );

            1)  Returns trajectory with time points t=0, dt (input arg), 2dt, ..., ~ duration (input arg)
                    - using trajectory_implementation_->getDesiredStateFromTrajectory( t )


    F)  bool computeTrajectory (
            DroneRacing* reactive_nav,
            const quadrotor_common::TrajectoryPoint& start_state,
            const nav_msgs::Odometry state_estimate,
            Eigen::Vector3d network_selection
        );

            1) Caluclates endstate in world frame

            2) success trajectory_implementation_->computeTrajectory(
                    start state (input), end_state, only position constrained, desired_velocity, not model based)
            
            3) Return success
    
    G)  void setBestTrajectory (
            const int traj_idx, 
            const bool is_mb
        )
        
            1)  Calls trajectory_implementation_->setBestTrajectory(best_traj_idx, is_mb)

            2)  Calls trajectory_implementation_->clearTrajectories( is_mb )

            3)  Set use_mb_ = is_mb;



*/




/* NOTES

dont use ptrs, try to create all on the stack
includes
rm using namspace ...

3 cpp units to only one

*/