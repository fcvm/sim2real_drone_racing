#pragma once

#include <ros/ros.h>

#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <quadrotor_common/trajectory_point.h>




namespace drone_racing 
{


    class DroneRacing;

    class Visualizer;



    class GlobalTrajectory 
    {
        public: // Con- and destructor

            GlobalTrajectory(const ros::NodeHandle& pnh);

            virtual ~GlobalTrajectory();


        public: // Methods
    
            // Input:
            // - ptr to visualizer instance
            // - vector of 3d goal positions
            //
            // Extract goal positions that are at least 0.5 distant into vector of waypoints: waypoints_filtered
            // Calculate segment times between waypoints as distance/global_traj_max_v_: segment_times
            // Set minimization_weights (velocity, acceleration, jerk, snap)?
            // Create trajectory_settings with waypoints, minimization weights, poly order 11, conti order 4
            // Set maximal_des_thrust and maximal_roll_pitch_rate
            // If load_existing_trajectory_==true: call loadGlobalTrajectory(): load global_trajectory_sampled_ from traj_dir_ + "/global_trajectory.txt";
            // Else:
            //      generate minimum snap trajectory from 
            //          - segment times, trajectory settings, global_traj_max_v_, maximal_des_thrust, maximal_roll_pitch_rate
            //      sample trajectory with dt=0.05: global_trajectory_sampled_
            //      call saveGlobalTrajectory(): save global_trajectory_sampled_ to traj_dir_ + "/global_trajectory.txt";
            // visualize global_trajectory_sampled_ (visualizer->visualizeTrajectory)
            // Set trajectory_exists_ = true
            void generateGlobalTrajectory (
                std::shared_ptr<Visualizer>& visualizer,
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& goal_positions_
            );

            // Input:
            // - double horizon
            //
            // Starting from global_trajectory_sampled_[global_traj_idx_proj_], 
            //      return the next element with distance > horizon (input arg)
            quadrotor_common::TrajectoryPoint getNextStateOnGlobalTraj( const double horizon );

            // Input: 
            // - curr_desired_state
            //
            // 1) 
            // Find vector
            //      from:   global_trajectory_sampled_[global_traj_idx_proj_-1]
            //      to:     global_trajectory_sampled_[global_traj_idx_proj_]
            // Find the norm of above
            // Find vector:
            //      from:   global_trajectory_sampled_[global_traj_idx_proj_-1]
            //      to:     curr_desired_state
            // Find dot product of both vectors devided by norm
            //  in loop increase global_traj_idx_proj_ until 
            //      - the dot product <= 1
            //      - global_traj_idx_proj_ < global_trajectory_sampled_.size()
            //
            // 2) If global_traj_idx_proj_ exceeds global_trajectory_sampled_.size() set to 1
            //
            // 3) 
            // Find vector:
            //      from:   global_trajectory_sampled_[global_traj_idx_proj_]
            //      to:     curr_desired_state
            // If norm of vector >1:
            //      call findStartIndexGlobalTrajectory( curr_desired_state.position )
            //
            // 4) return global_trajectory_sampled_[ global_traj_idx_proj_ ];
            quadrotor_common::TrajectoryPoint projectOnTrajectory( quadrotor_common::TrajectoryPoint curr_desired_state );

            // Input: 
            // - position
            //
            // Sets global_traj_idx_nw_ and global_traj_idx_proj_ that 
            //      (global_trajectory_sampled_[i] - position).norm minimal
            // Sets global_v_max_ and global_v_min_
            void findStartIndexGlobalTrajectory( const Eigen::Vector3d est_pos );

            // Input:
            // - ptr to instance of DroneRacing
            //
            // Sets position to reactive_nav->getEstimatedPos()
            // Sets global_traj_idx_nw_ that 
            //      (global_trajectory_sampled_[i] - position).norm minimal
            void findStartIndexNWGlobalTrajectory( DroneRacing* reactive_nav );

            // Return global_v_max_
            double getMaxVelocity();

            // Return trajectory_exists_
            bool exists();


        private: // Methods

            void saveGlobalTrajectory();

            void loadGlobalTrajectory();

            void loadParameters();


        private: // Member variables

            ros::NodeHandle pnh_;

            std::vector<quadrotor_common::TrajectoryPoint> global_trajectory_sampled_;
            double global_v_max_ = 0.0;
            double global_v_min_ = 100.0;

            double global_traj_max_v_;
            double global_traj_horizon_;

            // hack for network
            int global_traj_idx_nw_ = 1;
            Eigen::Vector3d tracking_dir_nw_;
            double tracking_dist_nw_;

            // hack for projection
            int global_traj_idx_proj_ = 1;
            Eigen::Vector3d tracking_dir_proj_;
            double tracking_dist_proj_;
            bool handheld_experiment_;
            bool trajectory_exists_ = false;
            std::string root_dir_;

            std::string traj_dir_;

            bool load_existing_trajectory_;
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> waypoints_;

    };


} /* namespace drone_racing */











/* DESCRIPTION

When an instance of the class GlobalTrajectory is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &pnh (input arg)
            (double)            global_traj_max_v_          =   {"global_traj_max_v", 1.0}
            (double)            global_traj_horizon_        =   {"global_traj_horizon", 0.5}
            (bool)              handheld_experiment_        =   {"handheld_experiment", false}
            (std::string)       root_dir_                   =   {"root_dir", ""}
            (bool)              load_existing_trajectory_   =   {"load_existing_trajectory", false}
            (std::string)       traj_dir_                   =   {"trajectory_path", ""}

            



The following public methods are provided:

    A)  void generateGlobalTrajectory (
            std::shared_ptr<Visualizer>& visualizer,
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& goal_positions_
        )

        1)  Extract goal positions that are at least 0.5 distant into vector of waypoints: waypoints_filtered
            Calculate segment times between waypoints as distance/global_traj_max_v_: segment_times
            Set minimization_weights (velocity, acceleration, jerk, snap)?
            Create trajectory_settings with waypoints, minimization weights, poly order 11, conti order 4
            Set maximal_des_thrust and maximal_roll_pitch_rate to hard coded values

        2)  If load_existing_trajectory_==true: 
                Call loadGlobalTrajectory(): load global_trajectory_sampled_ from file traj_dir_ + "/global_trajectory.txt";
            Else:
                 generate minimum snap trajectory from 
                     - segment times, trajectory settings, global_traj_max_v_, maximal_des_thrust, maximal_roll_pitch_rate
                 sample trajectory with dt=0.05: global_trajectory_sampled_
                 call saveGlobalTrajectory(): save global_trajectory_sampled_ to traj_dir_ + "/global_trajectory.txt";
        
        3)  visualize global_trajectory_sampled_ (visualizer->visualizeTrajectory)
            Set trajectory_exists_ = true
    

    B)  quadrotor_common::TrajectoryPoint getNextStateOnGlobalTraj( const double horizon )

        1)  Starting from global_trajectory_sampled_[global_traj_idx_proj_]:
                Return the next element with distance > horizon (input arg)

    
    C)  quadrotor_common::TrajectoryPoint projectOnTrajectory( quadrotor_common::TrajectoryPoint curr_desired_state )

        1)  Calculate trajectory direction vector
                 from:   global_trajectory_sampled_[global_traj_idx_proj_-1]
                 to:     global_trajectory_sampled_[global_traj_idx_proj_]
            And its norm
        
            Find divergence vector:
                 from:   global_trajectory_sampled_[global_traj_idx_proj_-1]
                 to:     curr_desired_state

            Find dot product of both vectors each devided by norm of trajectory direction vector
        
            In loop increase global_traj_idx_proj_ while
                 - the dot product > 1
                 - global_traj_idx_proj_ < global_trajectory_sampled_.size()
        
        2)  If global_traj_idx_proj_ exceeds global_trajectory_sampled_.size() set to 1
        
        3)  Find divergence vector:
                from:   global_trajectory_sampled_[global_traj_idx_proj_]
                to:     curr_desired_state
            
            If norm of vector >1:
                Call findStartIndexGlobalTrajectory( curr_desired_state.position )
        
        4)  Return global_trajectory_sampled_[ global_traj_idx_proj_ ];

    D)  void findStartIndexGlobalTrajectory( const Eigen::Vector3d est_pos )

        1)  Sets 
                - global_traj_idx_nw_ 
                - global_traj_idx_proj_
            by 
                min - global_trajectory_sampled_[i] - est_pos).norm
        
        2)  Sets global_v_max_ and global_v_min_
    
    E)  void findStartIndexNWGlobalTrajectory( DroneRacing* reactive_nav )

        1)  Sets position to reactive_nav->getEstimatedPos()
            
            Sets global_traj_idx_nw_ 
            by 
                min - (global_trajectory_sampled_[i] - position).norm minimal

    F)  double getMaxVelocity()

        1)  Return global_v_max_

    G)  bool exists()

        1) Return trajectory_exists_
            

*/




/* NOTES



*/