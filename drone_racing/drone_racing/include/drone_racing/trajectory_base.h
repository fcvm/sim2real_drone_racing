#pragma once

#include <mutex>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "drone_racing/data_saver.h"




namespace drone_racing 
{

    class TrajectoryBase // Abstract class
    {

        public: // Con- and destructor

            TrajectoryBase (
                const ros::NodeHandle &nh, 
                const ros::NodeHandle &pnh
            );

            TrajectoryBase() : TrajectoryBase( ros::NodeHandle(), ros::NodeHandle("~") ) 
            {}

            virtual ~TrajectoryBase();



        public: // Enums

            enum class EnforceEndOfTraj 
            {
                kPos, kPosVel, kPosVelAcc
            };



        public: // Pure virtual methods

            virtual void clearTrajectories( const bool is_mb ) 
            = 0;

            virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory() 
            = 0;

            virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory( const double t ) 
            = 0;

            virtual double getEndTimeFromTrajectory() 
            = 0;

            virtual bool computeTrajectory (
                const quadrotor_common::TrajectoryPoint &start_state,
                const quadrotor_common::TrajectoryPoint &end_state,
                const EnforceEndOfTraj end_constraint,
                const double &desired_speed,
                const bool is_mb
            ) 
            = 0;

            virtual void setBestTrajectory (
                const int traj_idx, 
                const bool is_mb
            ) 
            = 0;



        public: // Methods

            void setStartTime();



        protected: // Member variables

            ros::Time time_start_best_trajectory_;
            // Rapid Trajectory Parameters
            double min_normalized_thrust_;
            double max_normalized_thrust_;
            double max_roll_pitch_rate_;
            double min_traj_sampling_time_;

            double traj_max_z_;
            double traj_min_z_;
            double max_velocity_;



        private: // Methods

            void loadParameters();


        
        private: // Member varibles

            //std::unique_ptr<drone_racing::TrajectoryBase> trajectory_implementation_;
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            //double yaw_trajectory_;

            
    };

} /* namespace drone_racing */






/* DESCRIPTION

When an instance of the class TrajectoryBase is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   nh_                         =   const ros::NodeHandle &nh (input arg)
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &pnh (input arg)
            (double)            yaw_trajectory              =   0.0
            (ros::Time)         time_start_best_trajectory_ instanciated
            (double)            traj_max_z_                 =   {"trajectory_max_z", 2.5}
            (double)            traj_min_z_                 =   {"trajectory_min_z", 0.0}
            (double)            max_velocity_               =   {"max_velocity", 1.0}
            (double)            min_normalized_thrust_      =   {"min_normalized_thrust", 5.0}
            (double)            max_normalized_thrust_      =   {"max_normalized_thrust", 15.0}
            (double)            max_roll_pitch_rate_        =   {"max_roll_pitch_rate", 0.5}
            (double)            min_traj_sampling_time_     =   {"min_trajectory_sampling_time", 0.02}



The following public methods are provided:

    A)  void setStartTime()

            1)  time_start_best_trajectory_ = ros::Time::now();


        
The following methods are pure virtual:

    A)  void clearTrajectories( const bool is_mb )

    B)  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory()

    C)  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory( const double t ) 

    D)  double getEndTimeFromTrajectory()

    E)  bool computeTrajectory (
            const quadrotor_common::TrajectoryPoint &start_state,
            const quadrotor_common::TrajectoryPoint &end_state,
            const EnforceEndOfTraj end_constraint,
            const double &desired_speed,
            const bool is_mb
        )
    
    F)  void setBestTrajectory (
            const int traj_idx, 
            const bool is_mb
        )



*/



/* NOTES

This abstract class gets inherited from class MinimumJerkTrajectory

std::unique_ptr<drone_racing::TrajectoryBase> trajectory_implementation_; not used


*/