#pragma once

#include <mutex>
#include <ros/ros.h>

#include <quadrotor_common/trajectory_point.h>

#include "rapid_trajectories/RapidTrajectoryGenerator.h"
#include "rapid_trajectories/Vec3.h"
#include "drone_racing/trajectory_base.h"



using namespace RapidQuadrocopterTrajectoryGenerator;



namespace drone_racing 
{

    //  Public inheritance: 
    //      the public and protected members of TrajectoryBase 
    //      keep their member access in MinimumJerkTrajectory 
    //      while the private members are inaccessible to MinimumJerkTrajectory 
    class MinimumJerkTrajectory : public TrajectoryBase
    {

        public: // Overridings of all pure virtual functions from TrajectoryBase

            // Input: bool if model based
            // Erase all elements from trajectories_mb_ or trajectories_nw_ according to input arg
            virtual void clearTrajectories( const bool is_mb )
            override;

            // Calculate time: now - time_start_best_trajectory_ + 0.02
            // Fetch position, velocity, acceleration at that time from best_traj_
            // Calculate heading of quadrotor according to the velocity
            // Return whole state (pos ,vel, acc, heading)
            virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory() 
            override;

            // Input: time
            // Like above only time is not calulated but input arg
            virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory( const double t ) 
            override;

            // return best_traj_->GetEndTime()
            virtual double getEndTimeFromTrajectory() 
            override;

            // Input:
            // - start and end state
            // - end constraint: position, position+velocity, position+velocity+acceleration
            // - desired speed
            // - bool if it is model based
            //
            // Calculates a trajectory from start to end state 
            // - end state according to end constraint.
            // - execution time = max distance / {desired speed, start_velocity_norm2 + 0.5, max_velocity_}
            // Check feasibility
            // - input: min_normalized_thrust_, max_normalized_thrust_, max_roll_pitch_rate_, min_traj_sampling_time_
            // - floor/ceiling: traj_min_z_, traj_max_z_
            // If feasibile: Push back trajectory into
            // - trajectories_mb_ or trajectories_nw_ according to bool input arg
            // - return true
            virtual bool computeTrajectory (
                const quadrotor_common::TrajectoryPoint &start_state,
                const quadrotor_common::TrajectoryPoint &end_state,
                const EnforceEndOfTraj end_constraint,
                const double &desired_speed,
                const bool is_mb
            ) 
            override;

            // Input:
            // - trajectory index
            // - bool if model based
            // Set best_traj_ to trajectory at index (input arg) of trajectories_mb_ or trajectories_nw_
            virtual void setBestTrajectory (
                const int traj_idx, 
                const bool is_mb
            ) 
            override;



        private: // Member variables

            std::vector<std::shared_ptr<RapidTrajectoryGenerator> > trajectories_mb_;
            std::vector<std::shared_ptr<RapidTrajectoryGenerator> > trajectories_nw_;
            std::shared_ptr<RapidTrajectoryGenerator> best_traj_;
    };


} 




/* DESCRIPTION

When an instance of the class MinimumJerkTrajectory is created the following happens:
    
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

    B)  void clearTrajectories( const bool is_mb )

            1)  Erase all elements from std::vectors, trajectories_mb_ or trajectories_nw_, according to input bool arg

    C)  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory()

            1)  Return state (pos,vel,acc,heading) from best_traj_ at time, now - time_start_best_trajectory_ + 0.02

    D)  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory( const double t ) 

            1)  Return state (pos,vel,acc,heading) from best_traj_ at input time arg

    E)  double getEndTimeFromTrajectory()

            1)  return best_traj_->GetEndTime()

    F)  bool computeTrajectory (
            const quadrotor_common::TrajectoryPoint &start_state,
            const quadrotor_common::TrajectoryPoint &end_state,
            const EnforceEndOfTraj end_constraint,
            const double &desired_speed,
            const bool is_mb
        )

            1)  Calculates a trajectory from start to end state (input args)
                    - end state according to end constraint (input arg).
                    - execution time = max distance / {desired speed (input arg), start state (input arg) velocity norm2 + 0.5, max_velocity_}
            2)  Check feasibility of trajectory
                    - of input by using: min_normalized_thrust_, max_normalized_thrust_, max_roll_pitch_rate_, min_traj_sampling_time_
                    - of position - floor/ceiling by using: traj_min_z_, traj_max_z_
            3)  If feasibile: Push back trajectory into
                    - trajectories_mb_ or trajectories_nw_ according to bool input arg
                and return true
    
    G)  void setBestTrajectory (
            const int traj_idx, 
            const bool is_mb
        )
            1)  Set best_traj_ to trajectory at index (input arg) of trajectories_mb_ or trajectories_nw_ according to bool input arg



*/



/* NOTES

class MinimumJerkTrajectory fully implemented not abstract

try to not use shared ptrs and do all on the stack

I will try to avoid to take a look at the implementation of RapidTrajectoryGenerator and hope i can just use it


*/