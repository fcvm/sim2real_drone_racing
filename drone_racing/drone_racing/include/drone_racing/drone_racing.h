#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <kindr/minimal/quat-transformation.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include "drone_racing/data_saver.h"
#include "drone_racing/gate_replacer.h"
#include "drone_racing/gazebo_rviz_visualizer.h"
#include "drone_racing/global_trajectory.h"
#include "drone_racing/trajectory_manager.h"
#include "drone_racing/visualizer.h"



using namespace RapidQuadrocopterTrajectoryGenerator;



namespace rpg 
{

    typedef kindr::minimal::QuatTransformation Pose;
    typedef kindr::minimal::RotationQuaternion Rotation;
  	typedef Eigen::Vector3d Position;

}




namespace drone_racing { class DroneRacing 
{


    public: // Con- and destructors

        DroneRacing (
            const ros::NodeHandle& nh, 
            const ros::NodeHandle& pnh
        );

        DroneRacing() : DroneRacing(ros::NodeHandle(), ros::NodeHandle("~")) {}

        virtual ~DroneRacing();


    public: // Methods

        Eigen::Vector3d getEstimatedPos();


    public: // Shared Ptrs member variables

        std::shared_ptr<Visualizer>                                     visualizer_;
        std::shared_ptr<DataSaver>                                      data_saver_;
        std::shared_ptr<TrajectoryManager>                              trajectory_manager_;
        std::shared_ptr<GlobalTrajectory>                               global_trajectory_;
        std::shared_ptr<gate_replacer::GateReplacer>                    gate_replacer_;
        std::shared_ptr<gazebo_rviz_visualizer::GazeboRvizVisualizer>   gazebo_rviz_visualizer_;


    private: // Enums
      
        enum class State 
        {
            kOff, kHover, kWaitingForFeedthroughActivated, kRacing
        };


    private: // Methods

        void mainloop( const ros::TimerEvent& time );

        void updateStateMachine();

        void performNavigation();

        void publishDesiredState();

        bool setGoalWasTriggered() const;

        bool checkStateEstDesStateDivergence();

        void resetDesiredState();

        void loadParameters();

        void transformToQuadFrame();

        void transformToWorldFrame();

        void transformStateEstimateToWorldFrame();

        quadrotor_common::TrajectoryPoint getEndState();

        double getDesiredVelocity();

        void updateWaypointGoals();

        double constrain (
            const double& prev_value, 
            const double& new_value, 
            const double& threshold
        );

        void executeTestTime();

        void collectData();

    
    private: // Callback methods

        void startNavigationCallback( const std_msgs::EmptyConstPtr& msg );

        void hardStopCallback( const std_msgs::EmptyConstPtr& msg );

        void stateEstimateCallback( const nav_msgs::OdometryConstPtr& msg );

        void networkCallback( const geometry_msgs::TwistStampedConstPtr& msg );

        void enableTestTimeCallback( const std_msgs::BoolConstPtr& msg );

        void setRunIdxCallback( const std_msgs::Int16ConstPtr& msg );
        
        void setupEnvironmentCallback( const std_msgs::EmptyConstPtr& msg );

        
    private: // ROS member variables
        
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber copilot_feedback_sub_;
        ros::Subscriber state_estimate_sub_;
        ros::Subscriber feedthrough_sub_;
        ros::Subscriber start_navigation_sub_;
        ros::Subscriber soft_stop_sub_;
        ros::Subscriber hard_stop_sub_;
        ros::Subscriber cnn_sub_;
        ros::Subscriber test_time_sub_;
        ros::Subscriber run_idx_sub_;
        ros::Subscriber env_setup_sub_;

        ros::Publisher crashed_pub_;
        ros::Publisher passed_gate_pub_;
        ros::Publisher feedthrough_pub_;
        ros::Publisher desired_state_pub_;
        ros::Publisher divergence_pub_;

        ros::Timer main_loop_timer_;
        ros::WallTime start_time_;

        tf::TransformListener tf_listener_;


    private: // Member variables

        Eigen::Vector3d network_selection_ = Eigen::Vector3d(0.0, 0.0, 1.0);

        rpg::Pose T_S_Q_;
        rpg::Pose T_W_Q_;
        rpg::Pose T_W_S_;

        quadrotor_common::TrajectoryPoint desired_state_quad_;
        quadrotor_common::TrajectoryPoint desired_state_world_;

        nav_msgs::Odometry state_estimate_quad_;
        nav_msgs::Odometry state_estimate_world_;
        bool environment_is_set_up_;
        State state_machine_;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > goal_positions_;
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > gate_positions_;

        bool goal_selected_;
        int failed_trials_;
        bool moving_gates_;
        int curr_goal_index_ = 0;
        int last_goal_index_ = 0;
        double d_replan_;
        double camera_semi_fov_rad_yaw_;
        double camera_semi_fov_rad_pitch_;
        double max_velocity_;
        double max_divergence_;
        int max_failed_trials_;
        bool test_time_ = false;
        int mainloop_iter_;
        int plan_every_n_iter_;
        int asked_teacher_n_times_ = 0;
        bool directory_created_ = false;
        bool record_data_;
        int model_based_frame_;
        int executed_nn_traj_;
        double sec_no_record_at_start_;
        double horizon_min_;
        double prev_des_velocity_;
        bool perturb_actions_;
        double gates_static_amp_;

        


};
} // namespace drone_racing










/* DESCRIPTION

When an instance of the class DroneRacing is created the following happens:
    
    1)  Several member variables are set
            (ros::NodeHandle)   nh_                         =   const ros::NodeHandle &nh (input arg)
            (ros::NodeHandle)   pnh_                        =   const ros::NodeHandle &nh_private (input arg)
            (State)             state_machine_              =   State::kOff
            (bool)              goal_selected_              =   false
            (int)               failed_trials_              =   0
            (int)               model_based_frame_          =   0
            (int)               executed_nn_traj_           =   0
            (bool)              environment_is_set_up_      =   false

        Several member variables are fetched from ROS param server
            (int)               plan_every_n_iter_              =   {"plan_every_nth_iteration",        1       }
            (double)            max_velocity_                   =   {"max_velocity",                    1.0     }
            (double)            max_divergence_                 =   {"max_divergence",                  0.2     }
            (double)            d_replan_                       =   {"d_replan",                        1.0     }
            (double)            camera_semi_fov_rad_yaw_        =   {"camera_fov_yaw",                  45.0    } * pi/180
            (double)            camera_semi_fov_rad_pitch_      =   {"camera_fov_pitch",                45.0    } * pi/180
            (bool)              moving_gates_                   =   {"moving_gates",                    false   }
            (double)            gates_static_amp_               =   {"gates_static_amplitude",          0.0     }
            (int)               max_failed_trials_              =   {"max_failed_trials",               5       }
            (double)            sec_no_record_at_start_         =   {"wait_n_sec_till_record",          0.0     }
            (bool)              record_data_                    =   {"record_data",                     false   }
            (bool)              perturb_actions_                =   {"perturb_actions",                 false   }
            (int)               curr_goal_index_                =   {"curr_goal_idx",                   0       }
            (double)            gate_height_                    =   {"gate_height",                     1.0     }
            (double)            horizon_min_                    =   {"horizon_min",                     1.0     }

        Several member variables are default-initialized
            (TrajectoryPoint)   desired_state_quad_
            (TrajectoryPoint)   desired_state_world_
            (Odometry)          state_estimate_quad_
            (Odometry)          state_estimate_world_
            (TransformListener) tf_listener_

    2)  Publishers are initialized to the following ROS topics
            "/crashed"                                  (std_msgs::Empty)                              
            "/passed_gate"                              (std_msgs::Empty)                              
            "copilot/feedthrough"                       (std_msgs::Bool)                               
            "autopilot/reference_state"                 (quadrotor_msgs::TrajectoryPoint)                  
            "divergence"                                (std_msgs::Float64)                    

    3)  Subscribers are initialized to the following ROS topics
            A)  "state_estimate"                        (nav_msgs::Odometry)
            B)  "start_navigation"                      (std_msgs::Empty)
            C)  "hard_stop"                             (std_msgs::Empty)
            D)  "/cnn_out/traj"                         (geometry_msgs::TwistStamped)
            E)  "only_network"                          (std_msgs::Bool)
            F)  "run_idx"                               (std_msgs::Int16)
            G)  "setup_environment"                     (std_msgs::Empty)
        
        Their callbacks do the following:
            A)  state_estimate_quad_    =   message 
                T_S_Q_                  =   message
            B)  goal_selected           =   true
                start_time_             =   now
                resetDesiredState()
                    desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                        All members equal to zero except
                            position = state_estimate_quad_.pose.pose.position
                            heading = state_estimate_quad_.pose.pose.orientation
                    transformToWorldFrame()
                        desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                publishDesiredState()
                    desired_state_quad_ -> "autopilot/reference_state"
                true -> "copilot/feedthrough"
            C)  asked_teacher_n_times_, executed_nn_traj_, model_based_frame_       = 0
                goal_selected_ = false
                data_saver_->stopRecording()
                resetDesiredState()
                    desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                        All members equal to zero except
                            position = state_estimate_quad_.pose.pose.position
                            heading = state_estimate_quad_.pose.pose.orientation
                    transformToWorldFrame()
                        desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                publishDesiredState()
                    desired_state_quad_ -> "autopilot/reference_state"
                state_machine_ = State::kHover
            D)  network_selection_      =   linear twist (x,y,z) of message
                data_saver_->setNetworkSelection( network_selection_ )
            E)  test_time_ = message
            F)  directory_created_ = false
            G)  (int)       curr_goal_index_    =   {curr_goal_idx, 0}
                environment_is_set_up_      = false
                clear goal_positions_.clear and gate_positions_
                gate_replacer_->replaceGatesExplicit()
                goal_positions_ = gate_replacer_->getGoalPositions()
                gate_positions_ = gate_replacer_->getGatePositions()
                gazebo_rviz_visualizer_->visualizeGates()
                global_trajectory_->generateGlobalTrajectory( visualizer_, goal_positions_ )
                if test_time_ == true
                    getEstimatedPos()
                        T_S_Q_ = state_estimate_quad_
                        T_W_S_ = neutral rotation
                        T_W_Q_ = T_S_Q_
                        visualizer_->displayDebug ( T_W_Q_.getPosition(), 4, visualizer_->Color::kGreen )
                        return T_W_Q_.getPosition()
                else 
                    global_trajectory_->findStartIndexGlobalTrajectory( getEstimatedPos() )
                data_saver_->stopRecording();
                environment_is_set_up_      = true
                goal_selected_              = true
                start_time_                 = now
                resetDesiredState()
                    desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                        All members equal to zero except
                            position = state_estimate_quad_.pose.pose.position
                            heading = state_estimate_quad_.pose.pose.orientation
                    transformToWorldFrame()
                        desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                publishDesiredState()
                    desired_state_quad_ -> "autopilot/reference_state"
                true -> "copilot/feedthrough"
                set last_goal_index_ depending to curr_goal_index_

    4)  Reset shared pointers
            data_saver_.reset             ( new DataSaver()                                   );
            visualizer_.reset             ( new Visualizer()                                  );
            trajectory_manager_.reset     ( new TrajectoryManager( pnh_ )                     );
            global_trajectory_.reset      ( new GlobalTrajectory( pnh_ )                      );
            gate_replacer_.reset          ( new gate_replacer::GateReplacer                   );
            gazebo_rviz_visualizer_.reset ( new gazebo_rviz_visualizer::GazeboRvizVisualizer  );

    5)  Timer with callback at specific rate
            mainloop()                              [every 0.02 seconds]
                visualizer_->displayQuadrotor()
                if environment_is_set_up_ == true
                    if moving_gates_ == true and sec_no_record_at_start_ passed
                        gate_replacer_->makeGatesMove()
                    gazebo_rviz_visualizer_->visualizeGates()
                    updateStateMachine()
                        if state_machine_ == kOff or kHover
                            visualizer_->displayGoalMarker( goal_positions_ )
                            if goal_selected_ == true
                                global_trajectory_->findStartIndexGlobalTrajectory( getEstimatedPos() )
                                global_trajectory_->findStartIndexNWGlobalTrajectory( this )
                                mainloop_iter_ = 0
                                prev_des_velocity_ = 0.0
                                state_machine_ = kRacing
                        if state_machine_ = kRacing
                            performNavigation()
                                (x=) norm(desired_state_world_ - state_estimate_world_) -> "divergence"
                                if x > max_divergence_
                                    goal_selected_ = false
                                    resetDesiredState()
                                        desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                                            All members equal to zero except
                                                position = state_estimate_quad_.pose.pose.position
                                                heading = state_estimate_quad_.pose.pose.orientation
                                        transformToWorldFrame()
                                            desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                                    publishDesiredState()
                                        desired_state_quad_ -> "autopilot/reference_state"
                                    state_machine_ = kHover
                                    data_saver_->labelAsBadData()
                                    data_saver_->stopRecording()
                                    std_msgs::Empty() -> "/crashed"
                                else
                                    if test_time_: executeTestTime()
                                        if mainloop_iter_ % plan_every_n_iter_ == 0
                                            bool success_nw = trajectory_manager_->computeTrajectory (this, desired_state_world_, state_estimate_world_, network_selection_)
                                            updateWaypointGoals()
                                                visualizer_->displayDebug() for
                                                    - desired_state_world_ in kRed
                                                    - global_trajectory_->projectOnTrajectory( desired_state_world_ ) in kBlue
                                                    - goal_positions_.at(curr_goal_index_) in kBlack
                                                dist_to_next_gate = desired_state_world_.position - goal_positions_.at(curr_goal_index_)
                                                if norm( dist_to_next_gate ) > d_replan_
                                                    curr_goal_index_++ (with % check)
                                                    last_goal_index_ = curr_goal_index_-1 (with % check)
                                                    std_msgs::Empty() -> "/passed_gate"
                                            if success_nw
                                                trajectory_manager_->setBestTraj ( 0, false )
                                                model_based_frame_ = 0
                                                executed_nn_traj_++
                                            trajectory_manager_->clearTrajectories( true )
                                            trajectory_manager_->clearTrajectories( false )
                                            if success_nw
                                                trajectory_manager_->visualizeTrajectory( this )
                                                failed_trials_ = 0#
                                                trajectory_manager_->setStartTime()
                                            else
                                                if failed_trials_ < max_failed_trials_
                                                    failed_trials_++
                                                else
                                                    resetDesiredState()
                                                        desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                                                            All members equal to zero except
                                                                position = state_estimate_quad_.pose.pose.position
                                                                heading = state_estimate_quad_.pose.pose.orientation
                                                        transformToWorldFrame()
                                                            desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                                                    publishDesiredState()
                                                        desired_state_quad_ -> "autopilot/reference_state"
                                                    state_machine_ = kHover
                                                    data_saver_->labelAsBadData()
                                                    data_saver_->stopRecording()
                                                    std_msgs::Empty() -> "/crashed"
                                    else: collectData()
                                        if mainloop_iter_ % plan_every_n_iter_ == 0
                                            updateWaypointGoals():
                                                visualizer_->displayDebug() for
                                                    - desired_state_world_ in kRed
                                                    - global_trajectory_->projectOnTrajectory( desired_state_world_ ) in kBlue
                                                    - goal_positions_.at(curr_goal_index_) in kBlack
                                                dist_to_next_gate = desired_state_world_.position - goal_positions_.at(curr_goal_index_)
                                                if norm( dist_to_next_gate ) > d_replan_
                                                    curr_goal_index_++ (with % check)
                                                    last_goal_index_ = curr_goal_index_-1 (with % check)
                                                    std_msgs::Empty() -> "/passed_gate"
                                            desired_velocity_mb = getDesiredVelocity():
                                                next_des_state_mb = global_trajectory_->getNextStateOnGlobalTraj( 0.5 )
                                                return next_des_state_mb.velocity.norm() / global_trajectory_->getMaxVelocity()
                                            end_state = getEndState():
                                                dist_to_next_gate = norm ( desired_state_world_ - goal_positions_.at(curr_goal_index_) )
                                                dist_to_last_gate = norm ( desired_state_world_ - goal_positions_.at(last_goal_index_) )
                                                horizon =  min( dist_to_next_gate, dist_to_last_gate ) but at least = horizon_min_
                                                end_state = global_trajectory_->getNextStateOnGlobalTraj( horizon )
                                                if moving_gates_ or gates_static_amp_ > 0.0
                                                    end_state.position = goal_positions_.at(curr_goal_index_)
                                                visualizer_->displayDebug( end_state, 0, visualizer_->Color::kYellow )
                                                return end_state
                                            v_rescaled = desired_velocity_mb * max_velocity_
                                            v_execute = min (  desired_state_world_.velocity.norm() + 0.5, v_rescaled )
                                            v_execute = constrain( prev_des_velocity_, v_execute, 0.02):
                                                v_execute \in [ prev_des_velocity_ - 0.02, prev_des_velocity_ + 0.02 ]
                                            prev_des_velocity_ = v_execute
                                            T_W_goal = end_state
                                            T_W_B = state_estimate_world_
                                            T_B_goal = T_W_B.inverse() * T_W_goal
                                            yaw_to_end = arctan (T_B_goal.getPosition().y() / T_B_goal.getPosition().x())
                                            traj_coor_right = -yaw_to_end / camera_semi_fov_rad_yaw_ but at least in [-1, 1]
                                            dist_to_goal = T_B_goal.getPosition().norm()
                                            pitch_to_end = = arctan (T_B_goal.getPosition().z() / dist_to_goal)
                                            traj_coor_up = pitch_to_end / camera_semi_fov_rad_pitch_ but at least in [-1 ,1]
                                            DataSaver::trajectory_info traj_info
                                                - idx_right = traj_coor_right
                                                - idx_up = traj_coor_up
                                                - curr_vel = desired_velocity_mb
                                                - max_vel = global_trajectory_->getMaxVelocity()
                                            data_saver_->setModelBasedPrediction( traj_info.idx_right, traj_info.idx_up, traj_info.curr_vel )
                                            if perturb_actions_
                                                random_perturbation = random 3d vec in +/-[0.1, 0.1, 0.05]
                                            bool success = trajectory_manager_->computeTrajectory (this, desired_state_world_, state_estimate_world_, Eigen::Vector3d ( traj_info.idx_right, traj_info.idx_up, traj_info.curr_vel ) + random_perturbation )
                                            if ( ros::WallTime::now() - start_time_ ).toSec() > sec_no_record_at_start_
                                                if record_data_
                                                    if not directory_created_
                                                        data_saver_->createDirectory()
                                                        directory_created_ = true
                                                    data_saver_->startRecording()
                                            data_saver_->saveImage ( traj_info, model_based_frame_ )
                                            if success
                                                trajectory_manager_->setBestTraj ( 0, false )
                                            asked_teacher_n_times_++, model_based_frame_++
                                            if success
                                                trajectory_manager_->visualizeTrajectory( this )
                                                failed_trials_ = 0
                                                trajectory_manager_->setStartTime()
                                            else
                                                if failed_trials_ < max_failed_trials_
                                                    failed_trials_++
                                                else
                                                    resetDesiredState()
                                                        desired_state_quad_ = quadrotor_common::TrajectoryPoint()
                                                            All members equal to zero except
                                                                position = state_estimate_quad_.pose.pose.position
                                                                heading = state_estimate_quad_.pose.pose.orientation
                                                        transformToWorldFrame()
                                                            desired_state_world_ = desired_state_quad_ ??? because T_W_S_ is neutral rotation???
                                                    publishDesiredState()
                                                        desired_state_quad_ -> "autopilot/reference_state"
                                                    data_saver_->stopRecording()
                                                    state_machine_ = State::kHover
                                                    goal_selected_ = false
                            mainloop_iter_++
                    switch state_machine_
                        kOff, kHover, kWaitingForFeedthroughActivated: break
                        kRacing: 
                            desired_state_world_ = trajectory_manager_->getDesiredStateFromTrajectory()
                            transformToQuadFrame():
                                desired_state_quad_ <- desired_state_world_
                            publishDesiredState()
                                desired_state_quad_ -> "autopilot/reference_state"
                            break
                    if (ros::WallTime::now() - start_mainloop).toSec() > 0.02
                        ROS_ERROR







*/




/* NOTES

member variables and member parameters (constant)




*/